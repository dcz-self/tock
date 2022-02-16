//! hil driver for Bmp280 Temperature and Pressure Sensor
//!
//! Based off the SHT3x code.
//!
//! Not implemented: pressure
//!
//! TODO: convert to calibrated, check status, sleep before checking status

use core::cell::Cell;
use kernel::debug;
use kernel::hil;
use kernel::hil::i2c;
use kernel::hil::time::{Alarm, ConvertTicks};
use kernel::utilities::cells::{OptionalCell, TakeCell};
use kernel::ErrorCode;

pub static BASE_ADDR: u8 = 0x76;

/// Currently includes temperature only.
/// This is the biggest buffer size in this module.
pub const CALIBRATION_BYTES: usize = 6;

#[allow(non_camel_case_types)]
#[allow(dead_code)]
enum Register {
    /// First register of calibration data.
    /// Each register is 2 bytes long.
    DIG_T1 = 0x88,
    DIG_T2 = 0x8a,
    DIG_T3 = 0x8c,
    ID = 0xd0,
    RESET = 0xe0,
    /// measuring: [3]
    /// im_update: [0]
    STATUS = 0xf3,
    /// osrs_t: [7:5]
    /// osrs_p: [4:2]
    /// mode: [1:0]
    CTRL_MEAS = 0xf4,
    /// t_sb: [7:5]
    /// filter: [4:2]
    /// spi3w_en: [0]
    CONFIG = 0xf5,
    PRESS_MSB = 0xf7,
    PRESS_LSB = 0xf8,
    /// xlsb: [7:4]
    PRESS_XLSB = 0xf9,
    TEMP_MSB = 0xfa,
    TEMP_LSB = 0xfb,
    /// xlsb: [7:4]
    TEMP_XLSB = 0xfc,
}

#[derive(Clone, Copy, PartialEq, Debug)]
struct CalibrationData {
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,
    // TODO: pressure calibration
}

/// CAUTION: calibration data puts least significant byte in the lowest address,
/// readouts do the opposite.
fn twobyte(lsb: u8, msb: u8) -> u16 {
    ((msb as u16) << 8) + lsb as u16
}

impl CalibrationData {
    fn new(i2c_raw: &[u8]) -> Self {
        CalibrationData {
            dig_t1: twobyte(i2c_raw[0], i2c_raw[1]) as u16,
            dig_t2: twobyte(i2c_raw[2], i2c_raw[3]) as i16,
            dig_t3: twobyte(i2c_raw[4], i2c_raw[5]) as i16,
        }
    }
    
    fn temp_from_raw(&self, raw_temp: u32) -> i32 {
        debug!("raw temp: {}", raw_temp);
        let temp = raw_temp as i32; // guaranteed to succeed because raw temp has only 20 significant bits maximum.
        let dig_t1 = self.dig_t1 as i32; // same, 16-bits
        let dig_t2 = self.dig_t2 as i32; // same, 16-bits
        let dig_t3 = self.dig_t3 as i32; // same, 16-bits
        // From the datasheet
        let var1 = (((temp >> 3) - (dig_t1 << 1)) * dig_t2) >> 11;
        let a = (temp >> 4) - dig_t1;
        let var2 = (((a * a) >> 12) * dig_t3) >> 14;
        let t_fine = var1 + var2;
        ((t_fine * 5) + 128) >> 8
    }
}

/// Internal state.
/// Each state can lead to the next on in order of appearance.
#[derive(Clone, Copy, PartialEq, Debug)]
enum State {
    Uninitialized,
    InitId,
    /// It's not guaranteed that the MCU reset is the same as device power-on,
    /// so an explicit reset is necessary.
    InitResetting,
    InitWaitingReady,
    InitReadingCalibration,

    Idle(CalibrationData),

    // States related to sample readout
    /// One-shot mode request sent
    Configuring(CalibrationData),
    /// Sampling takes milliseconds, so spend most of that time sleeping.
    WaitingForAlarm(CalibrationData),
    /// Polling for the readout to become ready.
    Waiting(CalibrationData),
    /// Waiting for readout to return.
    /// This state can also lead back to Idle.
    Reading(CalibrationData),
    
    /// The ID is not matching. Reset cannot be attempted.
    MismatchedDevice,
    /// Irrecoverable. Currently only when init fails.
    Error,
}

pub struct Bmp280<'a, A: Alarm<'a>> {
    i2c: &'a dyn i2c::I2CDevice,
    temperature_client: OptionalCell<&'a dyn hil::sensors::TemperatureClient>,
    // This might be better as a `RefCell`,
    // because `State` is multiple bytes due to the `CalibrationData`.
    // `Cell` requires Copy, which might get expensive, while `RefCell` doesn't.
    // It's probably not a good idea to split `CalibrationData`
    // into a separate place, because it will make state more duplicated.
    state: Cell<State>,
    /// Stores i2c commands
    buffer: TakeCell<'static, [u8]>,
    /// Needed to wait for readout completion, which can take milliseconds.
    /// It's possible to implement this without an alarm with busy polling, but that's wasteful.
    alarm: &'a A,
}

impl<'a, A: Alarm<'a>> Bmp280<'a, A> {
    pub fn new(
        i2c: &'a dyn i2c::I2CDevice,
        buffer: &'static mut [u8],
        alarm: &'a A,
    ) -> Self {
        Self {
            i2c: i2c,
            temperature_client: OptionalCell::empty(),
            state: Cell::new(State::Uninitialized),
            buffer: TakeCell::new(buffer),
            alarm: alarm,
        }
    }
    
    /// Resets the device and brings it into a known state.
    pub fn begin_reset(&self) -> Result<(), ErrorCode> {
        debug!("init: {:?}", self.state.get());
        self.buffer.take().map_or_else(
            || panic!("BMP280 No buffer available!"),
            |buffer| {
                match self.state.get() {
                    State::Uninitialized | State::Error => {
                        self.i2c.enable();
                        self.i2c_read::<1>(buffer, Register::ID);
                        self.state.set(State::InitId);
                        Ok(())
                    },
                    State::MismatchedDevice => Err(ErrorCode::NODEVICE),
                    _ => Err(ErrorCode::ALREADY),
                }
            }
        )
    }

    pub fn read_temperature(&self) -> Result<(), ErrorCode> {
        use State::*;
        debug!("read temp: {:?}", self.state.get());
        match self.state.get() {
            // Actually, the sensor might be on, just in default state.
            Uninitialized => Err(ErrorCode::OFF),
            InitId | InitResetting | InitWaitingReady | InitReadingCalibration => Err(ErrorCode::BUSY),
            Idle(calibration) => self.buffer.take().map_or_else(
                || panic!("BMP280 No buffer available!"),
                |buffer| {
                    self.i2c.enable();
                    // todo: use bitfield crate
                    // forced mode, oversampling 1
                    let val = 0b00100001;
                    self.i2c_write(buffer, Register::CTRL_MEAS, [val]);
                    self.state.set(State::Configuring(calibration));
                    Ok(())
                },
            ),
            Configuring(_) | WaitingForAlarm(_) | Waiting(_) | Reading(_) => Err(ErrorCode::BUSY),
            Error => Err(ErrorCode::FAIL),
            MismatchedDevice => Err(ErrorCode::NODEVICE),
        }
    }
    
    fn handle_alarm(&self) {
        use State::*;
        match self.state.get() {
            WaitingForAlarm(calibration) => self.buffer.take().map_or_else(
                || panic!("BMP280 No buffer available!"),
                |buffer| {
                    self.i2c.enable();
                    self.check_ready(buffer);
                    self.state.set(State::Waiting(calibration))
                },
            ),
            _ => {}
        }
    }
    
    fn arm_alarm(&self) {
        // Datasheet says temp oversampling=1 makes a reading typically take 5.5ms.
        // (Maximally 6.4ms).
        let delay = self.alarm.ticks_from_us(6400);
        self.alarm.set_alarm(self.alarm.now(), delay);
    }

    fn i2c_write<const COUNT: usize>(&self, buffer: &'static mut [u8], addr: Register, data: [u8; COUNT]) {
        buffer[0] = addr as u8;
        for (i, d) in data.iter().enumerate() {
            buffer[i + 1] = *d;
        }
        // TODO: propagate errors
        self.i2c.write(buffer, COUNT as u8 + 1).unwrap();
    }
    
    fn i2c_read<const COUNT: u8>(&self, buffer: &'static mut [u8], addr: Register) {
        buffer[0] = addr as u8;
        // TODO: propagate errors
        self.i2c.write_read(buffer, 1, COUNT).unwrap();
    }
    
    fn read_i2c(&self, buffer: &'static mut [u8], addr: Register, count: u8) {
        buffer[0] = addr as u8;
        // TODO: propagate errors
        self.i2c.write_read(buffer, 1, count).unwrap();
    }

    fn i2c_parse_read<const COUNT: u8>(buffer: &[u8]) -> &[u8] {
        &buffer[..(COUNT as usize)]
    }

    fn parse_read_i2c(buffer: &[u8], count: u8) -> &[u8] {
        &buffer[..(count as usize)]
    }
    
    fn check_ready(&self, buffer: &'static mut [u8]) {
        self.read_i2c(buffer, Register::STATUS, 1)
    }
}

impl<'a, A: Alarm<'a>> i2c::I2CClient for Bmp280<'a, A> {
    fn command_complete(&self, buffer: &'static mut [u8], status: Result<(), i2c::Error>) {
        const INVALID_TEMPERATURE: i32 = i32::MIN;
        //debug!("i2c_reply: {:?}", self.state.get());
        let mut return_buffer = None;
        let mut temp_readout = None;

        let new_state = match status {
            Ok(()) => match self.state.get() {
                State::InitId => {
                    let id = Self::i2c_parse_read::<1>(buffer);
                    debug!("id: {:b}", id[0]);
                    if id[0] == 0x58 {
                        self.i2c_write(buffer, Register::RESET, [0xb6]);
                        State::InitResetting
                    } else {
                        State::MismatchedDevice
                    }
                }
                State::InitResetting => {
                    self.check_ready(buffer);
                    State::InitWaitingReady
                },
                State::InitWaitingReady => {
                    let waiting = Self::parse_read_i2c(buffer, 1)[0];
                    debug!("waiting: {:b}", waiting);
                    if waiting & 0b1 == 0 {
                        // finished init
                        self.i2c_read::<{CALIBRATION_BYTES as u8}>(buffer, Register::DIG_T1);
                        State::InitReadingCalibration
                    } else {
                        self.check_ready(buffer);
                        State::InitWaitingReady
                    }
                },
                State::InitReadingCalibration => { 
                    let data = Self::parse_read_i2c(buffer, CALIBRATION_BYTES as u8);
                    let calibration = CalibrationData::new(data);
                    return_buffer = Some(buffer);
                    State::Idle(calibration)
                },
                // Readout-related states
                State::Configuring(calibration) => {
                    return_buffer = Some(buffer);
                    self.arm_alarm();
                    State::WaitingForAlarm(calibration)
                },
                State::Waiting(calibration) => {
                    let waiting_value = Self::parse_read_i2c(buffer, 1);
                    //debug!("waiting: {:b}", waiting_value[0]);
                    // not waiting
                    if waiting_value[0] & 0b1000 == 0 {
                        self.read_i2c(buffer, Register::TEMP_MSB, 3);
                        State::Reading(calibration)
                    } else {
                        self.check_ready(buffer);
                        State::Waiting(calibration)
                    }
                }
                State::Reading(calibration) => {
                    let readout = Self::parse_read_i2c(buffer, 3);
                    let msb = readout[0] as u32;
                    let lsb = readout[1] as u32;
                    let xlsb = readout[2] as u32;
                    let raw_temp = (msb << 12) + (lsb << 4) + (xlsb >> 4);
                    return_buffer = Some(buffer);
                    temp_readout = Some(calibration.temp_from_raw(raw_temp));
                    State::Idle(calibration)
                },
                other => {
                    debug!("BMP280 received unexpected i2c reply in state {:?}", other);
                    return_buffer = Some(buffer);
                    other
                },
            },
            Err(_i2c_err) => {
                return_buffer = Some(buffer);
                match self.state.get() {
                    State::Configuring(calibration) | State::Waiting(calibration) | State::Reading(calibration) => {
                        temp_readout = Some(INVALID_TEMPERATURE);
                        State::Idle(calibration)
                    },
                    State::InitId | State::InitResetting | State::InitWaitingReady | State::InitReadingCalibration
                        => State::Error,
                    other => {
                        debug!("BMP280 received unexpected i2c reply in state {:?}", other);
                        other
                    },
                }
            },
        };

        if let Some(buffer) = return_buffer {
            self.i2c.disable();
            self.buffer.replace(buffer);
        }
        // Setting state before the callback,
        // in case the callback wants to use the same driver again.
        self.state.set(new_state);
        if let Some(temp) = temp_readout {
            debug!("temp {}", temp);
            self.temperature_client
                .map(|cb| cb.callback(temp as usize));
        }
        //debug!("new state: {:?}", self.state.get());
    }
}

impl<'a, A: Alarm<'a>> hil::sensors::TemperatureDriver<'a> for Bmp280<'a, A> {
    fn set_client(&self, client: &'a dyn hil::sensors::TemperatureClient) {
        self.temperature_client.set(client);
    }

    fn read_temperature(&self) -> Result<(), ErrorCode> {
        self.read_temperature()
    }
}

impl<'a, A: hil::time::Alarm<'a>> hil::time::AlarmClient for Bmp280<'a, A> {
    fn alarm(&self) {
        self.handle_alarm();
    }
}
