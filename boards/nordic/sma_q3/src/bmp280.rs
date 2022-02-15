//! hil driver for Bmp280 Temperature and Pressure Sensor
//!
//! Based off the SHT3x code.
//!
//! Not implemented: pressure
//!
//! TODO: convert to calibrated, check status, sleep before checking status

use core::cell::Cell;
use kernel::debug;
use kernel::hil::i2c;
use kernel::hil::time::Alarm;
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
    /// measuring: [3] (or is it really [2]? bit 2 is set on real hardware)
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

fn twobyte(msb: u8, lsb: u8) -> u16 {
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
        // FIXME
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
    InitConfiguring,
    InitReadingCalibration,
    Idle(CalibrationData),
    /// A request is in flight, but has not been returned to userspace yet.
    /// Waiting for the readout to become ready.
    Waiting(CalibrationData),
    /// Waiting for data to return.
    /// This state can also lead back to Idle.
    Reading(CalibrationData),
}

pub struct Bmp280<'a, A: Alarm<'a>> {
    i2c: &'a dyn i2c::I2CDevice,
    temperature_client: OptionalCell<&'a dyn kernel::hil::sensors::TemperatureClient>,
    // This might be better as a `RefCell`,
    // because `State` is multiple bytes due to the `CalibrationData`.
    // `Cell` requires Copy, which might get expensive, while `RefCell` doesn't.
    // It's probably not a good idea to split `CalibrationData`
    // into a separate place, because it will make state more duplicated.
    state: Cell<State>,
    /// Stores i2c commands
    buffer: TakeCell<'static, [u8]>,
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

    pub fn read_temperature(&self) -> Result<(), ErrorCode> {
        use State::*;
        debug!("read temp: {:?}", self.state.get());
        match self.state.get() {
            // Actually, the sensor might be on, just in default state.
            Uninitialized => Err(ErrorCode::OFF),
            InitConfiguring | InitReadingCalibration => Err(ErrorCode::BUSY),
            Idle(calibration) => self.buffer.take().map_or_else(
                || panic!("BMP280 No buffer available!"),
                |buffer| {
                    self.state.set(State::Waiting(calibration));
                    self.i2c.enable();
                    self.check_ready(buffer);
                    Ok(())
                }
            ),
            Reading(_calibration) | Waiting(_calibration) => Err(ErrorCode::BUSY),
            _ => Ok(()),
        }
    }
    
    pub fn begin_initialize(&self) -> Result<(), ErrorCode> {
        debug!("init: {:?}", self.state.get());
        self.buffer.take().map_or_else(
            || panic!("BMP280 No buffer available!"),
            |buffer| {
                match self.state.get() {
                    State::Uninitialized => {
                        self.i2c.enable();
                        self.i2c_read::<1>(buffer, Register::ID);
                        self.state.set(State::InitId);
                        Ok(())
                    },
                    _ => Err(ErrorCode::ALREADY),
                }
            }
        )
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
        self.i2c_read::<1>(buffer, Register::STATUS);
    }
}

impl<'a, A: Alarm<'a>> i2c::I2CClient for Bmp280<'a, A> {
    fn command_complete(&self, buffer: &'static mut [u8], status: Result<(), i2c::Error>) {
        debug!("i2c_reply: {:?}", self.state.get());
        match status {
            Ok(()) => {
                let (new_state, temp, buffer) = match self.state.get() {
                    State::InitId => {
                        let id = Self::i2c_parse_read::<1>(buffer);
                        debug!("id: {:b}", id[0]);
                        if id[0] != 0x58 {
                            // TODO: go into error state instead
                            panic!("BMP280: wrong id");
                        }
                        self.i2c_write(buffer, Register::RESET, [0xb6]);
                        (State::InitResetting, None, None)
                    }
                    State::InitResetting => {
                        self.check_ready(buffer);
                        (State::InitWaitingReady, None, None)
                    },
                    State::InitWaitingReady => {
                        let waiting = Self::parse_read_i2c(buffer, 1)[0];
                        if waiting & 0b1 == 0 {
                            // finished init
                            // todo: use bitfield crate
                            // forced mode, oversampling 1
                            let val = 0b00100001;
                            self.i2c_write(buffer, Register::CTRL_MEAS, [val]);
                            (State::InitConfiguring, None, None)  
                        } else {
                            self.check_ready(buffer);
                            (State::InitWaitingReady, None, None)
                        }
                    },
                    State::InitConfiguring => {
                        self.i2c_read::<{CALIBRATION_BYTES as u8}>(buffer, Register::DIG_T1);
                        (State::InitReadingCalibration, None, None)
                    },
                    State::InitReadingCalibration => { 
                        let data = Self::parse_read_i2c(buffer, CALIBRATION_BYTES as u8);
                        (State::Idle(CalibrationData::new(data)), None, Some(buffer))
                    },
                    State::Waiting(calibration) => {
                        let waiting_value = Self::parse_read_i2c(buffer, 1);
                        debug!("waiting: {:b}", waiting_value[0]);
                        // not waiting
                        if waiting_value[0] & 0b10000 == 0 {
                            self.i2c_read::<3>(buffer, Register::TEMP_MSB);
                            //self.i2c_read::<3>(buffer, Register::PRESS_MSB);
                            (State::Reading(calibration), None, None)
                        } else {
                            self.check_ready(buffer);
                            (State::Waiting(calibration), None, None)
                        }
                    }
                    State::Reading(calibration) => {
                        let readout = Self::parse_read_i2c(buffer, 3);
                        let msb = readout[0];
                        let lsb = readout[1];
                        let raw_temp = ((msb as u32) << 8) + (lsb as u32);
                        (State::Idle(calibration), Some(calibration.temp_from_raw(raw_temp as u32)), Some(buffer))
                    },
                    other => {
                        debug!("BMP280 received i2c reply in state {:?}", other);
                        (other, None, Some(buffer))
                    },
                };
                if let State::Idle(_) = new_state {
                    self.i2c.disable();
                }
                self.state.set(new_state);
                if let Some(buffer) = buffer {
                    self.buffer.replace(buffer);
                }

                if let Some(temp) = temp {
                    debug!("temp {}", temp);
                    self.temperature_client
                        .map(|cb| cb.callback(temp as usize));
                }
            }
            _ => {
                self.buffer.replace(buffer);
                self.i2c.disable();
                self.temperature_client.map(|cb| cb.callback(usize::MAX));
            }
        }
        debug!("new state: {:?}", self.state.get());
    }
}

impl<'a, A: Alarm<'a>> kernel::hil::sensors::TemperatureDriver<'a> for Bmp280<'a, A> {
    fn set_client(&self, client: &'a dyn kernel::hil::sensors::TemperatureClient) {
        self.temperature_client.set(client);
    }

    fn read_temperature(&self) -> Result<(), ErrorCode> {
        self.read_temperature()
    }
}
