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

/// Currently includes temperature only
const CALIBRATION_BYTES: u8 = 6;

#[allow(non_camel_case_types)]
#[allow(dead_code)]
enum Registers {
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
    
    fn temp_from_raw(&self, raw_temp: usize) -> usize {
    // FIXME
        raw_temp
    }
}

/// Internal state.
/// Each state can lead to the next on in order of appearance.
#[derive(Clone, Copy, PartialEq, Debug)]
enum State {
    Uninitialized,
    InitConfiguring,
    InitReadingCalibration,
    Idle(CalibrationData),
    /// A request is in flight, but has not been returned to userspace yet.
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

    fn read_temperature(&self) -> Result<(), ErrorCode> {
        use State::*;
        match self.state.get() {
            // Actually, the sensor might be on, just in default state.
            Uninitialized => Err(ErrorCode::OFF),
            InitConfiguring | InitReadingCalibration => Err(ErrorCode::BUSY),
            Idle(calibration) => self.buffer.take().map_or_else(
                || panic!("BMP280 No buffer available!"),
                |buffer| {
                    self.state.set(State::Reading(calibration));
                    self.i2c.enable();

                    buffer[0] = Registers::TEMP_MSB as u8;
                    // why is explicit length needed here?
                    // TODO: propagate errors
                    self.i2c.read(buffer, 3).unwrap();

                    Ok(())
                }
            ),
            Reading(_calibration) => Err(ErrorCode::BUSY),
        }
    }
    
    pub fn begin_initialize(&self) -> Result<(), ErrorCode> {
        self.buffer.take().map_or_else(
            || panic!("BMP280 No buffer available!"),
            |buffer| {
                match self.state.get() {
                    State::Uninitialized => {
                        self.state.set(State::InitConfiguring);
                        self.i2c.enable();

                        buffer[0] = Registers::CTRL_MEAS as u8;
                        // todo: use bitfield crate
                        buffer[1] = 0b00100001;

                        // why is explicit length needed here?
                        // TODO: propagate errors
                        self.i2c.write(buffer, 2).unwrap();
                        Ok(())
                    },
                    _ => Err(ErrorCode::ALREADY),
                }
            }
        )
    }
}

impl<'a, A: Alarm<'a>> i2c::I2CClient for Bmp280<'a, A> {
    fn command_complete(&self, buffer: &'static mut [u8], status: Result<(), i2c::Error>) {
        match status {
            Ok(()) => {
                let (new_state, temp) = match self.state.get() {
                    State::InitConfiguring => {
                        self.buffer.take().map_or_else(
                            || panic!("BMP280 No buffer available!"),
                            |buffer| {
                                buffer[0] = Registers::DIG_T1 as u8;
                                self.i2c.write(buffer, CALIBRATION_BYTES).unwrap()
                            }
                        );
                        (State::InitReadingCalibration, None)
                    },
                    State::InitReadingCalibration => (State::Idle(CalibrationData::new(buffer)), None),
                    State::Reading(calibration) => {
                        let msb = buffer[0];
                        let lsb = buffer[1];
                        let raw_temp = ((msb as usize) << 8) + (lsb as usize);
                        (State::Idle(calibration), Some(calibration.temp_from_raw(raw_temp)))
                    },
                    other => {
                        debug!("BMP280 received i2c reply in state {:?}", other);
                        (other, None)
                    },
                };
                if let State::Idle(_) = new_state {
                    self.i2c.disable();
                }
                self.state.set(new_state);
                if let Some(temp) = temp {
                    self.temperature_client
                        .map(|cb| cb.callback(temp));
                }
            }
            _ => {
                // why does the buffer get replaced?
                // where does this buffer come from?
                self.buffer.replace(buffer);
                self.i2c.disable();
                self.temperature_client.map(|cb| cb.callback(usize::MAX));
            }
        }
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
