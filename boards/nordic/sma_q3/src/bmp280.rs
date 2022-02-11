//! hil driver for BMP280 Temperature and Pressure Sensor
//!
//! Not implemented: pressure

use core::cell::Cell;
use kernel::debug;
use kernel::hil::i2c;
use kernel::hil::time::Alarm;
use kernel::utilities::cells::{OptionalCell, TakeCell};
use kernel::ErrorCode;

pub static BASE_ADDR: u8 = 0x76;

enum Registers {
    Id = 0xd0,
    Reset = 0xe0,
    /// measuring: [3]
    /// im_update: [0]
    Status = 0xf3,
    /// osrs_t: [7:5]
    /// osrs_p: [4:2]
    /// mode: [1:0]
    CtrlMeas = 0xf4,
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
enum State {
    Uninitialized,
    Initializing,
    Idle,
    /// A request is in flight, but has not been returned to userspace yet.
    Reading,
}

pub struct BMP280<'a, A: Alarm<'a>> {
    i2c: &'a dyn i2c::I2CDevice,
    temperature_client: OptionalCell<&'a dyn kernel::hil::sensors::TemperatureClient>,
    state: Cell<State>,
    // Stores i2c commands
    buffer: TakeCell<'static, [u8]>,
    alarm: &'a A,
}

impl<'a, A: Alarm<'a>> BMP280<'a, A> {
    pub fn new(
        i2c: &'a dyn i2c::I2CDevice,
        buffer: &'static mut [u8],
        alarm: &'a A,
    ) -> Self {
        BMP280 {
            i2c: i2c,
            temperature_client: OptionalCell::empty(),
            state: Cell::new(State::Idle),
            buffer: TakeCell::new(buffer),
            alarm: alarm,
        }
    }

    fn read_temperature(&self) -> Result<(), ErrorCode> {
        use State::*;
        match self.state.get() {
            // Actually, the sensor might be on, just in default state.
            Uninitialized => Err(ErrorCode::OFF),
            Initializing => Err(ErrorCode::BUSY),
            Reading => Err(ErrorCode::BUSY),
            Idle => self.read_temp_data(),
        }
    }

    fn read_temp_data(&self) -> Result<(), ErrorCode> {
        self.buffer.take().map_or_else(
            || panic!("BMP280 No buffer available!"),
            |buffer| {
                self.state.set(State::Reading);
                self.i2c.enable();

                buffer[0] = Registers::TEMP_MSB as u8;
                // why is explicit length needed here?
                // TODO: propagate errors
                self.i2c.read(buffer, 3).unwrap();

                Ok(())
            },
        )
    }
    
    pub fn begin_initialize(&self) -> Result<(), ErrorCode> {
        self.buffer.take().map_or_else(
            || panic!("BMP280 No buffer available!"),
            |buffer| {
                match self.state.get() {
                    State::Uninitialized => {
                        self.state.set(State::Initializing);
                        self.i2c.enable();

                        buffer[0] = Registers::CtrlMeas as u8;
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

impl<'a, A: Alarm<'a>> i2c::I2CClient for BMP280<'a, A> {
    fn command_complete(&self, buffer: &'static mut [u8], status: Result<(), i2c::Error>) {
        match status {
            Ok(()) => {
                let (new_state, temp) = match self.state.get() {
                    State::Reading => {
                        let msb = buffer[0];
                        let lsb = buffer[1];
                        let raw_temp = ((msb as usize) << 8) + (lsb as usize);
                        // TODO: calculate actual temperature
                        (State::Idle, Some(raw_temp))
                    },
                    State::Initializing => (State::Idle, None),
                    other => {
                        debug!("MP280 received i2c reply in state {:?}", other);
                        (other, None)
                    },
                };
                if let State::Idle = new_state {
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

impl<'a, A: Alarm<'a>> kernel::hil::sensors::TemperatureDriver<'a> for BMP280<'a, A> {
    fn set_client(&self, client: &'a dyn kernel::hil::sensors::TemperatureClient) {
        self.temperature_client.set(client);
    }

    fn read_temperature(&self) -> Result<(), ErrorCode> {
        self.read_temperature()
    }
}
