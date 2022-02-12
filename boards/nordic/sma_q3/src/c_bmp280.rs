//! Component for the BMP280 Temperature and Pressure Sensor.
//!
//! Based off the SHT3x code.
//!
//! I2C Interface
//!
//! Usage
//! -----
//!
//! With the default i2c address
//! ```rust
//! let bmp280 = components::bmp280::Bmp280Component::new(sensors_i2c_bus, mux_alarm).finalize(
//!         components::bmp280_component_helper!(nrf52::rtc::Rtc<'static>),
//!     );
//! bmp280.reset();
//! ```
//!
//! With a specified i2c address
//! ```rust
//! let bmp280 = components::bmp280::Bmp280Component::new(sensors_i2c_bus, mux_alarm).finalize(
//!         components::bmp280_component_helper!(nrf52::rtc::Rtc<'static>, capsules::bmp280::BASE_ADDR),
//!     );
//! bmp280.reset();
//! ```

use crate::bmp280::Bmp280;
use capsules::virtual_alarm::{MuxAlarm, VirtualMuxAlarm};
use capsules::virtual_i2c::MuxI2C;
use components::i2c::I2CComponent;
use components::i2c_component_helper;
use core::mem::MaybeUninit;
use kernel::component::Component;
use kernel::hil::time::Alarm;

use kernel::static_init_half;

/// Setup static space for the driver and its requirements.
#[macro_export]
macro_rules! bmp280_component_helper {
    ($A:ty) => {{
        use crate::bmp280;
        $crate::bmp280_component_helper!($A, bmp280::BASE_ADDR)
    }};

    // used for specifically stating the i2c address
    // as some boards (like nrf52) require a shift
    ($A:ty, $address: expr) => {{
        use crate::bmp280::Bmp280;
        use capsules::virtual_i2c::I2CDevice;
        use core::mem::MaybeUninit;

        static mut BUFFER: [u8; 6] = [0; 6];

        static mut bmp280: MaybeUninit<Bmp280<'static, VirtualMuxAlarm<'static, $A>>> =
            MaybeUninit::uninit();
        static mut bmp280_alarm: MaybeUninit<VirtualMuxAlarm<'static, $A>> = MaybeUninit::uninit();
        (&mut bmp280_alarm, &mut BUFFER, &mut bmp280, $address)
    }};
}

pub struct Bmp280Component<A: 'static + Alarm<'static>> {
    i2c_mux: &'static MuxI2C<'static>,
    alarm_mux: &'static MuxAlarm<'static, A>,
}

impl<A: 'static + Alarm<'static>> Bmp280Component<A> {
    pub fn new(
        i2c_mux: &'static MuxI2C<'static>,
        alarm_mux: &'static MuxAlarm<'static, A>,
    ) -> Bmp280Component<A> {
        Bmp280Component { i2c_mux, alarm_mux }
    }
}

impl<A: 'static + Alarm<'static>> Component for Bmp280Component<A> {
    type StaticInput = (
        &'static mut MaybeUninit<VirtualMuxAlarm<'static, A>>,
        &'static mut [u8],
        &'static mut MaybeUninit<Bmp280<'static, VirtualMuxAlarm<'static, A>>>,
        u8,
    );
    type Output = &'static Bmp280<'static, VirtualMuxAlarm<'static, A>>;

    unsafe fn finalize(self, static_buffer: Self::StaticInput) -> Self::Output {
        let bmp280_i2c = I2CComponent::new(self.i2c_mux, static_buffer.3)
            .finalize(i2c_component_helper!());

        let bmp280_alarm = static_init_half!(
            static_buffer.0,
            VirtualMuxAlarm<'static, A>,
            VirtualMuxAlarm::new(self.alarm_mux)
        );
        bmp280_alarm.setup();

        let bmp280 = static_init_half!(
            static_buffer.2,
            Bmp280<'static, VirtualMuxAlarm<'static, A>>,
            Bmp280::new(bmp280_i2c, static_buffer.1, bmp280_alarm)
        );
        bmp280_i2c.set_client(bmp280);
        // TODO: need to implement waiting for ready
        //bmp280_alarm.set_alarm_client(bmp280);

        bmp280
    }
}
