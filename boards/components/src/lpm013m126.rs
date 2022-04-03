//! Component for the Japan Display LPM013M126 display.
//!
//! Usage
//! -----
//!
//! ```rust
//! // Optional
//! let spi_device = static_init!(
//!     VirtualSpiMasterDevice<'static, nrf52840::spi::SPIM>,
//!     VirtualSpiMasterDevice::new(
//!         mux_spi,
//!         &nrf52840_peripherals.gpio_port[Pin::P0_05], // CS pin
//!     ),
//! );
//! let display
//!     = components::lpm013m126::Lpm013m126Component {
//!         spi: spi_device,
//!         disp: disp_pin,
//!         extcomin: extcomin_pin,
//!         alarm_mux,
//!     }
//!     .finalize(
//!         components::lpm013m126_component_helper!(
//!             nrf52840::rtc::Rtc<'static>,
//!             nrf52840::gpio::GPIOPin,
//!             VirtualSpiMasterDevice<'static, nrf52840::spi::SPIM>,
//!         )
//!     );
//! display.initialize().unwrap();
//! // wait for `ScreenClient::screen_is_ready` callback
//! ```

use capsules::lpm013m126::Lpm013m126;
use capsules::virtual_alarm::{MuxAlarm, VirtualMuxAlarm};
use capsules::virtual_spi::{MuxSpiMaster, VirtualSpiMasterDevice};
use kernel::component::Component;
use kernel::hil::gpio;
use kernel::hil::spi::{SpiMaster, SpiMasterDevice};
use kernel::hil::time::Alarm;
use kernel::utilities::static_init::StaticUninitializedBuffer;

/// CS is active high
pub struct Inverted<'a, P: gpio::Pin>(pub &'a P);

impl<'a, P: gpio::Pin> gpio::Configure for Inverted<'a, P> {
    fn configuration(&self) -> gpio::Configuration {
        self.0.configuration()
    }
    fn make_output(&self) -> gpio::Configuration {
        self.0.make_output()
    }
    fn disable_output(&self) -> gpio::Configuration {
        self.0.disable_output()
    }
    fn make_input(&self) -> gpio::Configuration {
        self.0.make_input()
    }
    fn disable_input(&self) -> gpio::Configuration {
        self.0.disable_input()
    }
    fn deactivate_to_low_power(&self) {
        self.0.deactivate_to_low_power()
    }
    fn set_floating_state(&self, _: gpio::FloatingState) {
        unimplemented!() // not sure what it looks like with inversion
    }
    fn floating_state(&self) -> gpio::FloatingState {
        unimplemented!() // not sure what it looks like with inversion
    }
}

impl<'a, P: gpio::Pin> gpio::Output for Inverted<'a, P> {
    fn set(&self) {
        self.0.clear()
    }
    fn clear(&self) {
        self.0.set()
    }
    fn toggle(&self) -> bool {
        self.0.toggle()
    }
}

impl<'a, P: gpio::Pin> gpio::Input for Inverted<'a, P> {
    fn read(&self) -> bool {
        !self.0.read()
    }
}

/// Setup static space for the driver and its requirements.
#[macro_export]
macro_rules! lpm013m126_component_helper {
    ($A:ty, $P:ty, $S:ty, $mux_spi:expr, $chip_select:expr $(,)?) => {{
        use capsules::lpm013m126::{BUFFER_SIZE, Lpm013m126};
        use capsules::virtual_alarm::VirtualMuxAlarm;
        use capsules::virtual_spi::MuxSpiMaster;
        use kernel::static_buf;

        let alarm = static_buf!(VirtualMuxAlarm<'static, $A>);
        static mut BUFFER: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];

        let mux_spi: &MuxSpiMaster<'static, $S> = $mux_spi;
        let chip_select: &Inverted<'static, $P> = $chip_select;
        
        let spi_device = static_init!(
            VirtualSpiMasterDevice<'static, $S>,
            VirtualSpiMasterDevice::new(
                mux_spi,
                chip_select,
            ),
        );
        spi_device.setup();

        let lpm013m126 = static_buf!(Lpm013m126<
            'static,
            VirtualMuxAlarm<'static, $A>,
            $P,
            VirtualSpiMasterDevice<'static, $S>,
        >);

        (alarm, &mut BUFFER, spi_device, mux_spi, lpm013m126)
    }};
}

pub struct Lpm013m126Component<A, P, S>
    where
    A: 'static + Alarm<'static>,
    P: 'static + gpio::Pin,P: gpio::Pin,
    S: 'static + SpiMaster,
{
    //pub spi: &'static S,
    pub spi: core::marker::PhantomData<S>,
    pub disp: &'static P,
    pub extcomin: &'static P,
    pub alarm_mux: &'static MuxAlarm<'static, A>,
}

impl<A, P, S> Component for Lpm013m126Component<A, P, S>
    where
    A: 'static + Alarm<'static>,
    P: 'static + gpio::Pin,
    S: 'static + SpiMaster,
{
    type StaticInput = (
        StaticUninitializedBuffer<VirtualMuxAlarm<'static, A>>,
        &'static mut [u8],
        &'static VirtualSpiMasterDevice<'static, S>,
        &'static MuxSpiMaster<'static, S>,
        StaticUninitializedBuffer<
            Lpm013m126<'static, VirtualMuxAlarm<'static, A>, P, VirtualSpiMasterDevice<'static, S>>
        >,
    );
    type Output
        = &'static Lpm013m126<'static, VirtualMuxAlarm<'static, A>, P, VirtualSpiMasterDevice<'static, S>>;

    unsafe fn finalize(self, s: Self::StaticInput) -> Self::Output {
        let (alarm, buffer, spi_device, mux_spi, lpm013m126) = s;
        let lpm013m126_alarm = alarm.initialize(
            VirtualMuxAlarm::new(self.alarm_mux)
        );
        lpm013m126_alarm.setup();
        
        let lpm013m126 = lpm013m126.initialize(Lpm013m126::new(
            spi_device,
            self.extcomin,
            self.disp,
            lpm013m126_alarm,
            buffer,
        ));
        spi_device.set_client(lpm013m126);
        lpm013m126_alarm.set_alarm_client(lpm013m126);

        lpm013m126
    }
}
