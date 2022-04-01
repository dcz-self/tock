//! Component for the Japan Display LPM013M126 display.
//!
//! Usage
//! -----
//!
//! ```rust
//! let display
//!     = components::lpm013m126::Lpm013m126Component {
//!         spi,
//!         disp: disp_pin,
//!         extcomin: extcomin_pin,
//!         alarm_mux,
//!     }
//!     .finalize(
//!         components::lpm013m126_component_helper!(nrf52840::rtc::Rtc<'static>),
//!     );
//! display.initialize();
//! // wait for `ScreenClient::screen_is_ready` callback
//! ```

use capsules::lpm013m126::Lpm013m126;
use capsules::virtual_alarm::{MuxAlarm, VirtualMuxAlarm};
use core::mem::MaybeUninit;
use kernel::component::Component;
use kernel::hil::gpio;
use kernel::hil::spi::{SpiMasterDevice};
use kernel::hil::time::Alarm;
use kernel::utilities::static_init::StaticUninitializedBuffer;

/// Setup static space for the driver and its requirements.
#[macro_export]
macro_rules! lpm013m126_component_helper {
    ($A:ty, $P:ty, $S:ty $(,)?) => {{
        use capsules::lpm013m126::{BUFFER_SIZE, Lpm013m126};
        use kernel::static_buf;

        static mut BUFFER: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];

        let lpm013m126 = static_buf!(Lpm013m126<
            'static,
            VirtualMuxAlarm<'static, $A>,
            $P,
            $S,
        >);
        let alarm = static_buf!(VirtualMuxAlarm<'static, $A>);
        (alarm, &mut BUFFER, lpm013m126)
    }};
}

pub struct Lpm013m126Component<A, P, S>
    where
    A: 'static + Alarm<'static>,
    P: 'static + gpio::Pin,P: gpio::Pin,
    S: 'static + SpiMasterDevice,
{
    pub spi: &'static S,
    pub disp: &'static P,
    pub extcomin: &'static P,
    pub alarm_mux: &'static MuxAlarm<'static, A>,
}

impl<A, P, S> Component for Lpm013m126Component<A, P, S>
    where
    A: 'static + Alarm<'static>,
    P: 'static + gpio::Pin,
    S: 'static + SpiMasterDevice,
{
    type StaticInput = (
        StaticUninitializedBuffer<VirtualMuxAlarm<'static, A>>,
        &'static mut [u8],
        StaticUninitializedBuffer<
            Lpm013m126<'static, VirtualMuxAlarm<'static, A>, P, S>
        >,
    );
    type Output
        = &'static Lpm013m126<'static, VirtualMuxAlarm<'static, A>, P, S>;

    unsafe fn finalize(self, s: Self::StaticInput) -> Self::Output {
        let lpm013m126_alarm = s.0.initialize(
            VirtualMuxAlarm::new(self.alarm_mux)
        );
        lpm013m126_alarm.setup();

        let lpm013m126 = s.2.initialize(Lpm013m126::new(
            self.spi,
            self.extcomin,
            self.disp,
            lpm013m126_alarm,
            s.1,
        ));
        self.spi.set_client(lpm013m126);
        lpm013m126_alarm.set_alarm_client(lpm013m126);

        lpm013m126
    }
}
