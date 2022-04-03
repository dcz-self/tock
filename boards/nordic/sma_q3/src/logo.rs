/*! Shows Jazda logo in 1-bit */
use kernel::ErrorCode;

struct D<S: 'static + kernel::hil::time::Alarm<'static>, P: 'static + kernel::hil::gpio::Pin, B: 'static + kernel::hil::spi::SpiMasterDevice>(
    &'static capsules::lpm013m126::Lpm013m126<'static, S, P, B>
);

impl<S: 'static + kernel::hil::time::Alarm<'static>, P: 'static + kernel::hil::gpio::Pin, B: 'static + kernel::hil::spi::SpiMasterDevice> kernel::hil::screen::ScreenClient for D<S, P, B> {
    fn screen_is_ready(&self) {
        debug!("Display ready");
        let mut b = unsafe { static_init!(
            [u8; 3344],
            [0; 3344],
        ) };
        b.copy_from_slice(include_bytes!("./logo.bin"));
        dbg!(self.0.write(&mut b[..], 3344));
    }
    fn command_complete(&self, res: Result<(), ErrorCode>) {
        debug!("Command complete");
    }
    fn write_complete(&self, _: &'static mut [u8], _: Result<(), ErrorCode>) {
        debug!("Write complete");
    }
}
use kernel::hil::screen::Screen;
use capsules::virtual_spi::VirtualSpiMasterDevice;

unsafe fn<DS: Screen> init_logo_once(display: DS) {
    debug!("Display started");
    let cd = static_init!(
        D<
            VirtualMuxAlarm<'static, nrf52840::rtc::Rtc<'static>>,
        nrf52840::gpio::GPIOPin,
        VirtualSpiMasterDevice<'static, nrf52840::spi::SPIM>,
        >,
        D(display)
    );
    
    display.set_client(Some(cd));
}