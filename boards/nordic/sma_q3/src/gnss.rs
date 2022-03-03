/*! Prints out GNSS output. Nothing fancy yet.
 */
use kernel::debug;
use kernel::hil;
use kernel::hil::uart;
use kernel::ErrorCode;
use kernel::utilities::cells::TakeCell;

use crate::dbg;

pub const BUFFER_SIZE: usize = 64;

pub struct Gnss<'a, T, F>{
    uart: &'a T,
    read_buffer: TakeCell<'static, [u8]>,
    flash: &'a F,
}

impl<'a, 'b: 'a, T: uart::Receive<'a>, F> Gnss<'b, T, F> where Self: 'a {
    pub fn new(uart: &'b T, flash: &'b F, buffer: &'static mut [u8]) -> Self {
        Self {
            uart,
            flash,
            read_buffer: TakeCell::new(buffer),
        }
    }
    pub fn start_receive(&self) {
        self.uart.receive_buffer(self.read_buffer.take().unwrap(), BUFFER_SIZE).unwrap();
    }
}

impl<'a, T: uart::Receive<'a>, F> uart::ReceiveClient for Gnss<'_, T, F> where Self: 'a {
    fn received_buffer(
        &self,
        buffer: &'static mut [u8],
        rx_len: usize,
        rcode: Result<(), ErrorCode>,
        error: uart::Error,
    ) {
        //kernel::debug!("{}", core::str::from_utf8(&buffer[..rx_len]).unwrap_or("-----INVALID"));

        self.read_buffer.replace(buffer);
        self.start_receive();
    }
}

impl<'a, T, F: hil::flash::Flash> hil::flash::Client<F> for Gnss<'a, T, F> {
    fn read_complete(&self, buffer: &'static mut F::Page, error: hil::flash::Error) {
        dbg!(&buffer.as_mut()[0..16]);
    }
    fn write_complete(&self, buffer: &'static mut F::Page, error: hil::flash::Error) {
        debug!("written");
    }
    fn erase_complete(&self, error: hil::flash::Error) {
        debug!("erased");
    }
}