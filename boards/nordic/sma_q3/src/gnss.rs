/*! Prints out GNSS output. Nothing fancy yet.
 */
use kernel::hil::uart;
use kernel::ErrorCode;
use kernel::utilities::cells::TakeCell;

use crate::dbg;

pub const BUFFER_SIZE: usize = 64;

pub struct Gnss<'a, T>{
    uart: &'a T,
    buffer: TakeCell<'static, [u8]>,
}

impl<'a, 'b: 'a, T: uart::Receive<'a>> Gnss<'b, T> where Self: 'a {
    pub fn new(uart: &'b T, buffer: &'static mut [u8]) -> Self {
        Self {
            uart,
            buffer: TakeCell::new(buffer),
        }
    }
    pub fn start_receive(&self) {
        dbg!(self.uart.receive_buffer(self.buffer.take().unwrap(), 1)).unwrap();
    }
}

impl<'a, T: uart::Receive<'a>> uart::ReceiveClient for Gnss<'_, T> where Self: 'a {
    fn received_buffer(
        &self,
        buffer: &'static mut [u8],
        rx_len: usize,
        rcode: Result<(), ErrorCode>,
        error: uart::Error,
    ) {
        dbg!(rcode);
        dbg!(error);
        kernel::debug!("uart rx: {}", core::str::from_utf8(dbg!(&buffer[..dbg!(rx_len)])).unwrap_or("-----INVALID"));

        self.buffer.replace(buffer);
        self.start_receive();
    }
}
