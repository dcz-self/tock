/*! Prints out GNSS output. Nothing fancy yet.
 */
use kernel::hil::uart;
use kernel::ErrorCode;
use kernel::utilities::cells::TakeCell;

use crate::dbg;

const BUFFER_SIZE: usize = 64;

struct Gnss<'a, T>{
    uart: &'a T,
    buffer: TakeCell<'static, [u8]>,
}

impl<'a, T: uart::Receive<'a>> Gnss<'_, T> where Self: 'a {
    fn start_receive(&self) {
        dbg!(self.uart.receive_buffer(self.buffer.take().unwrap(), BUFFER_SIZE)).unwrap();
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
        kernel::debug!("uart rx: {}", core::str::from_utf8(&buffer[..rx_len]).unwrap_or("-----INVALID"));

        self.buffer.replace(buffer);
        self.start_receive();
    }
    
}
