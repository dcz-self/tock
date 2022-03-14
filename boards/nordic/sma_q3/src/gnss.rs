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
        //self.uart.receive_buffer(self.read_buffer.take().unwrap(), BUFFER_SIZE).unwrap();
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
        kernel::debug!("{}", core::str::from_utf8(&buffer[..rx_len]).unwrap_or("-----INVALID"));

        self.read_buffer.replace(buffer);
        self.start_receive();
    }
}

impl<'a, T, F: hil::block_storage::BlockStorage<4096, 4096>>
    hil::block_storage::Client<4096, 4096>
    for Gnss<'a, T, F>
{
    fn read_complete(&self, buffer: &'static mut [u8], error: Result<(), ErrorCode>) {
        dbg!("Read");
        dbg!(&buffer.as_mut()[0..16]);
    }
    fn write_complete(&self, buffer: &'static mut [u8], error: Result<(), ErrorCode>) {
        let region = hil::block_storage::Region::<4096>{
            index: hil::block_storage::BlockIndex(43),
            length_blocks: 1,
        };
        debug!("written");
        buffer[0] = 0;
        dbg!(self.flash.read(&region, buffer));
    }
    fn erase_complete(&self, error: Result<(), ErrorCode>) {
        debug!("erased");
        let mut buf = self.read_buffer.take().unwrap();
        buf[0] = 42;
        let region = hil::block_storage::Region::<4096>{
            index: hil::block_storage::BlockIndex(43),
            length_blocks: 1,
        };
        dbg!(self.flash.write(&region, buf));
    }
}