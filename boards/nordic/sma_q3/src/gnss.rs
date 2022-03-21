/*! Prints out GNSS output. Nothing fancy yet.
 */
use kernel::debug;
use kernel::hil;
use kernel::hil::uart;
use kernel::ErrorCode;
use kernel::utilities::cells::TakeCell;

use crate::dbg;

pub const BUFFER_SIZE: usize = 64;

pub struct Gnss<'a, T>{
    uart: &'a T,
    read_buffer: TakeCell<'static, [u8]>,
}

impl<'a, 'b: 'a, T: uart::Receive<'a>> Gnss<'b, T> where Self: 'a {
    pub fn new(uart: &'b T, buffer: &'static mut [u8]) -> Self {
        Self {
            uart,
            read_buffer: TakeCell::new(buffer),
        }
    }
    pub fn start_receive(&self) {
        self.uart.receive_buffer(self.read_buffer.take().unwrap(), BUFFER_SIZE).unwrap();
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
        kernel::debug!("{}", core::str::from_utf8(&buffer[..rx_len]).unwrap_or("-----INVALID"));

        self.read_buffer.replace(buffer);
        self.start_receive();
    }
}

use kernel::{ create_capability, static_init };
use kernel::capabilities;
use capsules::console;

static mut WRITE_BUF: [u8; 64] = [0; 64];
static mut READ_BUF: [u8; 64] = [0; 64];

pub unsafe fn finalize<T: hil::uart::UartData<'static>>(
    board_kernel: &'static kernel::Kernel,
    uart: &'static T,
    driver_num: usize
) -> &'static console::Console<'static> {
    let grant_cap = create_capability!(capabilities::MemoryAllocationCapability);

    let driver = static_init!(
        console::Console<'static>,
        console::Console::new(
            uart,
            &mut WRITE_BUF,
            &mut READ_BUF,
            board_kernel.create_grant(driver_num, &grant_cap)
        )
    );
    hil::uart::Transmit::set_transmit_client(uart, driver);
    hil::uart::Receive::set_receive_client(uart, driver);

    driver
}