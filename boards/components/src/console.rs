//! Components for Console, the generic serial interface, and for multiplexed access
//! to UART.
//!
//!
//! This provides two Components, `ConsoleComponent`, which implements a buffered
//! read/write console over a serial port, and `UartMuxComponent`, which provides
//! multiplexed access to hardware UART. As an example, the serial port used for
//! console on Imix is typically USART3 (the DEBUG USB connector).
//!
//! Usage
//! -----
//! ```rust
//! let uart_mux = UartMuxComponent::new(&sam4l::usart::USART3,
//!                                      115200,
//!                                      deferred_caller).finalize(());
//! let console = ConsoleComponent::new(board_kernel, uart_mux)
//!    .finalize(console_component_helper!());
//! ```
// Author: Philip Levis <pal@cs.stanford.edu>
// Last modified: 1/08/2020

use capsules::console;
use capsules::virtual_uart::{MuxUart, UartDevice};
use core::mem::MaybeUninit;
use kernel::capabilities;
use kernel::component::Component;
use kernel::create_capability;
use kernel::dynamic_deferred_call::DynamicDeferredCall;
use kernel::hil;
use kernel::hil::uart;
use kernel::{static_init, static_init_half};

pub const BUF_SIZE: usize = 64;

pub struct UartMuxComponent {
    uart: &'static dyn uart::Uart<'static>,
    baud_rate: u32,
    deferred_caller: &'static DynamicDeferredCall,
}

impl UartMuxComponent {
    pub fn new(
        uart: &'static dyn uart::Uart<'static>,
        baud_rate: u32,
        deferred_caller: &'static DynamicDeferredCall,
    ) -> UartMuxComponent {
        UartMuxComponent {
            uart,
            baud_rate,
            deferred_caller,
        }
    }
}

impl Component for UartMuxComponent {
    type StaticInput = ();
    type Output = &'static MuxUart<'static>;

    unsafe fn finalize(self, _s: Self::StaticInput) -> Self::Output {
        let uart_mux = static_init!(
            MuxUart<'static>,
            MuxUart::new(
                self.uart,
                &mut capsules::virtual_uart::RX_BUF,
                self.baud_rate,
                self.deferred_caller,
            )
        );
        uart_mux.initialize_callback_handle(
            self.deferred_caller.register(uart_mux).unwrap(), // Unwrap fail = no deferred call slot available for uart mux
        );

        uart_mux.initialize();
        hil::uart::Transmit::set_transmit_client(self.uart, uart_mux);
        hil::uart::Receive::set_receive_client(self.uart, uart_mux);

        uart_mux
    }
}

#[macro_export]
macro_rules! console_component_helper {
    () => {{
        use components::console::BUF_SIZE;
        use core::mem::MaybeUninit;
        static mut WRITE_BUF: MaybeUninit<[u8; BUF_SIZE]> = MaybeUninit::uninit();
        static mut READ_BUF: MaybeUninit<[u8; BUF_SIZE]> = MaybeUninit::uninit();
        (&mut WRITE_BUF, &mut READ_BUF)
    };};
}

pub struct ConsoleComponent {
    board_kernel: &'static kernel::Kernel,
    driver_num: usize,
    uart_device: &'static UartDevice,
    console: &'static console::Console<'static>
}

impl ConsoleComponent {
    pub fn new(
        board_kernel: &'static kernel::Kernel,
        driver_num: usize,
        uart_mux: &'static MuxUart,
    ) -> ConsoleComponent {
        ConsoleComponent {
            board_kernel: board_kernel,
            driver_num: driver_num,
            uart_mux: uart_mux,
        }
    }
}

impl Component for ConsoleComponent {
    type StaticInput = (
        &'static mut MaybeUninit<[u8; BUF_SIZE]>,
        &'static mut MaybeUninit<[u8; BUF_SIZE]>,
    );
    type Output = &'static console::Console<'static>;

    unsafe fn finalize(self, s: Self::StaticInput) -> Self::Output {
        let grant_cap = create_capability!(capabilities::MemoryAllocationCapability);

        let write_buffer = static_init_half!(s.0, [u8; BUF_SIZE], [0; BUF_SIZE],);

        let read_buffer = static_init_half!(s.1, [u8; BUF_SIZE], [0; BUF_SIZE],);

        // Create virtual device for console.
        let console_uart = static_init!(UartDevice, UartDevice::new(self.uart_mux, true));
        console_uart.setup();

        let console = static_init!(
            console::Console<'static>,
            console::Console::new(
                console_uart,
                write_buffer,
                read_buffer,
                self.board_kernel.create_grant(self.driver_num, &grant_cap)
            )
        );
        hil::uart::Transmit::set_transmit_client(console_uart, console);
        hil::uart::Receive::set_receive_client(console_uart, console);

        console
    }
}
