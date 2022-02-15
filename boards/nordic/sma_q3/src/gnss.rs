use kernel::hil::uart;
use kernel::ErrorCode;


const RX_BUF_LEN: usize = 64;
pub static mut RX_BUF: [u8; RX_BUF_LEN] = [0; RX_BUF_LEN];

struct Gnss<T>{
    uart: &T,
};

impl<T> uart::ReceiveClient for Gnss<T> {
    fn received_buffer(
        &self,
        buffer: &'static mut [u8],
        rx_len: usize,
        rcode: Result<(), ErrorCode>,
        error: uart::Error,
    ) {
        // Likely we will issue another receive in response to the previous one
        // finishing. `next_read_len` keeps track of the shortest outstanding
        // receive requested by any client. We start with the longest it can be,
        // i.e. the length of the buffer we pass to the UART.
        let mut next_read_len = buffer.len();
        let mut read_pending = false;

        kernel::debug!("uart rx: {}", str.from_utf8(buffer[..rx_len]).unwrap_or("-----INVALID"));

        // After we have finished all callbacks we can replace this buffer. We
        // have to wait to replace this to make sure that a client calling
        // `receive_buffer()` in its callback does not start an underlying UART
        // receive before all callbacks have finished.
        self.buffer.replace(buffer);

        // If either our outstanding receive was longer than the number of bytes
        // we just received, or if a new receive has been started, we start the
        // underlying UART receive again.
        if read_pending {
            self.start_receive(next_read_len);
        }
    }
}
