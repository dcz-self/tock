use crate::dbg;
use kernel::ErrorCode;
use kernel::debug;
use kernel::hil;
use kernel::utilities::cells::TakeCell;

pub struct Block<'a, F> {
    pub flash: &'a F,
    pub buffer: TakeCell<'static, [u8; 256]>,
}

impl<'a, F> Block<'a, F> {
    pub fn new(flash: &'a F, buffer: &'static mut [u8; 256]) -> Self {
        Block { flash, buffer: TakeCell::new(buffer) }
    }
}

impl<'a, F: hil::block_storage::Storage<256, 4096>>
    hil::block_storage::ReadableClient
    for Block<'a, F>
{
    fn read_complete(&self, buffer: &'static mut [u8], error: Result<(), ErrorCode>) {
        dbg!("Read");
        dbg!(&buffer.as_mut()[0..5]);
    }
}

impl<'a, F: hil::block_storage::Storage<256, 4096>>
    hil::block_storage::WriteableClient
    for Block<'a, F>
{
    fn write_complete(&self, buffer: &'static mut [u8], error: Result<(), ErrorCode>) {
        let region = hil::block_storage::BlockIndex(43);
        debug!("written");
        buffer[0] = 0;
        match self.flash.read(&region, buffer) {
            Err((e, _)) => {dbg!(e);},
            _ => {dbg!("read submitted");},
        }
    }
    fn discard_complete(&self, error: Result<(), ErrorCode>) {
        debug!("erased");
        let buf = self.buffer.take().unwrap();
        buf[0] = 42;
        let region = hil::block_storage::BlockIndex(43);
        dbg!(self.flash.write(&region, buf));
    }
}