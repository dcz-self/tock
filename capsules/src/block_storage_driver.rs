//! This provides userspace access to block storage.
//!
//! This is a basic implementation that gives total control of the storage
//! to the userspace.
//!
//! Example instantiation:
//!
//! ```rust
//! # use kernel::static_init;
//!
//! ```

use core::cell::Cell;
use core::cmp;

use kernel::grant::{AllowRoCount, AllowRwCount, Grant, UpcallCount};
use kernel::hil;
use kernel::hil::block_storage::{BlockIndex, Region};
use kernel::processbuffer::{ReadableProcessBuffer, WriteableProcessBuffer};
use kernel::syscall::{CommandReturn, SyscallDriver};
use kernel::utilities::cells::TakeCell;
use kernel::{ErrorCode, ProcessId};

use crate::driver;
pub const DRIVER_NUM: usize = driver::NUM::BlockStorage as usize;

enum Command {
    CHECK = 0,
    /// device size in bytes
    SIZE = 1,
    /// Size of write, erase blocks.
    /// Separate from size because doesn't fit in one return call.
    GEOMETRY = 2,
    /// Read an arbitrary range from an arbitrary address.
    READ_RANGE = 3,
    /// Read a single write block at given block index.
    READ = 4,
    /// Erase a single erase block at given block index.
    ERASE = 5,
    /// Write a single write block at given block index.
    WRITE = 6,
}

impl TryFrom<usize> for Command {
    type Error = ();
    fn try_from(v: usize) -> Result<Self, Self::Error> {
        match v {
            0 => Ok(Command::CHECK),
            1 => Ok(Command::SIZE),
            2 => Ok(Command::GEOMETRY),
            3 => Ok(Command::READ_RANGE),
            4 => Ok(Command::READ),
            5 => Ok(Command::ERASE),
            6 => Ok(Command::WRITE),
            _ => Err(()),
        }
    }
}

/// Ids for read-only allow buffers
mod ro_allow {
    pub const WRITE: usize = 0;
    /// The number of allow buffers the kernel stores for this grant
    pub const COUNT: usize = 1;
}

/// Ids for read-write allow buffers
mod rw_allow {
    pub const READ: usize = 0;
    /// The number of allow buffers the kernel stores for this grant
    pub const COUNT: usize = 1;
}

enum Upcall {
    READ = 0,
    ERASE = 1,
    WRITE = 2,
}

const UPCALL_COUNT: usize = 3;

//pub static mut BUFFER: [u8; 512] = [0; 512];

#[derive(Clone, Copy)]
enum Read {
    None,
    Requested(ProcessId),
}

#[derive(Clone, Copy)]
struct State {
    read: Read,
}

/// Userspace interface for `hil::block_storage::BlockStorage`.
///
/// Supports only one command of a given type at a time
/// (but may support only one command in flight at all,
/// if that's what the underlying device does).
///
/// Requires a buffer of size at least `W`.
///
/// `W` is the size of a write block, `E` is the erase block size.
pub struct BlockStorage<'a, T, const W: u32, const E: u32>
where
    T: hil::block_storage::BlockStorage<W, E>,
{
    // The underlying physical storage device.
    device: &'a T,
    // Per-app state.
    apps: Grant<
        (),
        UpcallCount<UPCALL_COUNT>,
        AllowRoCount<{ ro_allow::COUNT }>,
        AllowRwCount<{ rw_allow::COUNT }>,
    >,
    state: Cell<State>,
    buffer: TakeCell<'static, [u8]>,
}

impl<T, const W: u32, const E: u32> BlockStorage<'static, T, W, E>
where
    T: hil::block_storage::BlockStorage<W, E>,
{
    fn new(
        device: &'static T,
        grant: Grant<
            (),
            UpcallCount<UPCALL_COUNT>,
            AllowRoCount<{ ro_allow::COUNT }>,
            AllowRwCount<{ rw_allow::COUNT }>,
        >,
        buffer: &'static mut [u8],
    ) -> Self {
        Self {
            device,
            apps: grant,
            state: Cell::new(State { read: Read::None }),
            buffer: TakeCell::new(buffer),
        }
    }

    fn start_read(&self, region: &Region<W>, appid: ProcessId)
        -> Result<(), ErrorCode>
    {
        let state = self.state.get();
        match state.read {
            Read::Requested(..) => Err(ErrorCode::BUSY),
            Read::None => self.buffer.take().map_or_else(
                || Err(ErrorCode::NOMEM),
                |buffer| {
                    match self.device.read(region, buffer) {
                        Ok(()) => {
                            self.state.set(State {
                                read: Read::Requested(appid),
                                ..state
                            });
                            Ok(())
                        },
                        Err((e, buf)) => {
                            self.buffer.replace(buf);
                            Err(e)
                        }
                    }
                }
            ),
        }
    }
}

impl<T, const W: u32, const E: u32> hil::block_storage::Client<W, E> for BlockStorage<'_, T, W, E>
where
    T: hil::block_storage::BlockStorage<W, E>,
{
    fn read_complete(&self, read_buffer: &'static mut [u8], ret: Result<(), ErrorCode>) {
        let state = self.state.get();
        match state.read {
            Read::Requested(app_id) => {
                self
                    .apps
                    .enter(app_id, move |_, kernel_data| {
                        let ret = match ret {
                            Ok(()) => {
                                // Need to copy in the contents of the buffer
                                kernel_data
                                    .get_readwrite_processbuffer(rw_allow::READ)
                                    .and_then(|read| {
                                        read.mut_enter(|app_buffer| {
                                            let read_len = cmp::min(app_buffer.len(), W as usize);

                                            let d = &app_buffer[0..(read_len)];
                                            d.copy_from_slice(&read_buffer[0..read_len]);
                                        })
                                    })
                                    .map_err(ErrorCode::from)
                            },
                            Err(e) => Err(e),
                        };

                        // Replace the buffer we used to do this read.
                        self.buffer.replace(read_buffer);
                        self.state.set(State {
                            read: Read::None,
                            ..state
                        });
                        
                        // And then signal the app.
                        let upcall_data = ret.map_or_else(
                            |e| (1, e.into(), 0),
                            |()| (0, 0, 0),
                        );
                        kernel_data.schedule_upcall(Upcall::READ as usize, upcall_data)
                            .unwrap_or_else(|e| kernel::debug!("Can't upcall: {:?}", e))
                    })
                    .unwrap_or_else(|e| kernel::debug!("Can't get grant: {:?}", e))
            },
            _ => kernel::debug!("Unexpected read reply"),
        }
    }

    /// Block write complete.
    ///
    /// This will be called when the write operation is complete.
    fn write_complete(&self, write_buffer: &'static mut [u8], ret: Result<(), ErrorCode>) {}

    /// Block erase complete.
    ///
    /// This will be called when the erase operation is complete.
    fn erase_complete(&self, ret: Result<(), ErrorCode>) {}
}

impl<T, const W: u32, const E: u32> SyscallDriver for BlockStorage<'static, T, W, E>
where
    T: hil::block_storage::BlockStorage<W, E>,
{
    fn command(
        &self,
        command_num: usize,
        offset: usize,
        _length: usize,
        appid: ProcessId,
    ) -> CommandReturn {
        match Command::try_from(command_num) {
            Ok(Command::CHECK) => {
                CommandReturn::success()
            },
            Ok(Command::SIZE) => {
                self.device.get_size()
                    .map_or_else(CommandReturn::failure, CommandReturn::success_u64)
            },
            Ok(Command::GEOMETRY) => {
                CommandReturn::success_u32_u32(W, E)
            },
            Ok(Command::READ_RANGE) => {
                CommandReturn::failure(ErrorCode::NOSUPPORT)
            },
            Ok(Command::READ) => {
                let region = Region {
                    index: BlockIndex(offset as u32),
                    length_blocks: 1,
                };
                self.start_read(&region, appid).into()
            },
            Ok(Command::ERASE) => {
                CommandReturn::failure(ErrorCode::NOSUPPORT)
            },
            Ok(Command::WRITE) => {
                CommandReturn::failure(ErrorCode::NOSUPPORT)
            }
            Err(()) => CommandReturn::failure(ErrorCode::NOSUPPORT),
        }
    }

    fn allocate_grant(&self, processid: ProcessId) -> Result<(), kernel::process::Error> {
        self.apps.enter(processid, |_, _| {})
    }
}
