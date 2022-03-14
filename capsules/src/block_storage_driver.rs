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
use kernel::processbuffer::{ReadableProcessBuffer, WriteableProcessBuffer};
use kernel::syscall::{CommandReturn, SyscallDriver};
use kernel::utilities::cells::{OptionalCell, TakeCell};
use kernel::{ErrorCode, ProcessId};

/// Syscall driver number.
use crate::driver;
pub const DRIVER_NUM: usize = driver::NUM::NvmStorage as usize;

enum Command {
    READ_RANGE = 0,
    READ = 1,
    ERASE = 2,
    WRITE = 3,
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

pub static mut BUFFER: [u8; 512] = [0; 512];

#[derive(Clone, Copy, PartialEq)]
pub enum NonvolatileCommand {
    UserspaceRead,
    UserspaceWrite,
    KernelRead,
    KernelWrite,
}

#[derive(Clone, Copy)]
pub enum NonvolatileUser {
    App { app_id: ProcessId },
    Kernel,
}

pub struct App {
    pending_command: bool,
    command: NonvolatileCommand,
    offset: usize,
    length: usize,
}

impl Default for App {
    fn default() -> App {
        App {
            pending_command: false,
            command: NonvolatileCommand::UserspaceRead,
            offset: 0,
            length: 0,
        }
    }
}

/// Userspace interface for `hil::block_storage::BlockStorage`
pub struct BlockStorage<T, const W: u32, const E: u32>
    where T: hil::block_storage::BlockStorage<W, E>
{
    // The underlying physical storage device.
    device: &'a dyn hil::nonvolatile_storage::NonvolatileStorage<'static>,
    // Per-app state.
    apps: Grant<
        App,
        UpcallCount<3>,
        AllowRoCount<{ ro_allow::COUNT }>,
        AllowRwCount<{ rw_allow::COUNT }>,
    >,
}

impl<T: hil::block_storage::BlockStorage> hil::block_storage::Client
    for BlockStorage<T>
{
    fn read_complete(&self, read_buffer: &'static mut [u8], ret: Result<(), ErrorCode>) {
        let state = self.state.get();
        match state.read {
            Read::Requested(length) => self.current_user.take().map(|app_id| {
                self.apps.enter(app_id, move |_, kernel_data| {
                    // Need to copy in the contents of the buffer
                    kernel_data
                        .get_readwrite_processbuffer(rw_allow::READ)
                        .and_then(|read| {
                            read.mut_enter(|app_buffer| {
                                let read_len = cmp::min(app_buffer.len(), length);

                                let d = &mut app_buffer[0..(read_len as usize)];
                                d.copy_from_slice(&buffer[0..read_len]);
                            })
                        });

                    // Replace the buffer we used to do this read.
                    self.buffer.replace(buffer);

                    // And then signal the app.
                    kernel_data.schedule_upcall(0, (length, 0, 0)).ok();
                });
            }),
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

impl<T: hil::block_storage::BlockStorage> SyscallDriver for BlockStorage<T> {
    /// Setup shared kernel-writable buffers.
    ///
    /// ### `allow_num`
    ///
    /// - `0`: Setup a buffer to read from the nonvolatile storage into.

    /// Setup shared kernel-readable buffers.
    ///
    /// ### `allow_num`
    ///
    /// - `0`: Setup a buffer to write bytes to the nonvolatile storage.

    // Setup callbacks.
    //
    // ### `subscribe_num`
    //
    // - `0`: Setup a read done callback.
    // - `1`: Setup a write done callback.

    /// Command interface.
    ///
    /// Commands are selected by the lowest 8 bits of the first argument.
    ///
    /// ### `command_num`
    ///
    /// - `0`: Return Ok(()) if this driver is included on the platform.
    /// - `1`: Return the number of bytes available to userspace.
    /// - `2`: Start a read from the nonvolatile storage.
    /// - `3`: Start a write to the nonvolatile_storage.
    fn command(
        &self,
        command_num: usize,
        offset: usize,
        length: usize,
        appid: ProcessId,
    ) -> CommandReturn {
        match command_num {
            0 /* This driver exists. */ => {
                CommandReturn::success()
            }

            1 /* How many bytes are accessible from userspace */ => {
                // TODO: Would break on 64-bit platforms
                CommandReturn::success_u32(self.userspace_length as u32)
            },

            2 /* Issue a read command */ => {
                let res =
                    self.enqueue_command(
                        NonvolatileCommand::UserspaceRead,
                        offset,
                        length,
                        Some(appid),
                    );

                match res {
                    Ok(()) => CommandReturn::success(),
                    Err(e) => CommandReturn::failure(e),
                }
            }

            3 /* Issue a write command */ => {
                let res =
                    self.enqueue_command(
                        NonvolatileCommand::UserspaceWrite,
                        offset,
                        length,
                        Some(appid),
                    );

                match res {
                    Ok(()) => CommandReturn::success(),
                    Err(e) => CommandReturn::failure(e),
                }
            }

            _ => CommandReturn::failure(ErrorCode::NOSUPPORT),
        }
    }

    fn allocate_grant(&self, processid: ProcessId) -> Result<(), kernel::process::Error> {
        self.apps.enter(processid, |_, _| {})
    }
}
