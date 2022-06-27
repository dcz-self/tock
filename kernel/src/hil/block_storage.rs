//! Interface for reading, writing, and erasing storage blocks.
//!
//! Operates on blocks, with an optional way to read unaligned bytes.
//!
//! Here's an example implementation for a raw flash chip:
//!
//! ```rust
//! use kernel::hil;
//! use kernel::ErrorCode;
//!
//! const WRITE_BLOCK_BYTES: u32 = 256;
//! const ERASE_BLOCK_BYTES: usize = 4096;
//!
//! struct RawFlashChip {};
//!
//! type WriteRegion = hil::block_storage::Region<WRITE_BLOCK_BYTES>;
//! type EraseRegion = hil::block_storage::Region<ERASE_BLOCK_BYTES>;
//!
//! impl hil::block_storage::BlockStorage<WRITE_BLOCK_BYTES, ERASE_BLOCK_BYTES> for RawFlashChip {
//!     (implement associated functions here)
//! }
//! ```
//!
//! Similar implementation for a flash chip with FTL, or for a magnetic drive:
//!
//! ```rust
//! use kernel::hil;
//! use kernel::ErrorCode;
//!
//! const BLOCK_BYTES: usize = 4096;
//!
//! struct SimpleBlockDevice {};
//!
//! type Region = hil::block_storage::Region<BLOCK_BYTES>;
//!
//! impl hil::block_storage::BlockStorage<BLOCK_BYTES, BLOCK_BYTES> for SimpleBlockDevice {
//!     // FTL takes care of multiple writes to the same block on flash,
//!     // and magnetic drives don't need to erase at all.
//!     fn erase(&self, region: &hil::block_storage::Region)
//!         -> Result<(), ErrorCode>
//!     { Ok(()) }
//!     
//!     (implement remaining associated functions here)
//! }
//! ```
use crate::ErrorCode;
use core::ops::Add;

/// An index to a block within device composed of `S`-sized blocks.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct BlockIndex<const S: usize>(pub u32);

impl<const S: usize> BlockIndex<S> {
    /// Returns the index that contains the address.
    pub fn new_containing(address: u64) -> Self {
        Self((address / S as u64) as u32)
    }

    /// Returns the index starting at the given address, if any.
    pub fn new_starting_at(address: u64) -> Option<Self> {
        if address % S as u64 == 0 {
            Some(Self::new_containing(address))
        } else {
            None
        }
    }
}

impl<const S: usize> From<BlockIndex<S>> for u64 {
    fn from(index: BlockIndex<S>) -> Self {
        index.0 as u64 * S as u64
    }
}

impl<const S: usize> Add<u32> for BlockIndex<S> {
    type Output = Self;
    fn add(self, other: u32) -> Self {
        BlockIndex(self.0 + other)
    }
}

/// Writeable persistent block storage device.
///
/// The device is formed from equally-sized storage blocks,
/// which are arranged one after another, without gaps or overlaps,
/// to form a linear storage of bytes.
///
/// There device is split into blocks in two ways, into:
/// - prepare blocks, which are the smallest unit of space
/// that can be prepared for writing (see `BlockStorage::prepare_write`)
/// - write blocks, which are the smallest unit of space that can be written
///
/// Every byte on the device belongs to exactly one prepare block,
/// and to exactly one write block at the same time.
///
/// `W`: The size of a write block in bytes.
/// `P`: The size of a prepare block in bytes.
pub trait BlockStorage<const W: usize, const P: usize> {
    /// Read data from a block, and into the buffer.
    ///
    /// `ErrorCode::INVAL` will be returned if
    /// - `region` exceeds the end of the device, or
    /// - `buf` is shorter than `region`.
    ///
    /// Returns `ErrorCode::BUSY` when another operation is in progress.
    ///
    /// On success, triggers `Client::read_complete` once.
    fn read(
        &self,
        region: &BlockIndex<W>,
        buf: &'static mut [u8],
    ) -> Result<(), (ErrorCode, &'static mut [u8])>;

    /// Write data from a buffer to storage.
    ///
    /// This function writes the contents of `buf` to memory,
    /// starting at the chosen `start_block`.
    ///
    /// `ErrorCode::INVAL` will be returned if
    /// - `region` exceeds the end of the device, or
    /// - `buf` is shorter than `region`.
    ///
    /// This function SHALL NOT prepare the block for writing first.
    /// The user of this function SHOULD ensure that a block is prepared
    /// before calling `write` (see `prepare_write`).
    ///
    /// Once a byte has been written as part of a write block,
    /// it SHOULD NOT be written again until it's prepared
    /// as part of a prepare block.
    /// Multiple writes to the same block are possible in this trait,
    /// but the result is dependent on the nature of the underlying hardware,
    /// and this interface does not provide a way to discover that.
    ///
    /// **Note** about raw flash devices: writes can turn bits from `1` to `0`.
    /// To change a bit from `0` to `1`, a region must be erased (prepared).
    ///
    /// **Note**: some raw flash hardware only allows a limited number of writes before
    /// an erase. If that is the case, the implementation MUST return an error
    /// `ErrorCode::NOMEM` when this happens, even if the hardware silently
    /// ignores the write.
    ///
    /// Returns `ErrorCode::BUSY` when another operation is in progress.
    ///
    /// On success, triggers `Client::write_complete` once.
    fn write(
        &self,
        region: &BlockIndex<W>,
        buf: &'static mut [u8],
    ) -> Result<(), (ErrorCode, &'static mut [u8])>;

    /// Makes a region ready for writing.
    ///
    /// This corresponds roughly to the erase operation on raw flash devices,
    /// but may also do nothing on devices where erasure is not necessary.
    ///
    /// Calling `prepare_write` may modify bytes in the selected `region`
    /// (typically, on raw flash devices, all bits will be set,
    /// i.e. each byte turns to 0xFF).
    ///
    /// If `region` exceeds the size of the device, returns `ErrorCode::INVAL`.
    ///
    /// Returns `ErrorCode::BUSY` when another operation is in progress.
    ///
    /// On success, triggers `Client::prepare_write_complete` once.
    fn prepare_write(&self, region: &BlockIndex<P>) -> Result<(), ErrorCode>;

    /// Returns the size of the device in bytes.
    fn get_size(&self) -> u64;
}

/// Specifies a storage area with byte granularity.
pub struct AddressRange {
    /// Offset from the beginning of the storage device.
    pub start_address: u64,
    /// Length of the range.
    pub length_bytes: u32,
}

impl AddressRange {
    pub fn get_end_address(&self) -> u64 {
        self.start_address + self.length_bytes as u64
    }
}

/// Devices which can read arbitrary byte-indexed ranges.
pub trait ReadRange {
    /// Read data from storage into a buffer.
    ///
    /// This function will read data stored in storage at `range` into `buf`.
    ///
    /// `ErrorCode::INVAL` will be returned if
    /// - `range` exceeds the end of the device, or
    /// - `buf` is shorter than `range`.
    ///
    /// Returns `ErrorCode::BUSY` when another operation is in progress.
    ///
    /// On success, triggers `Client::read_complete` once.
    fn read_range(
        &self,
        range: &AddressRange,
        buf: &'static mut [u8],
    ) -> Result<(), (ErrorCode, &'static mut [u8])>;
}

pub trait HasClient<'a, C> {
    /// Set the client for this peripheral. The client will be called
    /// when operations complete.
    fn set_client(&'a self, client: &'a C);
}

/// Implement `Client` to receive callbacks from `BlockStorage`.
pub trait Client<const W: usize, const E: usize> {
    /// This will be called when a read operation is complete.
    ///
    /// If the device is unable to read the region, returns `ErrorCode::FAIL`.
    ///
    /// On errors, the buffer contents are undefined.
    fn read_complete(&self, read_buffer: &'static mut [u8], ret: Result<(), ErrorCode>);

    /// This will be called when the write operation is complete.
    ///
    /// If the device is unable to write to the region,
    /// returns `ErrorCode::FAIL`.
    ///
    /// On errors, the contents of the storage region are undefined,
    /// and the region must be considered no longer prepared for writing.
    fn write_complete(&self, write_buffer: &'static mut [u8], ret: Result<(), ErrorCode>);

    /// This will be called when the erase operation is complete.
    ///
    /// If the device is unable to prepare the region,
    /// returns `ErrorCode::FAIL`.
    ///
    /// On errors, the contents of the storage region are undefined,
    /// and the region must be considered not prepared for writing.
    fn prepare_write_complete(&self, ret: Result<(), ErrorCode>);
}
