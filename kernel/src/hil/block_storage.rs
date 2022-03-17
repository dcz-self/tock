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

impl <const S: usize> Add<u32> for BlockIndex<S> {
    type Output = Self;
    fn add(self, other: u32) -> Self {
        BlockIndex(self.0 + other)
    }
}

/// A memory region composed of consecutive `S`-sized blocks.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Region<const S: usize> {
    pub index: BlockIndex<S>,
    pub length_blocks: u32,
}

/// Divide, round up
fn div_ceil(a: u64, b: u64) -> u64 {
    a / b // FIXME: trololo
}

impl<const S: usize> Region<S> {
    /// Returns the smallest region made of blocks which contains
    /// the specified range of addresses.
    pub fn new_containing(range: AddressRange) -> Self {
        let index = BlockIndex::new_containing(range.start_address);
        Self {
            index,
            length_blocks: {
                div_ceil(
                    range.start_address % S as u64 + range.length_bytes as u64,
                    S as u64,
                ) as u32
                // CAUTION: div_ceil is nightly
            },
        }
    }

    /// Returns the region starting and ending exactly in the same places
    /// as the given `range`, if such exists.
    pub fn new_exact(range: AddressRange) -> Option<Self> {
        BlockIndex::new_starting_at(range.start_address).and_then(|index| {
            if range.length_bytes % S as u32 == 0 {
                Some(Self {
                    index,
                    length_blocks: range.length_bytes / S as u32,
                })
            } else {
                None
            }
        })
    }

    pub fn get_length_bytes(&self) -> u32 {
        self.length_blocks * S as u32
    }
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

impl<const S: usize> From<Region<S>> for AddressRange {
    fn from(region: Region<S>) -> Self {
        let length_bytes = region.get_length_bytes();
        Self {
            start_address: region.index.into(),
            length_bytes,
        }
    }
}

/// Writeable persistent block storage device.
///
/// The device is formed from equally-sized storage blocks,
/// which are arranged one after another, without gaps or overlaps,
/// to form a linear storage of bytes.
///
/// There device is split into blocks in two ways, into:
/// - erase blocks, which are the smallest unit of space that can be erased
/// - write blocks, which are the smallest unit of space that can be written
///
/// Every byte on the device belongs to exactly one erase block,
/// and to exactly one write block at the same time.
///
/// `W`: The size of a write block in bytes.
/// `E`: The size of an erase block in bytes.
pub trait BlockStorage<const W: usize, const E: usize> {
    /// Read data from flash into a buffer.
    ///
    /// This function will read data stored in flash at `range` into `buf`.
    ///
    /// `ErrorCode::INVAL` will be returned if
    /// - `range` exceeds the end of the device, or
    /// - `buf` is shorter than `range`.
    ///
    /// On success, triggers `Client::read_complete` once.
    /// On failure returns a `ErrorCode` and the buffer passed in.
    /// If `ErrorCode::NOSUPPORT` is returned then `read_block`
    /// should be used instead.
    fn read_range(
        &self,
        range: &AddressRange,
        buf: &'static mut [u8],
    ) -> Result<(), (ErrorCode, &'static mut [u8])>;

    /// Read data from blocks starting at `block`, and into the buffer.
    ///
    /// `ErrorCode::INVAL` will be returned if
    /// - `range` exceeds the end of the device, or
    /// - `buf` is shorter than `region`.
    ///
    /// On success, triggers `Client::read_complete` once.
    fn read(
        &self,
        region: &Region<W>,
        buf: &'static mut [u8],
    ) -> Result<(), (ErrorCode, &'static mut [u8])>;

    /// Write data from a buffer to flash.
    ///
    /// This function writes the contents of `buf` to memory,
    /// starting at the chosen `start_block`.
    ///
    /// `ErrorCode::INVAL` will be returned if
    /// - `range` exceeds the end of the device, or
    /// - `buf` is shorter than `region`.
    ///
    /// This function SHALL NOT prepare the block for writing first.
    /// The user of this function SHOULD ensure that a block is erased
    /// before calling `write` (see `erase`).
    ///
    /// Once a byte has been written as part of a write block,
    /// it SHOULD NOT be written again until it's erased as part of an erase block.
    /// Multiple writes to the same block are possible in this trait,
    /// but the result is dependent on the nature of the underlying hardware,
    /// and this interface does not provide a way to discover that.
    ///
    /// **Note** about raw flash devices: writes can turn bits from `1` to `0`.
    /// To change a bit from `0` to `1`, a region must be erased.
    ///
    /// **Note**: some raw flash hardware only allows a limited number of writes before
    /// an erase. If that is the case, the implementation MUST return an error
    /// `ErrorCode::NOMEM` when this happens, even if the hardware silently
    /// ignores the write.
    ///
    /// On success, triggers `Client::write_complete` once.
    /// On failure returns a `ErrorCode` and the buffer passed in.
    fn write(
        &self,
        region: &Region<W>,
        buf: &'static mut [u8],
    ) -> Result<(), (ErrorCode, &'static mut [u8])>;

    /// Makes a region ready for writing.
    ///
    /// This corresponds roughly to the erase operation on raw flash devices,
    /// but may also do nothing on devices where erasure is not necessary.
    ///
    /// Calling `erase` may modify bytes in the selected `region` in any way
    /// (typically, on raw flash devices, all bits will be set,
    /// i.e. each byte turns to 0xFF).
    ///
    /// If `region` exceeds the size of the device, returns `ErrorCode::INVAL`.
    /// On success, triggers `Client::erase_complete` once.
    fn erase(&self, region: &Region<E>) -> Result<(), ErrorCode>;

    /// Returns the size of the device in bytes.
    fn get_size(&self) -> Result<u64, ErrorCode>;
}

pub trait HasClient<'a, C> {
    /// Set the client for this peripheral. The client will be called
    /// when operations complete.
    fn set_client(&'a self, client: &'a C);
}

/// Implement `Client` to receive callbacks from `BlockStorage`.
pub trait Client<const W: usize, const E: usize> {
    /// Block read complete.
    ///
    /// This will be called when the read operation is complete.
    fn read_complete(&self, read_buffer: &'static mut [u8], ret: Result<(), ErrorCode>);

    /// Block write complete.
    ///
    /// This will be called when the write operation is complete.
    fn write_complete(&self, write_buffer: &'static mut [u8], ret: Result<(), ErrorCode>);

    /// Block erase complete.
    ///
    /// This will be called when the erase operation is complete.
    fn erase_complete(&self, ret: Result<(), ErrorCode>);
}
