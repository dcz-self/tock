//! Interface for reading, writing, and erasing flash storage pages.
//!
//! Operates on single pages. The page size is set by the associated type
//! `page`. Here is an example of a page type and implementation of this trait:
//!
//! ```rust
//! use core::ops::{Index, IndexMut};
//!
//! use kernel::hil;
//! use kernel::hil::flash::Page;
//! use kernel::ErrorCode;
//!
//! // Size in bytes
//! const PAGE_SIZE: u32 = 1024;
//!
//! struct NewChipPage(pub [u8; PAGE_SIZE as usize]);
//!
//! impl Default for NewChipPage {
//!     fn default() -> Self {
//!         Self {
//!             0: [0; PAGE_SIZE as usize],
//!         }
//!     }
//! }
//!
//! impl NewChipPage {
//!     fn len(&self) -> usize {
//!         self.0.len()
//!     }
//! }
//!
//! impl AsMut<[u8]> for NewChipPage {
//!     fn as_mut(&mut self) -> &mut [u8] {
//!         &mut self.0
//!     }
//! }
//!
//! impl<const W: usize, const E: usize> hil::flash::Flash<W, E> for NewChipStruct {
//!     fn read(
//!         &self,
//!         region: &Page,
//!         buf: &'static mut [u8],
//!     ) -> Result<(), (ErrorCode, &'static mut [u8])> {
//!        unimplemented!()
//!     }
//!
//!     fn get_read_region(&self, address: u64, length: u32) -> Result<Page, ErrorCode> {
//!        unimplemented!()
//!     }
//!
//!     fn write(
//!         &self,
//!         region: &Page,
//!         buf: &'static mut [u8],
//!     ) -> Result<(), (ErrorCode, &'static mut [u8])> {
//!         unimplemented!()
//!     }
//!
//!     fn get_write_region(&self, address: u64, length: u32) -> Result<Page, ErrorCode> {
//!        unimplemented!()
//!     }
//!
//!     fn erase(&self, region: &Page) -> Result<(), ErrorCode> {
//!         unimplemented!()
//!     }
//!
//!     fn get_erase_region(&self, address: u64, length: u32) -> Result<Page, ErrorCode> {
//!        unimplemented!()
//!     }
//! }
//!
//! struct NewChipStruct {};
//!
//! impl<'a, C> hil::flash::HasClient<'a, C> for NewChipStruct {
//!     fn set_client(&'a self, client: &'a C) { }
//! }
//!
//! impl hil::flash::LegacyFlash for NewChipStruct {
//!     type Page = NewChipPage;
//!
//!     fn read_page(&self, page_number: usize, buf: &'static mut Self::Page) -> Result<(), (ErrorCode, &'static mut Self::Page)> { Err((ErrorCode::FAIL, buf)) }
//!     fn write_page(&self, page_number: usize, buf: &'static mut Self::Page) -> Result<(), (ErrorCode, &'static mut Self::Page)> { Err((ErrorCode::FAIL, buf)) }
//!     fn erase_page(&self, page_number: usize) -> Result<(), ErrorCode> { Err(ErrorCode::FAIL) }
//! }
//! ```
//!
//! A user of this flash interface might look like:
//!
//! ```rust
//! use kernel::utilities::cells::TakeCell;
//! use kernel::hil;
//! use kernel::ErrorCode;
//!
//! pub struct FlashUser<'a, F: hil::flash::Flash<W, E> + 'static, const W: usize, const E: usize> {
//!     driver: &'a F,
//!     buffer: TakeCell<'static, [u8; W]>,
//! }
//!
//! impl<'a, F: hil::flash::Flash<W, E>, const W: usize, const E: usize> FlashUser<'a, F, W, E> {
//!     pub fn new(driver: &'a F, buffer: &'static mut [u8; W]) -> FlashUser<'a, F, W, E> {
//!         FlashUser {
//!             driver: driver,
//!             buffer: TakeCell::new(buffer),
//!         }
//!     }
//! }
//!
//! impl<'a, F: hil::flash::Flash<W, E>, const W: usize, const E: usize> hil::flash::Client<W, E> for FlashUser<'a, F, W, E> {
//!     fn read_complete(&self, read_buffer: &'static mut [u8], ret: Result<(), ErrorCode>) {}
//!     fn write_complete(&self, write_buffer: &'static mut [u8], ret: Result<(), ErrorCode>) {}
//!     fn erase_complete(&self, ret: Result<(), ErrorCode>) {}
//! }
//! ```

use crate::ErrorCode;

/// Flash errors returned in the callbacks.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Error {
    /// Success.
    CommandComplete,

    /// An error occurred during the flash operation.
    FlashError,
}

pub trait HasClient<'a, C> {
    /// Set the client for this flash peripheral. The client will be called
    /// when operations complete.
    fn set_client(&'a self, client: &'a C);
}

/// A page of writable persistent flash memory.
pub trait LegacyFlash {
    /// Type of a single flash page for the given implementation.
    type Page: AsMut<[u8]> + Default;

    /// Read a page of flash into the buffer.
    fn read_page(
        &self,
        page_number: usize,
        buf: &'static mut Self::Page,
    ) -> Result<(), (ErrorCode, &'static mut Self::Page)>;

    /// Write a page of flash from the buffer.
    fn write_page(
        &self,
        page_number: usize,
        buf: &'static mut Self::Page,
    ) -> Result<(), (ErrorCode, &'static mut Self::Page)>;

    /// Erase a page of flash by setting every byte to 0xFF.
    fn erase_page(&self, page_number: usize) -> Result<(), ErrorCode>;
}

/// Implement `Client` to receive callbacks from `LegacyFlash`.
pub trait LegacyClient<F: LegacyFlash> {
    /// Flash read complete.
    fn read_complete(&self, read_buffer: &'static mut F::Page, error: Error);

    /// Flash write complete.
    fn write_complete(&self, write_buffer: &'static mut F::Page, error: Error);

    /// Flash erase complete.
    fn erase_complete(&self, error: Error);
}

/// An index to a block within device composed of `S`-sized blocks.
pub struct BlockIndex<const S: u32>(pub u32, PhantomData<[(), S]>);

impl<const S: u32> BlockIndex<R> {
    /// Returns the index to the block at `index`.
    pub fn new(index: u32) -> Self {
        Self(index, Default::default())
    }
    /// Returns the index that contains the address.
    pub fn new_containing(address: u64) -> Self {
        Self::new(address / S)
    }

    /// Returns the index starting at the given address, if any.
    pub fn new_starting_at(address: u64) -> Option<Self> {
        if address % S == 0 {
            Some(Self::new(address / S))
        } else { None }
    }
}

impl<const S: u32> From<BlockIndex<S>> for u64 {
    fn from(index: BlockIndex<S>) -> Self {
        index.0 as u64 * S as u64
    }
}

/// A memory region composed of consecutive `S`-sized blocks.
pub struct Region<const S: u32>{
    pub index: BlockIndex<S>,
    pub length_blocks: u32,
}

impl<const S: u32> Region<S> {
    /// Returns the smallest region made of blocks which contains
    /// the specified range of addresses.
    pub fn new_containing(range: AddressRange) -> Self {
        let index = BlockIndex::new_containing(range.start_address);
        Self {
            index,
            length_blocks: {
                (range.start_address % E + range.length_bytes)
                    .div_ceil(E) // CAUTION: nightly
            },
        }
    }

    /// Returns the region starting and ending exactly in the same places
    /// as the given `range`, if such exists.
    pub fn new_exact(range: AddressRange) -> Option<Self> {
        BlockIndex::new_starting_at(range.start_address)
            .and_then(|index| {
                if range.length_bytes % size == 0 {
                    Some(Self {
                        index,
                        length_blocks = range.length_bytes / size,
                    })
                } else {
                    None
                }
            })
    }
    
    pub fn get_length_bytes(&self) -> u32 {
        self.length_blocks * S
    }
}

/// Specifies a storage area with byte granularity.
pub struct AddressRange {
    /// Offset from the beginning of the storage device.
    pub start_address: u64,
    /// Length of the range.
    pub length_bytes: u32,
}

impl<const S: u32> From<Region<S>> for AddressRange {
    fn from(region: Region<S>) -> Self {
        Self {
            start_address: region.index.into(),
            length_bytes: region.get_length_bytes(),
        }
    }
}

/// Writeable persistent flash memory device.
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
pub trait Flash<const W: usize, const E: usize> {
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
    fn read(
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
    fn read_blocks(
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
    /// i.e. each byte turns to 0xFF.
    ///
    /// If `region` exceeds the size of the device, returns `ErrorCode::INVAL`.
    /// On success, triggers `Client::erase_complete` once.
    fn erase(&self, region: &Region<E>) -> Result<(), ErrorCode>;

    /// Return the size of the device in bytes.
    fn get_size(&self) -> Result<u64, ErrorCode>;
}

/// Implement `Client` to receive callbacks from `Flash`.
pub trait Client<const W: usize, const E: usize> {
    /// Flash read complete.
    ///
    /// This will be called when the read operation is complete.
    /// On success `ret` will be nothing.
    /// On error `ret` will contain a `ErrorCode`
    fn read_complete(&self, read_buffer: &'static mut [u8], ret: Result<(), ErrorCode>);

    /// Flash write complete.
    ///
    /// This will be called when the write operation is complete.
    /// On success `ret` will be nothing.
    /// On error `ret` will contain a `ErrorCode`
    fn write_complete(&self, write_buffer: &'static mut [u8], ret: Result<(), ErrorCode>);

    /// Flash erase complete.
    ///
    /// This will be called when the erase operation is complete.
    /// On success `ret` will be nothing.
    /// On error `ret` will contain a `ErrorCode`
    fn erase_complete(&self, ret: Result<(), ErrorCode>);
}
