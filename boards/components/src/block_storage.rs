//! Component for block storage drivers.
//!
//! This provides one component, BlockStorageComponent, which provides
//! a system call inteface to block storage.
//!
//! Usage
//! -----
//! ```rust
//! let block_storage = components::block_storage::BlockStorageComponent::new(
//!     &flash_device,
//! )
//! .finalize(components::nv_storage_component_helper!());
//! ```

use capsules::block_storage_driver;
use core::mem::MaybeUninit;
use kernel::capabilities;
use kernel::component::Component;
use kernel::create_capability;
use kernel::hil;
use kernel::{static_init, static_init_half};

// Setup static space for the objects.
#[macro_export]
macro_rules! block_storage_component_helper {
    () => {{
        use capsules::block_storage_driver::BlockStorage;
        use core::mem::MaybeUninit;
        use kernel::hil;
        static mut BUF1: MaybeUninit<[u8; W as usize]> = MaybeUninit::uninit();
        static mut BUF2: MaybeUninit<[u8; W as usize]> = MaybeUninit::uninit();
        (&mut BUF1, &mut BUF2)
    };};
}

pub struct BlockStorageComponent<B, const W: u32, const E: u32>
    where B: 'static
        + hil::block_storage::BlockStorage<W, E>
        //+ hil::block_storage::HasClient<'static, >,
{
    pub board_kernel: &'static kernel::Kernel,
    pub driver_num: usize,
    pub device: &'static B,
}

impl<B, const W: u32, const E: u32> Component for BlockStorageComponent<B, W, E>
    where B: 'static
        + hil::block_storage::BlockStorage<W, E>
        + hil::block_storage::HasClient<
            'static,
            block_storage_driver::BlockStorage<'static, B, W, E>
        >,
{
    type StaticInput = (
        &'static mut MaybeUninit<[u8; W as usize]>,
        &'static mut MaybeUninit<[u8; W as usize]>,
    );
    type Output = &'static block_storage_driver::BlockStorage<'static, B, W, E>;

    unsafe fn finalize(self, static_buffer: Self::StaticInput) -> Self::Output {
        let grant_cap = create_capability!(capabilities::MemoryAllocationCapability);

        let read_buffer = static_init_half!(
            static_buffer.0,
            [u8; W as usize],
            [0; W as usize],
        );

        let write_buffer = static_init_half!(
            static_buffer.1,
            [u8; W as usize],
            [0; W as usize],
        );
        
        let mut u: MaybeUninit<block_storage_driver::BlockStorage<'static, B, W, E>> = MaybeUninit::uninit();
        
        let syscall_driver = static_init_half!(
            &mut u,
            block_storage_driver::BlockStorage<'static, B, W, E>,
            block_storage_driver::BlockStorage::new(
                self.device,
                self.board_kernel.create_grant(self.driver_num, &grant_cap),
                write_buffer,
            )
        );
        
        hil::block_storage::HasClient::set_client(self.device, syscall_driver);
        syscall_driver
    }
}
