use crate::{ll, DefaultRawMutex};
use core::{cell::RefCell, marker::PhantomData, mem::MaybeUninit, ptr::NonNull};

use embassy_sync::blocking_mutex;
use esp_hal::dma::{DmaDescriptor, DmaDescriptorFlags, Owner};
use esp_wifi_sys::include::wifi_pkt_rx_ctrl_t;

trait DmaDescriptorExt {
    fn next(&mut self) -> Option<&mut DmaDescriptor>;
    fn set_next(&mut self, next: Option<&mut DmaDescriptor>);
}
impl DmaDescriptorExt for DmaDescriptor {
    fn next(&mut self) -> Option<&mut DmaDescriptor> {
        unsafe { self.next.as_mut() }
    }
    fn set_next(&mut self, next: Option<&mut DmaDescriptor>) {
        self.next = next
            .map(|next| next as *mut DmaDescriptor)
            .unwrap_or_default();
    }
}

const RX_BUFFER_SIZE: usize = 1600;

/// Resources for the Wi-Fi peripheral.
///
/// `BUFFER_COUNT` has to be at least 2.
pub struct WiFiResources<const BUFFER_COUNT: usize> {
    buffers: [[u8; RX_BUFFER_SIZE]; BUFFER_COUNT],
    dma_descriptors: [MaybeUninit<DmaDescriptor>; BUFFER_COUNT],
    dma_list: MaybeUninit<blocking_mutex::Mutex<DefaultRawMutex, RefCell<DMAList<'static>>>>,
}
impl<const BUFFER_COUNT: usize> WiFiResources<BUFFER_COUNT> {
    /// Create new DMA resources.
    pub const fn new() -> Self {
        assert!(BUFFER_COUNT >= 2, "BUFFER_COUNT has to be larger than 2.");
        Self {
            buffers: [[0u8; RX_BUFFER_SIZE]; BUFFER_COUNT],
            dma_descriptors: [MaybeUninit::uninit(); BUFFER_COUNT],
            dma_list: MaybeUninit::uninit(),
        }
    }
    /// Initialize the DMA resources.
    ///
    /// SAFETY:
    /// You must ensure, that this is only used as long as the DMA list lives!
    pub(crate) unsafe fn init(
        &mut self,
    ) -> &blocking_mutex::Mutex<DefaultRawMutex, RefCell<DMAList<'static>>> {
        // We already asserted this in `Self::new`, but maybe this helps LLVM.
        assert!(self.dma_descriptors.len() >= 2);

        // We iterate over and initialize the DMA descriptors in reverse, so that we can set the
        // `next` field directly.
        let mut init_iter = self.dma_descriptors.iter_mut().enumerate().rev().scan(
            core::ptr::null_mut(),
            |next, (i, dma_descriptor)| {
                let buffer = &raw mut self.buffers[i] as *mut u8;

                let mut descriptor = DmaDescriptor {
                    flags: DmaDescriptorFlags(0),
                    buffer,
                    next: *next,
                };
                descriptor.reset_for_rx();
                descriptor.set_size(RX_BUFFER_SIZE);
                descriptor.set_owner(Owner::Dma);

                let descriptor_ref = dma_descriptor.write(descriptor);

                *next = descriptor_ref as *mut DmaDescriptor;

                Some(descriptor_ref)
            },
        );

        // Since the iterator is reversed, the last descriptor is the first item of the iterator
        // and vice versa.
        let last_ptr = NonNull::from(init_iter.next().unwrap());
        let base_ptr = NonNull::from(init_iter.last().unwrap());

        self.dma_list
            .write(blocking_mutex::Mutex::new(RefCell::new(DMAList::new(
                base_ptr, last_ptr,
            ))))
    }
}
impl<const BUFFER_COUNT: usize> Default for WiFiResources<BUFFER_COUNT> {
    fn default() -> Self {
        Self::new()
    }
}

pub struct DMAList<'res> {
    rx_chain_ptrs: Option<(NonNull<DmaDescriptor>, NonNull<DmaDescriptor>)>,
    _phantom: PhantomData<&'res ()>,
}
impl<'res> DMAList<'res> {
    /// Instantiate a new DMAList.
    fn new(base_ptr: NonNull<DmaDescriptor>, last_ptr: NonNull<DmaDescriptor>) -> Self {
        unsafe { ll::init_rx(base_ptr) };

        trace!("Initialized DMA list.");
        Self::log_stats();
        Self {
            rx_chain_ptrs: Some((base_ptr, last_ptr)),
            _phantom: PhantomData,
        }
    }
    /// Sets [Self::rx_chain_ptrs], with the `dma_list_descriptor` at the base.
    ///
    /// This will automatically reload the RX descriptors.
    fn set_rx_chain_base(&mut self, base_descriptor: Option<NonNull<DmaDescriptor>>) {
        match (&mut self.rx_chain_ptrs, base_descriptor) {
            // If neither the DMA list nor the DMA list descriptor is empty, we simply set rx_chain_begin to dma_list_desciptor.
            (Some(rx_chain_ptrs), Some(dma_list_descriptor)) => {
                rx_chain_ptrs.0 = dma_list_descriptor;
            }
            // If the DMA list isn't empty, but we want to set it to empty.
            (Some(_), None) => self.rx_chain_ptrs = None,
            // The DMA list is currently empty. Therefore the dma_list_descriptor is now first and last.
            (None, Some(dma_list_descriptor)) => {
                self.rx_chain_ptrs = Some((dma_list_descriptor, dma_list_descriptor))
            }
            _ => {}
        }
        unsafe {
            ll::set_base_rx_descriptor(base_descriptor);
        }
    }
    /// Take the first [DMAListItem] out of the list.
    pub fn take_first(&mut self) -> Option<&'res mut DmaDescriptor> {
        let first = unsafe { self.rx_chain_ptrs?.0.as_mut() };
        trace!("Taking buffer: {:x} from DMA list.", first as *mut _ as u32);
        if first.flags.suc_eof() && first.len() >= size_of::<wifi_pkt_rx_ctrl_t>() {
            let next = first.next();
            if next.is_none() {
                trace!("Next was none.");
            };
            self.set_rx_chain_base(next.map(NonNull::from));
            first.set_owner(Owner::Cpu);

            Some(first)
        } else {
            None
        }
    }
    /// Clear the DMA list.
    ///
    /// This will only mark the buffers as empty, not clear them.
    pub fn clear(&mut self) {
        let Some(mut current) = self
            .rx_chain_ptrs
            .map(|mut ptrs| unsafe { ptrs.0.as_mut() })
        else {
            return;
        };
        while current.flags.suc_eof() {
            current.reset_for_rx();
            let Some(next) = current.next() else {
                break;
            };
            current = next;
        }
        self.set_rx_chain_base(self.rx_chain_ptrs.map(|ptrs| ptrs.0));
    }
    /// Returns a [DMAListItem] to the end of the list.
    pub fn recycle(&mut self, dma_list_descriptor: &mut DmaDescriptor) {
        dma_list_descriptor.reset_for_rx();
        trace!(
            "Returned buffer: {:x} to DMA list.",
            dma_list_descriptor as *mut _ as u32
        );

        // If the DMA list is not empty, we attach the descriptor to the end, reload the hardware
        // descriptors and set the last pointer to the descriptor, under some weird conditions.
        if let Some((_, ref mut last_ptr)) = self.rx_chain_ptrs {
            unsafe {
                last_ptr.as_mut().set_next(Some(dma_list_descriptor));

                ll::reload_hw_rx_descriptors();

                if ll::next_rx_descriptor_raw() as u32 != 0x3ff00000
                    || ll::last_rx_descriptor().map(NonNull::as_ptr) == Some(dma_list_descriptor)
                {
                    *last_ptr = NonNull::new(dma_list_descriptor).unwrap();
                    return;
                }
            }
        }
        // If the DMA list is empty, we make this descriptor the base.
        self.set_rx_chain_base(NonNull::new(dma_list_descriptor));
    }
    pub fn log_stats() {
        #[allow(unused)]
        unsafe {
            let (rx_next, rx_last) = (
                ll::next_rx_descriptor_raw() as u32,
                ll::last_rx_descriptor_raw() as u32,
            );
            trace!("DMA list: Next: {:x} Last: {:x}", rx_next, rx_last);
        }
    }
}
impl Drop for DMAList<'_> {
    fn drop(&mut self) {
        unsafe {
            ll::deinit_rx();
        }
    }
}
unsafe impl Send for DMAList<'_> {}
