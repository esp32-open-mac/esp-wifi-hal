//! Exercise the production DMA-list algorithm with host descriptors and MMIO stubs.
#![allow(dead_code)]
extern crate self as esp_hal;

macro_rules! trace {
    ($($arg:tt)*) => {};
}
macro_rules! debug {
    ($($arg:tt)*) => {};
}
macro_rules! info {
    ($($arg:tt)*) => {};
}

// The fields, completion bit and reset semantics used here match esp-hal 1.1.2.
pub mod dma {
    #[derive(Clone, Copy)]
    pub struct DmaDescriptorFlags(pub u32);
    impl DmaDescriptorFlags {
        pub fn suc_eof(&self) -> bool {
            self.0 & (1 << 30) != 0
        }
    }
    pub enum Owner {
        Cpu,
        Dma,
    }
    #[derive(Clone, Copy)]
    pub struct DmaDescriptor {
        pub flags: DmaDescriptorFlags,
        pub buffer: *mut u8,
        pub next: *mut DmaDescriptor,
    }
    impl DmaDescriptor {
        pub fn reset_for_rx(&mut self) {
            self.set_owner(Owner::Dma);
            self.flags.0 &= !(1 << 30);
            self.set_length(0);
        }
        pub fn set_size(&mut self, size: usize) {
            self.flags.0 = (self.flags.0 & !0xfff) | size as u32;
        }
        pub fn set_length(&mut self, len: usize) {
            self.flags.0 = (self.flags.0 & !0xfff000) | ((len as u32) << 12);
        }
        pub fn len(&self) -> usize {
            ((self.flags.0 >> 12) & 0xfff) as usize
        }
        pub fn set_owner(&mut self, owner: Owner) {
            self.flags.0 =
                (self.flags.0 & !(1 << 31)) | ((matches!(owner, Owner::Dma) as u32) << 31);
        }
    }
}
mod borrowed_buffer {
    pub struct BorrowedBuffer;
    impl BorrowedBuffer {
        pub const RX_CONTROL_HEADER_LENGTH: usize = 48;
    }
}
mod ll {
    use super::dma::DmaDescriptor;
    use std::{cell::Cell, ptr::NonNull};
    #[derive(Default)]
    pub struct LowLevelDriver {
        pub base: Cell<Option<NonNull<DmaDescriptor>>>,
        pub next: Cell<Option<NonNull<DmaDescriptor>>>,
        pub last: Cell<Option<NonNull<DmaDescriptor>>>,
    }
    impl LowLevelDriver {
        pub fn start_rx(&self, base: NonNull<DmaDescriptor>) {
            self.base.set(Some(base));
        }
        pub fn stop_rx(&self) {
            self.base.set(None);
        }
        pub fn set_base_rx_descriptor(&self, base: NonNull<DmaDescriptor>) {
            self.base.set(Some(base));
        }
        pub fn clear_base_rx_descriptor(&self) {
            self.base.set(None);
        }
        pub fn reload_hw_rx_descriptors(&self) {}
        pub fn next_rx_descriptor(&self) -> Option<NonNull<DmaDescriptor>> {
            self.next.get()
        }
        pub fn last_rx_descriptor(&self) -> Option<NonNull<DmaDescriptor>> {
            self.last.get()
        }
    }
}
#[path = "../../../esp-wifi-hal/src/dma_list.rs"]
mod dma_list;

use dma_list::{DmaBufferSlab, DmaList};
use std::ptr::NonNull;

fn setup() -> (
    DmaList,
    &'static ll::LowLevelDriver,
    NonNull<dma::DmaDescriptor>,
) {
    let driver = Box::leak(Box::new(ll::LowLevelDriver::default()));
    let slab = Box::leak(Box::new(DmaBufferSlab::<3, 1600>::new()));
    let (base, last) = unsafe { slab.init() };
    (DmaList::new(base, last, driver), driver, base)
}

#[test]
fn completed_short_descriptor_is_returned_for_recycling_before_the_next_frame() {
    for length in [0, 1, 24, 47] {
        let (mut list, driver, mut first) = setup();
        let second = unsafe { NonNull::new(first.as_ref().next).unwrap() };
        unsafe {
            first.as_mut().set_length(length);
            first.as_mut().flags.0 |= 1 << 30;
            (*second.as_ptr()).set_length(72);
            (*second.as_ptr()).flags.0 |= 1 << 30;
        }
        let discarded = list.take_first().expect("Completed short head stalled RX");
        assert_eq!(discarded.len(), length);
        driver.next.set(Some(second));
        list.recycle(discarded);
        let valid = list
            .take_first()
            .expect("Frame behind short descriptor was stranded");
        assert_eq!(valid.len(), 72);
    }
}

#[test]
fn incomplete_descriptor_is_left_for_hardware() {
    let (mut list, driver, base) = setup();
    assert!(list.take_first().is_none());
    assert_eq!(driver.base.get(), Some(base));
}

#[test]
fn empty_s3_hardware_queue_restarts_from_returned_descriptor() {
    let (mut list, driver, mut first) = setup();
    unsafe {
        first.as_mut().set_length(72);
        first.as_mut().flags.0 |= 1 << 30;
    }
    let returned = list.take_first().unwrap();
    driver.next.set(None); // S3 zero offset means hardware queue empty.
    driver.last.set(None);
    list.recycle(returned);
    assert_eq!(driver.base.get(), Some(first));
}

#[test]
fn returns_never_link_to_borrowed_descriptors_or_form_a_cycle() {
    for reverse in [false, true] {
        let (mut list, driver, mut first) = setup();
        let second = unsafe { NonNull::new(first.as_ref().next).unwrap() };
        let third = unsafe { NonNull::new(second.as_ref().next).unwrap() };
        unsafe {
            first.as_mut().set_length(72);
            first.as_mut().flags.0 |= 1 << 30;
            (*second.as_ptr()).set_length(72);
            (*second.as_ptr()).flags.0 |= 1 << 30;
        }
        let a = list.take_first().unwrap();
        let b = list.take_first().unwrap();
        let (returned, borrowed, returned_ptr, borrowed_ptr) = if reverse {
            (b, a, second, first)
        } else {
            (a, b, first, second)
        };
        driver.next.set(Some(third));
        list.recycle(returned);
        unsafe {
            assert_eq!((*third.as_ptr()).next, returned_ptr.as_ptr());
            assert!(
                (*returned_ptr.as_ptr()).next.is_null(),
                "Recycled descriptor points to a borrowed buffer"
            );
        }
        list.recycle(borrowed);
        unsafe {
            assert_eq!((*returned_ptr.as_ptr()).next, borrowed_ptr.as_ptr());
            assert!(
                (*borrowed_ptr.as_ptr()).next.is_null(),
                "Stale next link makes a hardware loop"
            );
        }
    }
}
