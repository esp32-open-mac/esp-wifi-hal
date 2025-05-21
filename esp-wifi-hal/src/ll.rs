//! Low Level functions.
//!
//! All functions in this module are unsafe, since their effect may be very context dependent.
use core::ptr::NonNull;

use esp_hal::{dma::DmaDescriptor, peripherals::WIFI};

/// Tell the hardware to reload the RX descriptors.
///
/// This will spin, until the bit is clear again.
pub unsafe fn reload_hw_rx_descriptors() {
    let reg = WIFI::regs().rx_ctrl();

    // Start the reload.
    reg.modify(|_, w| w.rx_descr_reload().set_bit());

    // Wait for the hardware descriptors to no longer be in reload.
    while reg.read().rx_descr_reload().bit() {}
}
/// Set the base descriptor.
///
/// If `base_descriptor` is [None], this will reset the register back to zero.
pub unsafe fn set_base_rx_descriptor(base_descriptor: Option<NonNull<DmaDescriptor>>) {
    let reg = WIFI::regs().rx_dma_list().rx_descr_base();
    if let Some(ptr) = base_descriptor {
        reg.write(|w| w.bits(ptr.as_ptr() as u32));
    } else {
        reg.reset();
    }
    reload_hw_rx_descriptors();
}
/// Get the raw value of the next RX descriptor register.
///
/// # Safety
/// This could be unsafe, since we don't know what happens if we read the register, while the
/// peripheral is powered down.
pub unsafe fn next_rx_descriptor_raw() -> *mut DmaDescriptor {
    WIFI::regs().rx_dma_list().rx_descr_next().read().bits() as *mut DmaDescriptor
}
/// Get the raw value of the last RX descriptor register.
///
/// # Safety
/// This could be unsafe, since we don't know what happens if we read the register, while the
/// peripheral is powered down.
pub unsafe fn last_rx_descriptor_raw() -> *mut DmaDescriptor {
    WIFI::regs().rx_dma_list().rx_descr_last().read().bits() as *mut DmaDescriptor
}

/// Get the last RX descriptor.
///
/// # Safety
/// This could be unsafe, since we don't know what happens if we read the register, while the
/// peripheral is powered down.
pub unsafe fn last_rx_descriptor() -> Option<NonNull<DmaDescriptor>> {
    NonNull::new(last_rx_descriptor_raw())
}
/// Enable or disable RX.
pub unsafe fn set_rx_enabled(enabled: bool) {
    WIFI::regs()
        .rx_ctrl()
        .modify(|_, w| w.rx_enable().bit(enabled));
}
/// Initialize RX.
///
/// This will set the base descriptor, reload the hardware descriptors and enable RX.
///
/// # Safety
/// The specified pointer must point to a valid DMA descriptor, with itself and it's associated
/// buffer in internal RAM.
pub unsafe fn init_rx(base_descriptor: NonNull<DmaDescriptor>) {
    set_base_rx_descriptor(Some(base_descriptor));
    set_rx_enabled(true);
}
/// Disable RX.
///
/// This will disable RX, zero the DMA list base descriptor and reload the hardware DMA list descriptors.
pub unsafe fn deinit_rx() {
    set_rx_enabled(false);
    set_base_rx_descriptor(None);
}

/// Convert a software slot index to the hardware slot index.
pub fn hw_slot_index(slot: usize) -> usize {
    4 - slot
}
/// Enable or disable the specified TX slot.
pub unsafe fn set_tx_slot_enabled(slot: usize, enabled: bool) {
    WIFI::regs()
        .tx_slot_config(hw_slot_index(slot))
        .plcp0()
        .modify(|_, w| w.slot_enabled().bit(enabled));
}
/// Validate or invalidate the specified TX slot.
pub unsafe fn set_tx_slot_validity(slot: usize, valid: bool) {
    WIFI::regs()
        .tx_slot_config(hw_slot_index(slot))
        .plcp0()
        .modify(|_, w| w.slot_valid().bit(valid));
}

#[inline(always)]
pub unsafe fn process_tx_completions(mut cb: impl FnMut(usize)) {
    let wifi = WIFI::regs();
    wifi.txq_state()
        .tx_complete_status()
        .read()
        .slot_iter()
        .enumerate()
        .filter_map(|(slot, r)| r.bit().then_some(slot))
        .for_each(|slot| {
            wifi.txq_state()
                .tx_complete_clear()
                .modify(|_, w| w.slot(slot as u8).set_bit());
            (cb)(slot);
        });
}
#[inline(always)]
pub unsafe fn process_tx_timeouts(mut cb: impl FnMut(usize)) {
    let wifi = WIFI::regs();
    wifi.txq_state()
        .tx_error_status()
        .read()
        .slot_timeout_iter()
        .enumerate()
        .filter_map(|(slot, r)| r.bit().then_some(slot))
        .for_each(|slot| {
            wifi.txq_state()
                .tx_error_clear()
                .modify(|_, w| w.slot_timeout(slot as u8).set_bit());
            (cb)(slot);
        });
}
#[inline(always)]
pub unsafe fn process_tx_collisions(mut cb: impl FnMut(usize)) {
    let wifi = WIFI::regs();
    wifi.txq_state()
        .tx_error_status()
        .read()
        .slot_collision_iter()
        .enumerate()
        .filter_map(|(slot, r)| r.bit().then_some(slot))
        .for_each(|slot| {
            wifi.txq_state()
                .tx_error_clear()
                .modify(|_, w| w.slot_collision(slot as u8).set_bit());
            (cb)(slot);
        });
}
