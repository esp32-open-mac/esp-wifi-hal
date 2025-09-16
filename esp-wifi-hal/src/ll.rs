//! Low Level functions.
//!
//! All functions in this module are unsafe, since their effect may be very context dependent.
use core::{iter::IntoIterator, ptr::NonNull};

use esp_hal::{
    clock::{ModemClockController, PhyClockGuard},
    dma::DmaDescriptor,
    peripherals::{LPWR, WIFI},
};
use esp_wifi_sys::include::*;

use crate::phy_init_data::PHY_INIT_DATA_DEFAULT;

#[inline]
/// Run a reversible sequence of functions either forward or in reverse.
///
/// If `forward` is `true`, the functions will be executed in the order in which they were passed.
/// Otherwise they'll be run in reverse order.
fn run_reversible_function_sequence<T, Iter>(functions: Iter, parameter: T, forward: bool)
where
    T: Copy,
    Iter: IntoIterator<Item = fn(T)>,
    <Iter as IntoIterator>::IntoIter: DoubleEndedIterator,
{
    let for_each_closure = |function: fn(T)| (function)(parameter);

    let function_iter = functions.into_iter();
    if forward {
        function_iter.for_each(for_each_closure);
    } else {
        function_iter.rev().for_each(for_each_closure);
    }
}

/// Low level driver for the Wi-Fi peripheral.
///
/// This is intended as an intermediary layer between the user facing API and the hardware, to make
/// driver maintenance easier.
///
/// # Safety
/// Pretty much all of the functions on this must be used with extreme caution, as the intention is
/// not to provide a fool proof API for the user, but a thin abstraction over the hardware, to make
/// development of the user facing API easier. A lot of the private functions do not take `&self`
/// as a parameter, as they must also be callable during initialization and aren't intended for
/// direct use anyways. Always pay close attention to their respective docs.
pub struct LowLevelDriver<'d> {
    /// The Wi-Fi peripheral.
    wifi: WIFI<'d>,
    /// Prevents the PHY clock from being disabled, while the driver is running.
    phy_clock_guard: PhyClockGuard<'d>,
}
impl<'d> LowLevelDriver<'d> {
    /// Create a new [LowLevelDriver].
    pub fn new(mut wifi: WIFI<'d>) -> Self {
        let phy_clock_guard = unsafe { Self::init(&mut wifi) };
        Self {
            wifi,
            phy_clock_guard,
        }
    }

    // Initialization

    /// Set the power and reset status of the Wi-Fi peripheral module.
    ///
    /// # Safety
    /// Enabling or disabling the Wi-Fi PD, outside of init or deinit, breaks a lot of assumptions
    /// other code makes, so take great care.
    unsafe fn set_module_status(enable_module: bool) {
        // For newer chips, we also have to handle the reset of the RF modules.
        run_reversible_function_sequence(
            [
                // Enable or disable forced power down
                |power_down| {
                    LPWR::regs()
                        .dig_pwc()
                        .modify(|_, w| w.wifi_force_pd().bit(power_down));
                },
                // Enable or disable isolation
                |isolate| {
                    LPWR::regs()
                        .dig_iso()
                        .modify(|_, w| w.wifi_force_iso().bit(isolate));
                },
            ],
            !enable_module,
            enable_module,
        );
    }
    /// Initialize the PHY.
    ///
    /// NOTE: This is very temporary and will be replaced, as soon as `esp-phy` is released.
    unsafe fn initialize_phy() -> PhyClockGuard<'d> {
        let clock_guard = unsafe { WIFI::steal() }.enable_phy_clock();
        let mut cal_data = [0u8; size_of::<esp_phy_calibration_data_t>()];
        let init_data = &PHY_INIT_DATA_DEFAULT;
        trace!("Enabling PHY.");
        unsafe {
            register_chipv7_phy(
                init_data,
                &mut cal_data as *mut _ as *mut esp_phy_calibration_data_t,
                esp_phy_calibration_mode_t_PHY_RF_CAL_FULL,
            );
        }
        clock_guard
    }
    /// Perform a full MAC reset.
    ///
    /// # Safety
    /// This will basically reset all state of the MAC to its defaults. Therefore it may also break
    /// assumptions made by some pieces of code.
    unsafe fn reset_mac() {
        // Perform a full reset of the Wi-Fi module.
        unsafe { WIFI::steal() }.reset_wifi_mac();
        // NOTE: coex_bt_high_prio would usually be here

        // Disable RX
        unsafe {
            Self::set_rx_status(false);
        }
    }
    /// Initialize or deinitialize the MAC.
    ///
    /// If `intialized` is `true`, the MAC will be initialized, and vice versa.
    /// # Safety
    /// We don't really know, what exactly this does, so we only call it, where the closed source
    /// driver would.
    unsafe fn set_mac_state(intialized: bool) {
        // TODO: Figure out, what these registers do precisely and move them into the PAC.
        cfg_if::cfg_if! {
            if #[cfg(feature = "esp32")] {
                const MAC_INIT_MASK: u32 = 0xffffe800;
                const MAC_READY_MASK: u32 = 0x2000;
            } else if #[cfg(feature = "esp32s2")] {
                const MAC_INIT_MASK: u32 = 0xff00efff;
                const MAC_READY_MASK: u32 = 0x6000;
            } else {
                compile_error!("The MAC init mask may have to be updated for different chips.");
            }
        }
        // Spin until the MAC state is marked as ready.
        // This is only required on the ESP32-S2.
        while !intialized
            && cfg!(feature = "esp32s2")
            && WIFI::regs().ctrl().read().bits() & MAC_READY_MASK == 0
        {}
        // If we are initializing the MAC, we need to clear some bits, by masking the reg, with the
        // MAC_INIT_MASK. On deinit, we need to set the bits we previously cleared.
        WIFI::regs().ctrl().modify(|r, w| unsafe {
            w.bits(if intialized {
                r.bits() & MAC_INIT_MASK
            } else {
                r.bits() | !MAC_INIT_MASK
            })
        });

        // Spin until the MAC is no longer ready.
        while !intialized && WIFI::regs().ctrl().read().bits() & MAC_READY_MASK != 0 {}
    }
    /// Initialize the hardware.
    ///
    /// As modem sleep is currently unimplemented, this does a full initialization.
    ///
    /// This includes:
    /// - Enabling the Wi-Fi power domain and modem clock
    /// - Initializing the PHY
    /// - Initializing the MAC
    /// - Setting up basic hardware functionality, like the MAC RX filters and crypto.
    ///
    /// What it doesn't do:
    /// - Enabling RX and setting up the DMA list
    /// - Setting any addresses
    /// - Setting an interrupt handler
    ///
    /// # Safety
    /// At the moment, this is only intended to be called upon initializing the low level driver.
    /// Reinitialization and partial PHY calibration are NOT handled at all. The effects of calling
    /// this multiple times are unknown.
    unsafe fn init(wifi: &mut WIFI<'d>) -> PhyClockGuard<'d> {
        // Enable the power domain.
        unsafe {
            Self::set_module_status(true);
        }
        // Enable the modem clock.
        wifi.enable_modem_clock(true);
        // Initialize the PHY.
        let clock_guard = unsafe { Self::initialize_phy() };
        unsafe {
            Self::reset_mac();
            Self::set_mac_state(true);
        }

        clock_guard
    }

    // RX

    /// Enable or disable the receiving of frames.
    ///
    /// # Safety
    /// Enabling RX, when the DMA list isn't setup correctly yet, may be incorrect.
    unsafe fn set_rx_status(enable_rx: bool) {
        WIFI::regs()
            .rx_ctrl()
            .modify(|_, w| w.rx_enable().bit(enable_rx));
    }
}

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
