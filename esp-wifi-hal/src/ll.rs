//! Low Level functions.
//!
//! All functions in this module are unsafe, since their effect may be very context dependent.
use core::{iter::IntoIterator, ptr::NonNull};

use esp_hal::{
    clock::{ModemClockController, PhyClockGuard},
    dma::DmaDescriptor,
    peripherals::{LPWR, WIFI as HalWIFI},
};
use esp_wifi_sys::include::*;

use crate::{esp_pac::{WIFI, wifi::crypto_key_slot::KEY_VALUE}, phy_init_data::PHY_INIT_DATA_DEFAULT};

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

#[inline(always)]
/// Split a MAC address into a low [u32] and high [u16].
///
/// Let's hope the compiler optimizes this to two load-store pairs.
fn split_address(address: &[u8; 6]) -> (u32, u16) {
    let (low, high) = address.split_at(4);
    (
        u32::from_ne_bytes(low.try_into().unwrap()),
        u16::from_ne_bytes(high.try_into().unwrap()),
    )
}

/// Parameters for a crypto key slot.
///
/// NOTE: These values can contradict each other, unlike in the user facing API, since this API is
/// intended to only be used with great care.
pub struct KeySlotParameters {
    /// Address of the party, with which this key is shared.
    pub address: [u8; 6],
    /// The key ID.
    ///
    /// This can be in the range from 0-3. All other values will get truncated.
    pub key_id: u8,
    /// The interface using this key.
    pub interface: u8,

    // A key can be for both pairwise and group traffic, if WEP is in use.
    /// Is the key for pairwise traffic.
    pub pairwise: bool,
    /// Is the key for group traffic.
    pub group: bool,

    /// The cryptographic algorithm.
    ///
    /// The mapping can be seen in the following table:
    ///
    /// Algorithm | Number
    /// -- | --
    /// WEP | 1
    /// TKIP | 2
    /// CCMP | 3
    /// SMS4 | 4
    /// (GCMP) | 5
    pub algorithm: u8,
    /// If the algorithm is WEP, is the key 104 bits long.
    pub wep_104: bool,
}
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
/// The bank of the RX filter.
pub enum RxFilterBank {
    /// Basic Service Set Identifier (BSSID)
    BSSID,
    /// Receiver Address
    ReceiverAddress,
}
impl RxFilterBank {
    pub(crate) fn into_bits(self) -> usize {
        match self {
            Self::BSSID => 0,
            Self::ReceiverAddress => 1,
        }
    }
}

#[cfg(any(feature = "esp32", feature = "esp32s2"))]
/// The number of "interfaces" supported by the hardware.
pub const INTERFACE_COUNT: usize = 4;

/// The number of key slots the hardware has.
pub const KEY_SLOT_COUNT: usize = 25;

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
///
/// # Panics
/// A lot of the functions, that can panic, will do so under similar conditions, so the docs are
/// shared. When an index (i.e. key slot or interface) is provided, it is expected, that the input
/// is within the valid range. If it is not, the function will panic.
pub struct LowLevelDriver<'d> {
    /// The Wi-Fi peripheral.
    wifi: HalWIFI<'d>,
    /// Prevents the PHY clock from being disabled, while the driver is running.
    phy_clock_guard: PhyClockGuard<'d>,
}
impl<'d> LowLevelDriver<'d> {
    /// Get the PAC Wi-Fi peripheral.
    ///
    /// This stops RA from doing funky shit.
    fn regs() -> WIFI {
        unsafe { WIFI::steal() }
    }
    /// Create a new [LowLevelDriver].
    pub fn new(mut wifi: esp_hal::peripherals::WIFI<'d>) -> Self {
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
        let clock_guard = unsafe { HalWIFI::steal() }.enable_phy_clock();
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
        unsafe { HalWIFI::steal() }.reset_wifi_mac();
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
            && Self::regs().ctrl().read().bits() & MAC_READY_MASK == 0
        {}
        // If we are initializing the MAC, we need to clear some bits, by masking the reg, with the
        // MAC_INIT_MASK. On deinit, we need to set the bits we previously cleared.
        Self::regs().ctrl().modify(|r, w| unsafe {
            w.bits(if intialized {
                r.bits() & MAC_INIT_MASK
            } else {
                r.bits() | !MAC_INIT_MASK
            })
        });

        // Spin until the MAC is no longer ready.
        while !intialized && Self::regs().ctrl().read().bits() & MAC_READY_MASK != 0 {}
    }
    #[inline]
    /// Setup relevant blocks of the MAC.
    ///
    /// # Safety
    /// This should only be called during init.
    unsafe fn setup_mac() {
        unsafe extern "C" {
            fn hal_init();
        }
        // We call the proprietary blob here, to do some init for us. In the long term, this will
        // be moved to open source code. In the meantime, we do the init, that we already understand a
        // second time in open source code
        unsafe {
            hal_init();
        }

        // Open source setup code
        Self::crypto_init();
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
    unsafe fn init(wifi: &mut HalWIFI<'d>) -> PhyClockGuard<'d> {
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

    #[inline]
    /// Enable or disable the receiving of frames.
    ///
    /// # Safety
    /// Enabling RX, when the DMA list isn't setup correctly yet, may be incorrect.
    unsafe fn set_rx_status(enable_rx: bool) {
        Self::regs()
            .rx_ctrl()
            .modify(|_, w| w.rx_enable().bit(enable_rx));
    }

    #[inline]
    /// Check if RX is enabled.
    ///
    /// We don't know, if this influences the RF frontend in any way.
    pub fn rx_enabled(&self) -> bool {
        Self::regs().rx_ctrl().read().rx_enable().bit()
    }
    /// Enable or disable the specified filter.
    pub fn set_filter_enable(&self, interface: usize, filter_bank: RxFilterBank, enabled: bool) {
        Self::regs()
            .filter_bank(filter_bank.into_bits())
            .mask_high(interface)
            .modify(|_, w| w.enabled().bit(enabled));
    }
    pub fn filter_enabled(&self, interface: usize, filter_bank: RxFilterBank) -> bool {
        Self::regs()
            .filter_bank(filter_bank.into_bits())
            .mask_high(interface)
            .read()
            .enabled()
            .bit()
    }
    /// Set the filter address for the specified interface and bank combination.
    ///
    /// This will neither enable the filter nor configure the mask.
    pub fn set_filter_address(
        &self,
        interface: usize,
        filter_bank: RxFilterBank,
        address: &[u8; 6],
    ) {
        let wifi = Self::regs();
        let filter_bank = wifi.filter_bank(filter_bank.into_bits());

        let (address_low, address_high) = split_address(address);
        filter_bank
            .addr_low(interface)
            .write(|w| unsafe { w.bits(address_low) });
        filter_bank
            .addr_high(interface)
            .write(|w| unsafe { w.addr().bits(address_high) });
    }
    /// Set the filter mask for the specified interface and bank combination.
    ///
    /// This will neither enable the filter nor configure the address.
    pub fn set_filter_mask(&self, interface: usize, filter_bank: RxFilterBank, mask: &[u8; 6]) {
        let wifi = Self::regs();
        let filter_bank = wifi.filter_bank(filter_bank.into_bits());

        let (mask_low, mask_high) = split_address(mask);
        filter_bank
            .mask_low(interface)
            .write(|w| unsafe { w.bits(mask_low) });
        // We need to modfiy here, since otherwise we would be clearing the enable bit, which is
        // not part of this functions job.
        filter_bank
            .mask_high(interface)
            .modify(|_, w| unsafe { w.mask().bits(mask_high) });
    }
    /// Clear a filter bank.
    ///
    /// This zeroes out the address and mask, while also disabling the filter as a side effect.
    /// Although the inverse is not true, it makes sense to do this, since that way we can just
    /// reset the [mask_high](crate::esp_pac::wifi::filter_bank::FILTER_BANK::mask_high) field,
    /// saving us one read. This also prevents keeping the slot enabled, while zeroing it, which
    /// would be invalid regardless.
    pub fn clear_filter_bank(&self, interface: usize, filter_bank: RxFilterBank) {
        let wifi = Self::regs();
        let filter_bank = wifi.filter_bank(filter_bank.into_bits());

        filter_bank.addr_low(interface).reset();
        filter_bank.addr_high(interface).reset();
        filter_bank.mask_low(interface).reset();
        filter_bank.mask_high(interface).reset();
    }

    // Crypto

    #[inline]
    /// Enable hardware cryptography for the specified interface.
    ///
    /// NOTE: We don't yet know, how to revert this, but it's assumed, that the two bits could each
    /// stand for TX and RX.
    ///
    /// # Panics
    /// If `interface` is greater, than [INTERFACE_COUNT], this will panic.
    fn enable_crypto_for_interface(interface: usize) {
        // Writing 0x3_000 in the crypto control register of an interface seem to be used to enable
        // crypto for that interface.
        Self::regs()
            .crypto_control()
            .interface_crypto_control(interface)
            .write(|w| unsafe { w.bits(0x0003_0000) });
    }
    #[inline]
    /// Initialize hardware cryptography.
    ///
    /// This is pretty much the same as `hal_crypto_init`, except we init crypto for all
    /// interfaces, not just zero and one.
    fn crypto_init() {
        // Enable crypto for all interfaces.
        (0..INTERFACE_COUNT).for_each(Self::enable_crypto_for_interface);

        // Resetting the general crypto control register effectively marks all slots as unused.
        Self::regs()
            .crypto_control()
            .general_crypto_control()
            .reset();
    }
    #[inline]
    /// Enable or disable a key slot.
    ///
    /// As soon, as the key slot is enabled, the hardware will treat the data in it as valid.
    pub fn set_key_slot_enable(&self, key_slot: usize, enabled: bool) {
        Self::regs()
            .crypto_control()
            .crypto_key_slot_state()
            .modify(|_, w| w.key_slot_enable(key_slot as u8).bit(enabled));
    }
    /// Set the cryptographic key for the specified key slot.
    ///
    /// # Panics
    /// If `key` is longer than 32 bytes, or `key_slot` larger than 24.
    pub fn set_key(&self, key_slot: usize, key: &[u8]) {
        assert!(key.len() <= 32, "Keys can't be longer than 32 bytes.");
        let wifi = Self::regs();
        let key_slot = wifi.crypto_key_slot(key_slot);

        // Let's hope the compiler is smart enough to emit no panics this way.
        let (full_key_words, remaining_key_bytes) = key.as_chunks::<4>();

        // We first write the full key words...
        for (word, key_value) in full_key_words
            .iter()
            .copied()
            .zip(key_slot.key_value_iter())
        {
            key_value.write(|w| unsafe { w.bits(u32::from_ne_bytes(word)) });
        }
        // and then the remaining bytes.
        if !remaining_key_bytes.is_empty() {
            assert!(remaining_key_bytes.len() < 4);

            let mut temp = [0u8; 4];
            temp[..remaining_key_bytes.len()].copy_from_slice(remaining_key_bytes);
            key_slot
                .key_value(full_key_words.len())
                .write(|w| unsafe { w.bits(u32::from_ne_bytes(temp)) });
        }
    }
    /// Set the key slot parameters.
    ///
    /// This will also set the 22nd bit, of the [addr_high register](crate::esp_pac::wifi::crypto_key_slot::addr_high),
    /// the purpose of which we don't really know. In the PACs, it can be found at
    /// [unknown](crate::esp_pac::wifi::crypto_key_slot::addr_high::W::unknown).
    ///
    /// The rationale behind having a single function setting all the parameters, is that it's unlikely
    /// one of these will change, while the key slot is in use. It is however possible, that the key
    /// gets changed, so that's in a separate function.
    pub fn set_key_slot_parameters(
        &self,
        key_slot: usize,
        key_slot_parameters: &KeySlotParameters,
    ) {
        let wifi = Self::regs();
        let key_slot = wifi.crypto_key_slot(key_slot);
        let (address_low, address_high) = split_address(&key_slot_parameters.address);

        key_slot
            .addr_low()
            .write(|w| unsafe { w.bits(address_low) });
        key_slot.addr_high().modify(|_, w| unsafe {
            w.key_id()
                .bits(key_slot_parameters.key_id)
                .pairwise_key()
                .bit(key_slot_parameters.pairwise)
                .group_key()
                .bit(key_slot_parameters.group)
                .wep_104()
                .bit(key_slot_parameters.wep_104)
                .algorithm()
                .bits(key_slot_parameters.algorithm)
                .interface_id()
                .bits(key_slot_parameters.interface)
                .unknown()
                .set_bit()
                .addr()
                .bits(address_high)
        });
    }
    /// Clear all data from a key slot.
    ///
    /// This will not disable the key slot, use [Self::set_key_slot_enable] for that.
    pub fn clear_key_slot(&self, key_slot: usize) {
        let wifi = Self::regs();
        let key_slot = wifi.crypto_key_slot(key_slot);

        // Effectively zeroes the entire slot.
        key_slot.addr_low().reset();
        key_slot.addr_high().reset();
        key_slot.key_value_iter().for_each(KEY_VALUE::reset);
    }
}

/// Tell the hardware to reload the RX descriptors.
///
/// This will spin, until the bit is clear again.
pub unsafe fn reload_hw_rx_descriptors() {
    let reg = HalWIFI::regs().rx_ctrl();

    // Start the reload.
    reg.modify(|_, w| w.rx_descr_reload().set_bit());

    // Wait for the hardware descriptors to no longer be in reload.
    while reg.read().rx_descr_reload().bit() {}
}
/// Set the base descriptor.
///
/// If `base_descriptor` is [None], this will reset the register back to zero.
pub unsafe fn set_base_rx_descriptor(base_descriptor: Option<NonNull<DmaDescriptor>>) {
    let reg = HalWIFI::regs().rx_dma_list().rx_descr_base();
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
    HalWIFI::regs().rx_dma_list().rx_descr_next().read().bits() as *mut DmaDescriptor
}
/// Get the raw value of the last RX descriptor register.
///
/// # Safety
/// This could be unsafe, since we don't know what happens if we read the register, while the
/// peripheral is powered down.
pub unsafe fn last_rx_descriptor_raw() -> *mut DmaDescriptor {
    HalWIFI::regs().rx_dma_list().rx_descr_last().read().bits() as *mut DmaDescriptor
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
    HalWIFI::regs()
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
    HalWIFI::regs()
        .tx_slot_config(hw_slot_index(slot))
        .plcp0()
        .modify(|_, w| w.slot_enabled().bit(enabled));
}
/// Validate or invalidate the specified TX slot.
pub unsafe fn set_tx_slot_validity(slot: usize, valid: bool) {
    HalWIFI::regs()
        .tx_slot_config(hw_slot_index(slot))
        .plcp0()
        .modify(|_, w| w.slot_valid().bit(valid));
}

#[inline(always)]
pub unsafe fn process_tx_completions(mut cb: impl FnMut(usize)) {
    let wifi = HalWIFI::regs();
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
    let wifi = HalWIFI::regs();
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
    let wifi = HalWIFI::regs();
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
