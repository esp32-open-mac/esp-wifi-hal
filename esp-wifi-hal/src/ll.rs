//! Low Level functions.
//!
//! All functions in this module are unsafe, since their effect may be very context dependent.
use core::{
    iter::IntoIterator,
    pin::Pin,
    ptr::{with_exposed_provenance_mut, NonNull},
};

use esp_hal::{
    clock::ModemClockController,
    dma::DmaDescriptor,
    interrupt::{self, CpuInterrupt, InterruptHandler},
    peripherals::{LPWR, WIFI as HalWIFI},
};
use esp_phy::{PhyController, PhyInitGuard};
use macro_bits::{bit, check_bit};
use portable_atomic::AtomicU64;

use crate::{
    esp_pac::{wifi::crypto_key_slot::KEY_VALUE, Interrupt as PacInterrupt, WIFI},
    ffi::{disable_wifi_agc, enable_wifi_agc, hal_init, tx_pwctrl_background}, WiFiRate,
};

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
    Bssid,
    /// Receiver Address
    ReceiverAddress,
}
impl RxFilterBank {
    pub(crate) fn into_bits(self) -> usize {
        match self {
            Self::Bssid => 0,
            Self::ReceiverAddress => 1,
        }
    }
}
macro_rules! cause_bitmask {
    ([$($(@chip($chips:tt))? $mask:expr),*]) => {
        const {
            let mut temp = 0;
            $(
                #[allow(unused)]
                let mut condition = true;
                $(
                    condition = cfg!(any($chips));
                )?
                temp |= if condition { $mask } else { 0 };
            )*
            temp
        }
    };
    ($mask:expr) => {
        $mask
    };
}
/// Create a struct to access the cause of an interrupt.
///
/// All declared accessor functions are marked as `inline`, since they are basically just a mask
/// and a compare.
macro_rules! interrupt_cause_struct {
    ($(#[$struct_meta:meta])* $struct_name:ident => {
        $(
            $(#[$function_meta:meta])*
            $function_name:ident => $mask:tt
        ),*
    }) => {
        $(#[$struct_meta])*
        #[cfg_attr(feature = "defmt", defmt::Format)]
        #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
        #[repr(C)]
        pub struct $struct_name(u32);
        impl $struct_name {
            #[inline]
            /// Get the raw interrupt cause.
            pub const fn raw_cause(&self) -> u32 {
                self.0
            }
            #[inline]
            /// Is there no known cause for the interrupt.
            pub fn is_emtpy(&self) -> bool {
                self.0 == 0
            }
            $(
                #[inline]
                $(#[$function_meta])*
                pub fn $function_name(&self) -> bool {
                    (self.0 & cause_bitmask!($mask)) != 0
                }
            )*
        }
    };
}
interrupt_cause_struct! {
    /// Cause for the MAC interrupt.
    MacInterruptCause => {
        /// A frame was received.
        ///
        /// We don't know, what the individual bits mean, but this works.
        rx => [0x100020, @chip(esp32) 0x4],
        /// A frame was transmitted successfully.
        tx_success => 0x80,
        /// A transmission timeout occured.
        ///
        /// We think this might have something to do with waiting for medium access.
        tx_timeout => 0x80000,
        /// A transmission collision occured.
        ///
        /// This might be caused by multiple transmissions with different ACs getting a TXOP at the
        /// same time.
        tx_collision => 0x100
    }
}
#[cfg(pwr_interrupt_present)]
interrupt_cause_struct! {
    /// Cause for the power interrupt.
    PwrInterruptCause => {
        /// A TBTT was reached or is about to be reached.
        ///
        /// NOTE: This is an unconfirmed assumption.
        tbtt => [@chip(esp32s2) 0x1e],
        /// We don't know them meaning of this yet.
        tsf_timer => [@chip(esp32s2) 0x1e0]
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// The different interrupts of the Wi-Fi peripheral.
pub enum WiFiInterrupt {
    /// Medium Access Control interrupt
    Mac,
    /// Baseband interrupt
    ///
    /// NOTE: We know literally nothing about this interrupt, as it doesn't appear to be used at
    /// all. This is only included for completeness.
    Bb,
    #[cfg(pwr_interrupt_present)]
    /// Low power interrupt
    Pwr,
    // With chips supporting Wi-Fi 6, there will be a BSS Color interrupt here.
}
impl From<WiFiInterrupt> for PacInterrupt {
    fn from(value: WiFiInterrupt) -> Self {
        match value {
            WiFiInterrupt::Mac => PacInterrupt::WIFI_MAC,
            WiFiInterrupt::Bb => PacInterrupt::WIFI_BB,
            #[cfg(pwr_interrupt_present)]
            WiFiInterrupt::Pwr => PacInterrupt::WIFI_PWR,
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// Result of the transmission.
///
/// This is separate from the MAC TX result, which accounts for ACK timeouts etc.
pub enum HardwareTxResult {
    /// TX completed successfully.
    Success,
    /// A timeout occured.
    ///
    /// This may be related to the timeout parameter.
    Timeout,
    /// A collision occured.
    ///
    /// This might be caused by internal TX queue scheduling conflicts.
    Collision,
}
impl HardwareTxResult {
    /// Get raw transmission status bits.
    fn raw_status(&self) -> u8 {
        let wifi = LowLevelDriver::regs_internal();
        (match self {
            HardwareTxResult::Success => wifi.txq_state().tx_complete_status().read().bits(),
            HardwareTxResult::Timeout => wifi.txq_state().tx_error_status().read().bits() >> 0x10,
            HardwareTxResult::Collision => wifi.txq_state().tx_error_status().read().bits(),
        }) as u8
    }
    /// Clear slot transmission status bit.
    fn clear_slot_bit(&self, slot: usize) {
        let wifi = LowLevelDriver::regs_internal();
        match self {
            HardwareTxResult::Success => wifi
                .txq_state()
                .tx_complete_clear()
                .modify(|_, w| w.slot(slot as u8).set_bit()),
            HardwareTxResult::Timeout => wifi
                .txq_state()
                .tx_error_clear()
                .modify(|_, w| w.slot_timeout(slot as u8).set_bit()),
            HardwareTxResult::Collision => wifi
                .txq_state()
                .tx_error_clear()
                .modify(|_, w| w.slot_collision(slot as u8).set_bit()),
        };
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// Status of the TX slot.
pub enum TxSlotStatus {
    /// The slot is currently disabled and the data in it may be invalid.
    Disabled,
    /// The data in the slot is valid, but the slot is not marked as ready yet.
    Valid,
    /// The data in the slot is valid and it is ready for transmission.
    Ready,
}
impl TxSlotStatus {
    /// Is the data in the slot valid.
    fn valid(&self) -> bool {
        *self != Self::Disabled
    }
    /// Is the slot enabled for transmission.
    fn enabled(&self) -> bool {
        *self == Self::Ready
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// Controls which control frames pass the filter.
pub struct ControlFrameFilterConfig {
    /// Control frame wrapper
    pub control_wrapper: bool,
    /// Block ACK request (BAR)
    pub block_ack_request: bool,
    /// Block ACK (BA)
    pub block_ack: bool,
    /// PS-Poll
    pub ps_poll: bool,
    /// Request-to-send (RTS)
    pub rts: bool,
    /// Clear-to-send (CTS)
    pub cts: bool,
    /// ACK
    pub ack: bool,
    /// Control frame end
    pub cf_end: bool,
    /// Control frame end and ACK
    pub cf_end_cf_ack: bool,
}
impl ControlFrameFilterConfig {
    /// Disable reception of all control frames.
    pub fn none() -> Self {
        Self::default()
    }
    /// Receive all control frames.
    pub fn all() -> Self {
        Self {
            control_wrapper: true,
            block_ack_request: true,
            block_ack: true,
            ps_poll: true,
            rts: true,
            cts: true,
            ack: true,
            cf_end: true,
            cf_end_cf_ack: true,
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// The EDCA access category.
///
/// It maps the EDCA access category to the hardware TX slot number.
pub enum EdcaAccessCategory {
    /// BA-AC
    Background = 1,
    #[default]
    /// BE-AC
    BestEffort = 2,
    /// VI-AC
    Video = 3,
    /// VO-AC
    Voice = 4,
}
impl EdcaAccessCategory {
    #[inline]
    /// Get the IEEE 802.11 Access Category Index (ACI).
    pub const fn access_category_index(&self) -> usize {
        match *self {
            Self::Background => 1,
            Self::BestEffort => 0,
            Self::Video => 2,
            Self::Voice => 3,
        }
    }
    #[inline]
    /// Convert the Access Category Index (ACI) into our AC type.
    pub const fn from_access_category_index(access_category_index: usize) -> Option<Self> {
        Some(match access_category_index {
            1 => Self::Background,
            0 => Self::BestEffort,
            2 => Self::Video,
            3 => Self::Voice,
            _ => return None,
        })
    }
    #[inline]
    /// Get the hardware slot number.
    ///
    /// NOTE: This slot numbering does not line up, with the one used by Espressif, as we index the
    /// slots in ascending address order (i.e. slot 0 registers have the lowest addresses), not
    /// descending order (i.e. slot 0 register have the highest addresses), which is used in the
    /// proprietary stack. This makes no difference in practice, other than being subjectively less
    /// confusing.
    pub const fn hardware_slot(&self) -> usize {
        *self as usize
    }
    #[inline]
    /// Get the EDCA queue from the slot number.
    ///
    /// NOTE: This slot numbering does not line up, with the one used by Espressif, as we index the
    /// slots in ascending address order (i.e. slot 0 registers have the lowest addresses), not
    /// descending order (i.e. slot 0 register have the highest addresses), which is used in the
    /// proprietary stack. This makes no difference in practice, other than being subjectively less
    /// confusing.
    pub const fn from_hardware_slot(slot: usize) -> Option<Self> {
        Some(match slot {
            1 => Self::Background,
            2 => Self::BestEffort,
            3 => Self::Video,
            4 => Self::Voice,
            _ => return None,
        })
    }
}
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// The hardware TX queues.
pub enum HardwareTxQueue {
    #[default]
    /// Distributed Channel Access Function
    ///
    /// This is the default channel access function.
    Dcf,
    /// Enhanced Distributed Channel Access Function
    ///
    /// This is the modified channel access function used for QoS traffic.
    Edcaf(EdcaAccessCategory),
}
impl HardwareTxQueue {
    #[inline]
    /// Get the hardware slot number.
    ///
    /// NOTE: This slot numbering does not line up, with the one used by Espressif, as we index the
    /// slots in ascending address order (i.e. slot 0 registers have the lowest addresses), not
    /// descending order (i.e. slot 0 register have the highest addresses), which is used in the
    /// proprietary stack. This makes no difference in practice, other than being subjectively less
    /// confusing.
    pub const fn hardware_slot(&self) -> usize {
        match *self {
            Self::Dcf => 4,
            Self::Edcaf(access_category) => access_category.hardware_slot(),
        }
    }
    #[inline]
    /// Get the queue from the slot number.
    ///
    /// NOTE: This slot numbering does not line up, with the one used by Espressif, as we index the
    /// slots in ascending address order (i.e. slot 0 registers have the lowest addresses), not
    /// descending order (i.e. slot 0 register have the highest addresses), which is used in the
    /// proprietary stack. This makes no difference in practice, other than being subjectively less
    /// confusing.
    pub const fn from_hardware_slot(slot: usize) -> Option<Self> {
        if slot == 0 {
            Some(Self::Dcf)
        } else if let Some(edca_ac) = EdcaAccessCategory::from_hardware_slot(slot) {
            Some(Self::Edcaf(edca_ac))
        } else {
            None
        }
    }
    #[inline]
    /// Is the queue an EDCA queue.
    pub const fn is_edca(&self) -> bool {
        matches!(self, Self::Edcaf(_))
    }
}

#[cfg(any(feature = "esp32", feature = "esp32s2"))]
/// The number of "interfaces" supported by the hardware.
pub const INTERFACE_COUNT: usize = 4;

/// The number of key slots the hardware has.
pub const KEY_SLOT_COUNT: usize = 25;

static MAC_TIME_OFFSET: AtomicU64 = AtomicU64::new(0);

/// Low level driver for the Wi-Fi peripheral.
///
/// This is intended as an intermediary layer between the user facing API and the hardware, to make
/// driver maintenance easier.
///
/// # Implementation specifics
/// ## TX
/// ### Slot Numbering
/// The hardware has 5 TX slots
/// The slot numbering used for TX is aligned with that of the proprietary stack, not with the
/// hardware. This means, that our slot numbering is reversed compared to the hardware slot
/// registers, as this reduces confusion when reverse engineering. Our slot 0 would therefore be
/// the highest slot in the hardware (i.e. slot 4).
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
pub struct LowLevelDriver {
    /// Prevents the PHY from being disabled, while the driver is running.
    _phy_init_guard: Option<PhyInitGuard<'static>>,
}
impl LowLevelDriver {
    /// Get the PAC Wi-Fi peripheral.
    ///
    /// # Safety
    /// This is only intended to be used for debugging and development purposes. The ways you can
    /// shoot yourself in the foot, by accessing the peripheral directly, are way too many to list.
    /// Using this should be a last resort and only be done, if you know what you're doing.
    pub unsafe fn regs() -> WIFI {
        Self::regs_internal()
    }
    /// Get the PAC Wi-Fi peripheral.
    ///
    /// This stops rust-analyzer from doing funky shit.
    fn regs_internal() -> WIFI {
        unsafe { WIFI::steal() }
    }
    /// Create a new [LowLevelDriver].
    pub fn new(_wifi: esp_hal::peripherals::WIFI<'_>) -> Self {
        // The reason we first create the struct an then initialize it, is so that we can access
        // methods during init.
        unsafe {
            Self {
                _phy_init_guard: None,
            }
            .init()
        }
    }

    // Initialization

    /// Set the power and reset status of the Wi-Fi peripheral module.
    ///
    /// # Safety
    /// Enabling or disabling the Wi-Fi PD, outside of init or deinit, breaks a lot of assumptions
    /// other code makes, so take great care.
    unsafe fn set_module_status(&self, enable_module: bool) {
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
    /// Perform a full MAC reset.
    ///
    /// # Safety
    /// This will basically reset all state of the MAC to its defaults. Therefore it may also break
    /// assumptions made by some pieces of code.
    unsafe fn reset_mac(&self) {
        // Perform a full reset of the Wi-Fi module.
        unsafe { HalWIFI::steal() }.reset_wifi_mac();
        // NOTE: coex_bt_high_prio would usually be here

        // Disable RX
        unsafe {
            Self::set_rx_enable(false);
        }
    }
    /// Initialize or deinitialize the MAC.
    ///
    /// If `intialized` is `true`, the MAC will be initialized, and vice versa.
    /// # Safety
    /// We don't really know, what exactly this does, so we only call it, where the closed source
    /// driver would.
    unsafe fn set_mac_state(&self, intialized: bool) {
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
            && Self::regs_internal().ctrl().read().bits() & MAC_READY_MASK == 0
        {}
        // If we are initializing the MAC, we need to clear some bits, by masking the reg, with the
        // MAC_INIT_MASK. On deinit, we need to set the bits we previously cleared.
        Self::regs_internal().ctrl().modify(|r, w| unsafe {
            w.bits(if intialized {
                r.bits() & MAC_INIT_MASK
            } else {
                r.bits() | !MAC_INIT_MASK
            })
        });

        // Spin until the MAC is no longer ready.
        while !intialized && Self::regs_internal().ctrl().read().bits() & MAC_READY_MASK != 0 {}
    }
    #[inline]
    /// Setup relevant blocks of the MAC.
    ///
    /// # Safety
    /// This should only be called during init.
    unsafe fn setup_mac(&self) {
        // We call the proprietary blob here, to do some init for us. In the long term, this will
        // be moved to open source code. In the meantime, we do the init, that we already understand a
        // second time in open source code, which should have no bad effects.
        unsafe {
            hal_init();
        }

        // Open source setup code
        self.setup_filtering();
        self.setup_crypto();
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
    unsafe fn init(mut self) -> Self {
        // Enable the power domain.
        unsafe {
            self.set_module_status(true);
        }
        let mut hal_wifi = unsafe { HalWIFI::steal() };
        #[cfg(esp32)]
        {
            <HalWIFI<'_> as esp_phy::MacTimeExt>::set_mac_time_update_cb(&hal_wifi, |duration| {
                let _ = MAC_TIME_OFFSET
                    .fetch_add(duration.as_micros(), core::sync::atomic::Ordering::Relaxed);
            });
        }
        // Enable the modem clock.
        hal_wifi.enable_modem_clock(true);
        // Initialize the PHY.
        self._phy_init_guard = Some(hal_wifi.enable_phy());
        unsafe {
            self.reset_mac();
            self.set_mac_state(true);
            self.setup_mac();
        }
        self
    }

    // Interrupt handling

    #[inline(always)]
    /// Get and clear the MAC interrupt cause.
    ///
    /// # Safety
    /// This is expected to be called ONLY from the MAC interrupt handler.
    pub unsafe fn get_and_clear_mac_interrupt_cause() -> MacInterruptCause {
        let cause = Self::regs_internal()
            .mac_interrupt()
            .wifi_int_status()
            .read()
            .bits();
        Self::regs_internal()
            .mac_interrupt()
            .wifi_int_clear()
            .write(|w| unsafe { w.bits(cause) });
        MacInterruptCause(cause)
    }
    #[cfg(pwr_interrupt_present)]
    #[inline(always)]
    /// Get and clear the PWR interrupt cause.
    ///
    /// # Safety
    /// This is expected to be called ONLY from the MAC interrupt handler.
    pub unsafe fn get_and_clear_pwr_interrupt_cause() -> PwrInterruptCause {
        let cause = Self::regs().pwr_interrupt().pwr_int_status().read().bits();
        Self::regs()
            .pwr_interrupt()
            .pwr_int_clear()
            .write(|w| unsafe { w.bits(cause) });
        PwrInterruptCause(cause)
    }
    /// Configure the specified interrupt.
    ///
    /// Usually all relevant interrupts are bound to the same interrupt handler and CPU interrupt.
    /// The default CPU interrupt for Wi-Fi is [CpuInterrupt::Interrupt0LevelPriority1].
    pub fn configure_interrupt(
        &self,
        interrupt: WiFiInterrupt,
        cpu_interrupt: CpuInterrupt,
        interrupt_handler: InterruptHandler,
    ) {
        unsafe {
            let interrupt = interrupt.into();
            interrupt::bind_interrupt(interrupt, interrupt_handler.handler());
            let _ = interrupt::enable_direct(interrupt, cpu_interrupt);
        }
    }

    // RX

    #[inline]
    /// Enable or disable the receiving of frames.
    ///
    /// # Safety
    /// Enabling RX, when the DMA list isn't setup correctly yet, may be incorrect.
    unsafe fn set_rx_enable(enable_rx: bool) {
        Self::regs_internal()
            .rx_ctrl()
            .modify(|_, w| w.rx_enable().bit(enable_rx));
    }
    #[inline]
    /// Check if RX is enabled.
    ///
    /// We don't know, if this influences the RF frontend in any way.
    pub fn rx_enabled(&self) -> bool {
        Self::regs_internal().rx_ctrl().read().rx_enable().bit()
    }
    /// Tell the hardware to reload the RX descriptors.
    ///
    /// This will spin, until the bit is clear again.
    pub fn reload_hw_rx_descriptors(&self) {
        let wifi = Self::regs_internal();
        let reg = wifi.rx_ctrl();

        // Start the reload.
        reg.modify(|_, w| w.rx_descr_reload().set_bit());

        // Wait for the hardware descriptors to no longer be in reload.
        while reg.read().rx_descr_reload().bit() {}
    }
    /// Set the base descriptor.
    pub fn set_base_rx_descriptor(&self, base_descriptor: NonNull<DmaDescriptor>) {
        Self::regs_internal()
            .rx_dma_list()
            .rx_descr_base()
            .write(|w| unsafe { w.bits(base_descriptor.expose_provenance().get() as u32) });
        self.reload_hw_rx_descriptors();
    }
    /// Reset the base descriptor.
    pub fn clear_base_rx_descriptor(&self) {
        Self::regs_internal().rx_dma_list().rx_descr_base().reset();
        self.reload_hw_rx_descriptors();
    }
    /// Start receiving frames.
    ///
    /// This will set the provided descriptor as the base of the RX DMA list and enable RX.
    pub fn start_rx(&self, base_descriptor: NonNull<DmaDescriptor>) {
        self.set_base_rx_descriptor(base_descriptor);
        unsafe {
            Self::set_rx_enable(true);
        }
    }
    /// Stop receiving frames.
    ///
    /// This will also clear the RX DMA list.
    pub fn stop_rx(&self) {
        unsafe {
            Self::set_rx_enable(false);
        }
        self.clear_base_rx_descriptor();
    }
    /// Get the base RX descriptor.
    pub fn base_rx_descriptor(&self) -> Option<NonNull<DmaDescriptor>> {
        NonNull::new(with_exposed_provenance_mut(
            Self::regs_internal()
                .rx_dma_list()
                .rx_descr_base()
                .read()
                .bits() as usize,
        ))
    }
    /// Get the next RX descriptor.
    pub fn next_rx_descriptor(&self) -> Option<NonNull<DmaDescriptor>> {
        NonNull::new(with_exposed_provenance_mut(
            Self::regs_internal()
                .rx_dma_list()
                .rx_descr_next()
                .read()
                .bits() as usize,
        ))
    }
    /// Get the last RX descriptor.
    pub fn last_rx_descriptor(&self) -> Option<NonNull<DmaDescriptor>> {
        NonNull::new(with_exposed_provenance_mut(
            Self::regs_internal()
                .rx_dma_list()
                .rx_descr_last()
                .read()
                .bits() as usize,
        ))
    }

    // RX filtering

    /// Enable or disable the specified filter.
    pub fn set_filter_enable(&self, interface: usize, filter_bank: RxFilterBank, enabled: bool) {
        Self::regs_internal()
            .filter_bank(filter_bank.into_bits())
            .mask_high(interface)
            .modify(|_, w| w.enabled().bit(enabled));
    }
    /// Check if the filter is enabled.
    pub fn filter_enabled(&self, interface: usize, filter_bank: RxFilterBank) -> bool {
        Self::regs_internal()
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
        let wifi = Self::regs_internal();
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
        let wifi = Self::regs_internal();
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
    ///
    /// This will also enable the BSSID check, to bring the filter into a sort of ground state,
    /// so that we can make assumptions about its behaviour when configuring other aspects.
    pub fn clear_filter(&self, interface: usize, filter_bank: RxFilterBank) {
        let wifi = Self::regs_internal();
        let filter_bank = wifi.filter_bank(filter_bank.into_bits());

        filter_bank.addr_low(interface).reset();
        filter_bank.addr_high(interface).reset();
        filter_bank.mask_low(interface).reset();
        // As a side effect, this also disables the filter.
        filter_bank.mask_high(interface).reset();
    }
    /// Reset the filters for an interface.
    ///
    /// This will clear the RX and BSSID filters, as well as enabling the BSSID check and enabling
    /// filtering for unicast and multicast frames.
    pub fn reset_interface_filters(&self, interface: usize) {
        self.clear_filter(interface, RxFilterBank::ReceiverAddress);
        self.clear_filter(interface, RxFilterBank::Bssid);

        self.set_bssid_check_enable(interface, true);
        self.set_filtered_address_types(interface, true, true);
    }
    /// Set the type of addresses that should be filtered.
    ///
    /// If an address type is unfiltered, frames with RAs matching that address type will pass the
    /// filter unconditionally. Otherwise the frame will be filtered as usual.
    pub fn set_filtered_address_types(&self, interface: usize, unicast: bool, multicast: bool) {
        Self::regs_internal()
            .filter_control(interface)
            .modify(|_, w| {
                w.block_unicast()
                    .bit(unicast)
                    .block_multicast()
                    .bit(multicast)
            });
    }
    /// Configure if the BSSID should be checked for RX filtering.
    ///
    /// If the BSSID check is enabled, a frame will only pass the RX filter, if its BSSID matches
    /// the one in the filter. By default this is enabled.
    pub fn set_bssid_check_enable(&self, interface: usize, enabled: bool) {
        Self::regs_internal()
            .filter_control(interface)
            .modify(|_, w| w.bssid_check().bit(enabled));
    }
    /// Configure which control frames pass the filter.
    pub fn set_control_frame_filter(&self, interface: usize, config: &ControlFrameFilterConfig) {
        Self::regs_internal()
            .rx_ctrl_filter(interface)
            .modify(|_, w| {
                w.control_wrapper()
                    .bit(config.control_wrapper)
                    .block_ack_request()
                    .bit(config.block_ack_request)
                    .block_ack()
                    .bit(config.block_ack)
                    .ps_poll()
                    .bit(config.ps_poll)
                    .rts()
                    .bit(config.rts)
                    .cts()
                    .bit(config.cts)
                    .ack()
                    .bit(config.ack)
                    .cf_end()
                    .bit(config.cf_end)
                    .cf_end_cf_ack()
                    .bit(config.cf_end_cf_ack)
            });
    }
    /// Set the parameters for scanning mode.
    ///
    /// We don't entirely know what these parmeters mean.
    pub fn set_scanning_mode_parameters(&self, interface: usize, beacons: bool, other_frames: bool) {
        Self::regs_internal().filter_control(interface).modify(|_, w| w.scan_mode().bit(beacons).data_and_mgmt_mode().bit(other_frames));
    }
    #[inline]
    /// Setup RX filtering.
    ///
    /// This will bring the filters to a known-good state, by enabling filtering for unicast and
    /// multicast frames, filtering all control frames and enabling the BSSID check.
    fn setup_filtering(&self) {
        (0..4).for_each(|interface| {
            self.set_filtered_address_types(interface, true, true);
            self.set_control_frame_filter(interface, &ControlFrameFilterConfig::none());
            self.set_bssid_check_enable(interface, true);
        });
    }

    // TX

    #[inline]
    /// Process a TX status.
    ///
    /// The provided closure will be called for each slot, with the [HardwareTxQueue] as a
    /// parameter.
    ///
    /// # Safety
    /// This has to be static, so that it can be called in an ISR, but it is also expected to only
    /// be called from there, since the Wi-Fi peripheral will have to be initialized there.
    pub unsafe fn process_tx_status(tx_status: HardwareTxResult, f: impl Fn(HardwareTxQueue)) {
        let raw_status = tx_status.raw_status();
        (0..5)
            .filter(|i| check_bit!(raw_status, bit!(i)))
            .for_each(|queue| {
                // For some reason the slot numbering needs to be reversed when working with the
                // interrupt cause register.
                (f)(HardwareTxQueue::from_hardware_slot(4 - queue).unwrap());
                tx_status.clear_slot_bit(queue);
            });
    }
    #[inline]
    /// Configure the status of a TX slot.
    ///
    /// # Safety
    /// This is only expected to be called in the MAC ISR handler. For other purposes use
    /// [LowLevelDriver::start_tx_for_slot]
    pub unsafe fn set_tx_queue_status(queue: HardwareTxQueue, tx_slot_status: TxSlotStatus) {
        Self::regs_internal()
            .tx_slot_config(queue.hardware_slot())
            .plcp0()
            .modify(|_, w| {
                w.slot_valid()
                    .bit(tx_slot_status.valid())
                    .slot_enabled()
                    .bit(tx_slot_status.enabled())
            });
    }
    #[inline]
    /// Start transmission for a queue.
    ///
    /// TODO: Investigate what happens, if the DMA descriptor pointer ist zero.
    pub fn start_tx_queue(&self, queue: HardwareTxQueue) {
        unsafe {
            Self::set_tx_queue_status(queue, TxSlotStatus::Ready);
        }
    }
    #[inline]
    /// Mark the transmission on a queue as done.
    pub fn tx_done(&self, queue: HardwareTxQueue) {
        unsafe {
            Self::set_tx_queue_status(queue, TxSlotStatus::Disabled);
        }
        Self::regs_internal()
            .tx_slot_config(queue.hardware_slot())
            .plcp0()
            .reset();
    }
    #[inline]
    /// Set parameters for channel access.
    ///
    /// We aren't really sure about any of these.
    pub fn set_channel_access_parameters(
        &self,
        queue: HardwareTxQueue,
        timeout: usize,
        backoff_slots: usize,
        aifsn: usize,
    ) {
        Self::regs_internal()
            .tx_slot_config(queue.hardware_slot())
            .config()
            .modify(|_, w| unsafe {
                w.timeout()
                    .bits(timeout as _)
                    .backoff_time()
                    .bits(backoff_slots as _)
                    .aifsn()
                    .bits(aifsn as _)
            });
    }
    #[inline]
    /// Configure the PLCP0 register for a TX queue.
    pub fn set_plcp0(
        &self,
        queue: HardwareTxQueue,
        dma_list_item: Pin<&DmaDescriptor>,
        wait_for_ack: bool,
    ) {
        Self::regs_internal()
            .tx_slot_config(queue.hardware_slot())
            .plcp0()
            .modify(|_, w| unsafe {
                w.dma_addr()
                    .bits(
                        (dma_list_item.get_ref() as *const DmaDescriptor).expose_provenance()
                            as u32,
                    )
                    .wait_for_ack()
                    .bit(wait_for_ack)
            });
    }
    #[inline]
    /// Configure the PLCP1 register for a TX queue.
    ///
    /// We are not 100% sure, that our assumption about the bandwidth config is correct.
    pub fn set_plcp1(
        &self,
        queue: HardwareTxQueue,
        rate: WiFiRate,
        frame_length: usize,
        interface: usize,
        key_slot: Option<u8>,
    ) {
        Self::regs_internal()
            .plcp1(queue.hardware_slot())
            .write(|w| unsafe {
                w.len()
                    .bits(frame_length as _)
                    .rate()
                    .bits(rate as _)
                    .is_80211_n()
                    .bit(rate.is_ht())
                    .interface_id()
                    .bits(interface as _)
                    // NOTE: This makes the default value zero, which is a valid key slot ID. However
                    // unless you were to TX a frame, that was perfectly crafted to match the key ID,
                    // address and interface of key slot zero, and have a CCMP header. I guess this
                    // isn't an issue then.
                    .key_slot_id()
                    .bits(key_slot.unwrap_or_default() as _)
                    // Maybe my guess about bandwidth was wrong in the end.
                    .bandwidth()
                    .set_bit()
            });
    }
    #[inline]
    /// Configure the PLCP2 register for a TX queue.
    ///
    /// Currently this doesn't do much, except setting a bit with unknown meaning to one.
    pub fn set_plcp2(&self, queue: HardwareTxQueue) {
        Self::regs_internal()
            .plcp2(queue.hardware_slot())
            .write(|w| w.unknown().set_bit());
    }
    #[inline]
    /// Set the duration for a TX queue.
    pub fn set_duration(&self, queue: HardwareTxQueue, duration: u16) {
        let duration = duration as u32;
        Self::regs_internal()
            .duration(queue.hardware_slot())
            .write(|w| unsafe { w.bits(duration | (duration << 16)) });
    }
    #[inline]
    /// Configure the HT related register for a TX queue.
    pub fn set_ht_parameters(
        &self,
        queue: HardwareTxQueue,
        mcs: u8,
        is_short_gi: bool,
        frame_length: usize,
    ) {
        Self::regs_internal()
            .ht_sig(queue.hardware_slot())
            .write(|w| unsafe {
                w.bits(
                    (mcs as u32 & 0b111)
                        | ((frame_length as u32 & 0xffff) << 8)
                        | (0b111 << 24)
                        | ((is_short_gi as u32) << 31),
                )
            });
        Self::regs_internal()
            .ht_unknown(queue.hardware_slot())
            .write(|w| unsafe { w.bits(frame_length as u32 | 0x50000) });
    }

    // Crypto

    #[inline]
    /// Initialize hardware cryptography.
    ///
    /// This is pretty much the same as `hal_crypto_init`, except we init crypto for all
    /// interfaces, not just zero and one.
    fn setup_crypto(&self) {
        // Enable crypto for all interfaces.
        (0..INTERFACE_COUNT).for_each(|interface| {
            // Writing 0x3_000 in the crypto control register of an interface seem to be used to enable
            // crypto for that interface.
            Self::regs_internal()
                .crypto_control()
                .interface_crypto_control(interface)
                .write(|w| unsafe { w.bits(0x0003_0000) });
        });

        // Resetting the general crypto control register effectively marks all slots as unused.
        Self::regs_internal()
            .crypto_control()
            .general_crypto_control()
            .reset();
    }
    /// Enable or disable a key slot.
    ///
    /// As soon, as the key slot is enabled, the hardware will treat the data in it as valid.
    pub fn set_key_slot_enable(&self, key_slot: usize, enabled: bool) {
        Self::regs_internal()
            .crypto_control()
            .crypto_key_slot_state()
            .modify(|_, w| w.key_slot_enable(key_slot as u8).bit(enabled));
    }
    /// Check if the key slot is enabled.
    pub fn key_slot_enabled(&self, key_slot: usize) -> bool {
        Self::regs_internal()
            .crypto_control()
            .crypto_key_slot_state()
            .read()
            .key_slot_enable(key_slot as u8)
            .bit()
    }
    /// Set the cryptographic key for the specified key slot.
    ///
    /// # Panics
    /// If `key` is longer than 32 bytes, or `key_slot` larger than 24.
    pub fn set_key(&self, key_slot: usize, key: &[u8]) {
        assert!(key.len() <= 32, "Keys can't be longer than 32 bytes.");
        let wifi = Self::regs_internal();
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
        let wifi = Self::regs_internal();
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
        let wifi = Self::regs_internal();
        let key_slot = wifi.crypto_key_slot(key_slot);

        // Effectively zeroes the entire slot.
        key_slot.addr_low().reset();
        key_slot.addr_high().reset();
        key_slot.key_value_iter().for_each(KEY_VALUE::reset);
    }
    /// Set the crypto parameters for an interface.
    ///
    /// Management Frame Protection (MFP) and Signaling and Payload Protection (SPP) are configured
    /// globally for every interface, as well as the use of an Authenticated Encryption with
    /// Associated Data (AEAD) system. It is not possible to use non-AEAD ciphers (i.e. WEP) in
    /// parallel with AEAD ciphers (all others) on the same interface.
    ///
    /// NOTE: SMS4 (WAPI) is not currently supported, as information about it is scarce and it's
    /// not really used in practice.
    pub fn set_interface_crypto_parameters(
        &self,
        interface: usize,
        protect_management_frames: bool,
        protect_signaling_and_payload: bool,
        is_cipher_aead: bool,
    ) {
        Self::regs_internal()
            .crypto_control()
            .interface_crypto_control(interface)
            .modify(|r, w| unsafe {
                w.bits(r.bits() | 0x10103)
                    .spp_enable()
                    .bit(protect_signaling_and_payload)
                    .pmf_disable()
                    .bit(!protect_management_frames)
                    .aead_cipher()
                    .bit(is_cipher_aead)
                    .sms4()
                    .clear_bit()
            });
    }
    /// Set the status of SMS4 (WAPI) encapsulation.
    ///
    /// SMS4 needs to be enabled here, if you want to use it elsewhere. Our current understanding
    /// is, that WAPI can't be used concurrently with other ciphers.
    pub fn set_sms4_status(&self, enabled: bool) {
        Self::regs_internal()
            .crypto_control()
            .general_crypto_control()
            .modify(|r, w| unsafe {
                w.bits(if enabled {
                    r.bits() | 0x00ffff00
                } else {
                    r.bits() & 0xff0000ff
                })
            });
    }

    // Timing

    /// Get the offset between the system timers and the MAC timer.
    pub fn mac_time_offset() -> esp_hal::time::Duration {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                let offset = MAC_TIME_OFFSET.load(core::sync::atomic::Ordering::Relaxed);
            } else {
                let offset = 0;
            }
        }
        esp_hal::time::Duration::from_micros(offset)
    }
    /// Get the current value of the MAC timer.
    pub fn mac_time(&self) -> esp_hal::time::Duration {
        esp_hal::time::Duration::from_micros(Self::regs_internal().mac_time().read().bits() as u64)
            + Self::mac_time_offset()
    }

    // RF

    /// Set the radio channel.
    ///
    /// The channel number is not validated.
    pub fn set_channel(&self, channel_number: u8) {
        unsafe {
            self.set_mac_state(false);
            #[cfg(nomac_channel_set)]
            crate::ffi::chip_v7_set_chan_nomac(channel_number, 0);
            #[cfg(not(nomac_channel_set))]
            crate::ffi::chip_v7_set_chan(channel_number, 0);
            disable_wifi_agc();
            self.set_mac_state(true);
            enable_wifi_agc();
        }
    }
    /// Run TX power control.
    ///
    /// We don't really know, what this does.
    pub fn run_power_control(&self) {
        unsafe {
            tx_pwctrl_background(1, 0);
        }
    }
}
