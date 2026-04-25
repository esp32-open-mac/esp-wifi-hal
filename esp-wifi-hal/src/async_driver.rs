use core::{cell::RefCell, future::Future, pin::Pin};
use portable_atomic::{AtomicU8, AtomicU16, Ordering};

use crate::{
    async_driver::private::{AsyncTransmitExt, HasDmaList, NonExhaustive},
    borrowed_buffer::BorrowedBuffer,
    crypto::CipherParameters,
    dma_list::DmaBufferSlab,
    ll::{
        ChannelAccessError, ControlFrameFilterConfig, HardwareTxQueue, HardwareTxQueueStatus,
        INTERFACE_COUNT, KEY_SLOT_COUNT, KeySlotParameters, LowLevelDriver, MacProtocolError,
        RxFilterBank, WiFiInterrupt,
    },
    rates::TxPhyRate,
};
use embassy_sync::blocking_mutex;
use esp_hal::{dma::DmaDescriptor, handler, interrupt::CpuInterrupt, peripherals::WIFI};
use macro_bits::{bit, check_bit};

use crate::{
    DefaultRawMutex,
    dma_list::DmaList,
    sync::{HardwareTxResultSignal, SignalQueue},
};

#[handler]
fn mac_handler() {
    let cause = unsafe { LowLevelDriver::get_and_clear_mac_interrupt_cause() };
    if cause.is_empty() {
        return;
    }

    if cause.rx() {
        WIFI_RX_SIGNAL_QUEUE.put();
    }
    if cause.tx_success() {
        let tx_queue_status = Ok(());
        unsafe {
            LowLevelDriver::process_tx_status(tx_queue_status, |queue| {
                HARDWARE_TX_RESULT_SIGNALS[queue.hardware_slot()].signal(tx_queue_status);
            })
        };
    }
    if cause.tx_timeout() {
        let tx_queue_status = Err(ChannelAccessError::Timeout);
        unsafe {
            LowLevelDriver::process_tx_status(tx_queue_status, |queue| {
                HARDWARE_TX_RESULT_SIGNALS[queue.hardware_slot()].signal(tx_queue_status);
                LowLevelDriver::set_tx_queue_status(queue, HardwareTxQueueStatus::Disabled);
            })
        };
    }
    if cause.tx_collision() {
        let tx_queue_status = Err(ChannelAccessError::Collision);
        unsafe {
            LowLevelDriver::process_tx_status(tx_queue_status, |queue| {
                HARDWARE_TX_RESULT_SIGNALS[queue.hardware_slot()].signal(tx_queue_status);
                LowLevelDriver::set_tx_queue_status(queue, HardwareTxQueueStatus::Disabled);
            })
        };
    }
}
#[cfg(pwr_interrupt_present)]
#[handler]
fn pwr_handler() {
    let _cause = unsafe { LowLevelDriver::get_and_clear_pwr_interrupt_cause() };
    // We don't do anything yet.
}

/// Tracks the number of frames, that are still in the RX queue.
static WIFI_RX_SIGNAL_QUEUE: SignalQueue = SignalQueue::new();

/// These are for knowing, when transmission has finished, and with which result.
static HARDWARE_TX_RESULT_SIGNALS: [HardwareTxResultSignal; 5] =
    [const { HardwareTxResultSignal::new() }; 5];

// We run tx_pwctrl_background every four transmissions.
static FRAMES_SINCE_LAST_TXPWR_CTRL: AtomicU8 = AtomicU8::new(0);

const RX_BUFFER_SIZE: usize = 1600;

/// Resources for the Wi-Fi peripheral.
///
/// `BUFFER_COUNT` has to be at least 2.
pub struct WiFiResources<const BUFFER_COUNT: usize> {
    /// Slab of DMA buffers and descriptors.
    buffer_slab: DmaBufferSlab<BUFFER_COUNT, RX_BUFFER_SIZE>,
    /// Storage for the DMA list.
    dma_list: Option<blocking_mutex::Mutex<DefaultRawMutex, RefCell<DmaList>>>,
    /// Storage for the LL driver.
    ll_driver: Option<LowLevelDriver>,
    /// DMA descriptors for all TX slots.
    tx_dma_descriptors: Option<[DmaDescriptor; 5]>,
}
impl<const BUFFER_COUNT: usize> WiFiResources<BUFFER_COUNT> {
    /// Create new DMA resources.
    pub const fn new() -> Self {
        assert!(BUFFER_COUNT >= 2, "BUFFER_COUNT has to be larger than 2.");
        Self {
            buffer_slab: DmaBufferSlab::new(),
            dma_list: None,
            ll_driver: None,
            tx_dma_descriptors: None,
        }
    }
    /// Initialize the DMA resources.
    ///
    /// SAFETY:
    /// You must ensure, that this is only called once, and that everything that is returned is
    /// used only while the resources are still alive.
    pub(crate) unsafe fn init(
        &mut self,
        ll_driver: LowLevelDriver,
    ) -> (
        &blocking_mutex::Mutex<DefaultRawMutex, RefCell<DmaList>>,
        &LowLevelDriver,
        &mut [DmaDescriptor; 5],
    ) {
        let (base_ptr, last_ptr) = unsafe { self.buffer_slab.init() };
        let ll_driver = self.ll_driver.insert(ll_driver);
        // # Safety
        // The DMA list is guaranteed to have the same lifetime as the LL driver, since they're
        // both contained in the [WiFiResources] struct, so they'll have the same lifetime.
        // This just avoids self referential structs.
        let dma_list = self
            .dma_list
            .insert(blocking_mutex::Mutex::new(RefCell::new(DmaList::new(
                base_ptr,
                last_ptr,
                unsafe {
                    core::mem::transmute::<&LowLevelDriver, &'static LowLevelDriver>(ll_driver)
                },
            ))));
        let tx_dma_descriptors = self.tx_dma_descriptors.insert([DmaDescriptor::EMPTY; 5]);
        (dma_list, ll_driver, tx_dma_descriptors)
    }
}

impl<const BUFFER_COUNT: usize> Default for WiFiResources<BUFFER_COUNT> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
/// This determines what frames bypass the RX filter.
pub enum ScanningMode {
    #[default]
    /// No frames can bypass the MAC filter.
    Disabled,
    /// Only beacons and probe response frames bypass the MAC filter.
    BeaconsOnly,
    /// Management and data frames bypass the MAC filter.
    ManagementAndData,
}

/// A trait giving access to the [LowLevelDriver] of a struct.
pub trait HasLowLevelDriver {
    /// Get the [LowLevelDriver].
    ///
    /// # Safety
    /// Honestly, here be dragons. Use this at your own risk. This is only intended for debugging
    /// or pushing the hardware beyond, what we implement. Read the docs of the [LowLevelDriver]
    /// carefully, if you want a shot at sleeping soundly when using this.
    ///
    /// (The author would like to note, that he also managed to shoot himself in the foot numerous
    /// times, with the LL driver.)
    unsafe fn ll_driver_ref(&self) -> &LowLevelDriver;

    /// Get the current MAC time.
    fn mac_time(&self) -> esp_hal::time::Instant {
        // # Safety
        // The LL driver will be alive for 'res, which is at least as long, as the existance of
        // this struct.
        unsafe { LowLevelDriver::mac_time() }
    }

    /// Returns the current channel.
    fn get_channel(&self) -> u8 {
        CURRENT_CHANNEL.load(Ordering::Relaxed)
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
/// Errors related to the crypto APIs.
pub enum CryptoError {
    /// The interface, key slot or key ID were out of bounds.
    OutOfBounds,
    /// The multicast bit of the MAC address was set.
    MulticastBitSet,
}
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
/// The requested channel is invalid.
pub struct InvalidChannelError;
/// Provides control over the channel.
pub trait ChannelControl: HasLowLevelDriver {
    /// Set the channel on which to operate.
    ///
    /// NOTE: This uses the proprietary blob.
    ///
    /// It also accepts channel 14, which is only allowed in Japan with the DSSS PHY. However, the
    /// word "allowed" is used very deliberately here. Make of that what you will...
    fn set_channel(&mut self, channel_number: u8) -> Result<(), InvalidChannelError> {
        if !(1..=14).contains(&channel_number) {
            return Err(InvalidChannelError);
        }
        trace!("Changing channel to {}.", channel_number);
        unsafe { self.ll_driver_ref() }.set_channel(channel_number);
        CURRENT_CHANNEL.store(channel_number, Ordering::Relaxed);
        Ok(())
    }
}
/// Provides control over hardware key slots.
pub trait CryptoControl: HasLowLevelDriver {
    /// Check if the key slot is active.
    ///
    /// If `key_slot` is larger than or equal to [WiFi::KEY_SLOT_COUNT], an error will be returned.
    fn key_slot_in_use(&self, key_slot: usize) -> Result<bool, OutOfBounds> {
        WiFi::validate_key_slot(key_slot)
            .map(|_| unsafe { self.ll_driver_ref() }.key_slot_enabled(key_slot))
    }
    /// Set a cryptographic key.
    ///
    /// This is equivalent to MLME-SETKEYS.request with one key descriptor.
    /// An error will be returned, if the key slot, key ID or interface are out of bounds, or the
    /// address is multicast. If the key slot is currently in use, this will overwrite the current
    /// entry.
    fn set_key(
        &mut self,
        key_slot: usize,
        interface: usize,
        key_id: u8,
        address: [u8; 6],
        cipher_parameters: CipherParameters<'_>,
    ) -> Result<(), CryptoError> {
        // If anyone wonders about all of the asserts here, I got bored and optimized the shit out
        // of this function. Sadly the compiler still fails to prove some other stuff.

        let key = cipher_parameters.key();

        let ll_driver = unsafe { self.ll_driver_ref() };

        WiFi::validate_key_slot(key_slot).map_err(|_| CryptoError::OutOfBounds)?;
        WiFi::validate_interface(interface).map_err(|_| CryptoError::OutOfBounds)?;
        if key_id >= 4 {
            return Err(CryptoError::OutOfBounds);
        }

        if check_bit!(address[0], bit!(0)) {
            return Err(CryptoError::MulticastBitSet);
        }

        ll_driver.set_key_slot_parameters(
            key_slot,
            &KeySlotParameters {
                address,
                key_id,
                interface: interface as u8,
                pairwise: cipher_parameters.is_pairwise(),
                group: cipher_parameters.is_group(),
                algorithm: cipher_parameters.algorithm(),
                wep_104: cipher_parameters.is_wep_104(),
            },
        );
        ll_driver.set_key(key_slot, key);
        ll_driver.set_key_slot_enable(key_slot, true);

        let (protect_management_frames, protect_signaling_and_payload) =
            if let Some(aes_cipher_parameters) = cipher_parameters.aes_cipher_parameters() {
                (
                    aes_cipher_parameters.mfp_enabled,
                    aes_cipher_parameters.spp_enabled,
                )
            } else {
                (false, false)
            };
        ll_driver.set_interface_crypto_parameters(
            interface,
            protect_management_frames,
            protect_signaling_and_payload,
            cipher_parameters.is_aead(),
        );
        // Currently, we do not support WAPI. What a shame...
        ll_driver.set_sms4_status(false);

        Ok(())
    }
    /// Delete a cryptographic key.
    ///
    /// This is equivalent to MLME-DELETEKEYS.request with one key descriptor.
    fn delete_key(&mut self, key_slot: usize) -> Result<(), OutOfBounds> {
        WiFi::validate_key_slot(key_slot).map(|_| {
            let ll_driver = unsafe { self.ll_driver_ref() };
            ll_driver.set_key_slot_enable(key_slot, false);
            ll_driver.clear_key_slot(key_slot);
        })
    }
    /// Dump the entire contents of a key slot.
    fn dump_key_slot(&self, key_slot: usize) -> Result<(), OutOfBounds> {
        WiFi::validate_key_slot(key_slot)?;

        let wifi = WIFI::regs();
        let crypto_key_slot = wifi.crypto_key_slot(key_slot);

        let mut key_bytes = [0x00u8; 32];
        for (buffer_chunk, key_word) in key_bytes
            .chunks_mut(4)
            .zip(crypto_key_slot.key_value_iter())
        {
            buffer_chunk.copy_from_slice(key_word.read().bits().to_le_bytes().as_slice());
        }
        let mut address = [0x00u8; 6];
        address[..4].copy_from_slice(
            crypto_key_slot
                .addr_low()
                .read()
                .bits()
                .to_le_bytes()
                .as_slice(),
        );
        address[4..].copy_from_slice(
            crypto_key_slot
                .addr_high()
                .read()
                .addr()
                .bits()
                .to_le_bytes()
                .as_slice(),
        );

        let control = crypto_key_slot.addr_high().read().bits() >> 16;

        #[cfg(feature = "defmt")]
        info!(
            "Key Slot: {} Address: {=[u8]:02x} Key: {=[u8]:02x} Control Reg: {:04x}",
            key_slot, address, key_bytes, control
        );
        #[cfg(not(feature = "defmt"))]
        info!(
            "Key Slot: {} Address: {:02x?} Key: {:02x?} Control Reg: {:04x}",
            key_slot, address, key_bytes, control
        );

        Ok(())
    }
    /// Dump the values of the crypto control registers.
    fn dump_crypto_config(&self) {
        let wifi = WIFI::regs();
        for (i, interface_crypto_control) in wifi
            .crypto_control()
            .interface_crypto_control_iter()
            .enumerate()
        {
            info!(
                "Interface: {} Crypto Control Reg: {:08x}",
                i,
                interface_crypto_control.read().bits()
            );
        }
        info!(
            "General Crypto Control Reg: {:08x}",
            wifi.crypto_control().general_crypto_control().read().bits()
        );
        let mut enabled_key_slots = [0x00u8; 32];
        let enabled_slot_count = wifi
            .crypto_control()
            .crypto_key_slot_state()
            .read()
            .key_slot_enable_iter()
            .enumerate()
            .filter_map(|(i, enabled)| enabled.bit().then_some(i))
            .zip(enabled_key_slots.iter_mut())
            .map(|(key_slot, k)| *k = key_slot as u8)
            .count();
        info!(
            "Enabled Key Slots: {:?}",
            &enabled_key_slots[..enabled_slot_count]
        );
    }
}
/// Controls the cryptographic hardware of the Wi-Fi peripheral.
pub struct CryptoController<'res> {
    /// The low level driver.
    ll_driver: &'res LowLevelDriver,
}
impl<'res> HasLowLevelDriver for CryptoController<'res> {
    unsafe fn ll_driver_ref(&self) -> &LowLevelDriver {
        self.ll_driver
    }
}
impl CryptoControl for CryptoController<'_> {}

/// Controls the filtering of a single RX interface.
pub struct RxInterfaceController<'res> {
    /// The low level driver.
    ll_driver: &'res LowLevelDriver,
    /// The RX interface this corresponds to.
    interface: usize,
}
impl RxInterfaceController<'_> {
    /// Specifiy whether the BSSID is checked or not.
    ///
    /// This is used to control a special behaviour of the hardware. If the RA filter is enabled,
    /// but the BSSID is disabled, it will assume that `BSSID == RA`. However setting this to
    /// `false` will stop this behaviour.
    pub fn set_filter_bssid_check(&self, enabled: bool) {
        self.ll_driver
            .set_bssid_check_enable(self.interface, enabled);
    }
    /// Configure a filter.
    ///
    /// This will default the mask to an all ones mask, and enable the filter.
    pub fn set_filter(&self, filter_bank: RxFilterBank, address: [u8; 6]) {
        self.ll_driver
            .set_filter_address(self.interface, filter_bank, &address);
        self.ll_driver
            .set_filter_mask(self.interface, filter_bank, &[0xff; 6]);
        self.ll_driver
            .set_filter_enable(self.interface, filter_bank, true);
    }
    /// Override the mask of a filter.
    ///
    /// This doesn't check, if the filter is actually enabled.
    pub fn override_filter_mask(&self, filter_bank: RxFilterBank, mask: [u8; 6]) {
        self.ll_driver
            .set_filter_mask(self.interface, filter_bank, &mask)
    }

    /// Clear and disable a filter for a bank.
    ///
    /// This will return the filter into its default state.
    pub fn clear_filter(&self, filter_bank: RxFilterBank) {
        self.ll_driver.clear_filter(self.interface, filter_bank);
    }
    /// Enable or disable scanning mode.
    pub fn set_scanning_mode(&self, scanning_mode: ScanningMode) {
        self.ll_driver.set_scanning_mode_parameters(
            self.interface,
            scanning_mode == ScanningMode::BeaconsOnly,
            scanning_mode == ScanningMode::ManagementAndData,
        );
    }
    /// Set the type of addresses that should be filtered.
    ///
    /// If an address type is unfiltered, frames with RAs matching that address type will pass the
    /// filter unconditionally. Otherwise the frame will be filtered as usual.
    pub fn set_filtered_address_types(&self, unicast: bool, multicast: bool) {
        self.ll_driver
            .set_filtered_address_types(self.interface, unicast, multicast);
    }
    /// Configure which control frames pass the filter.
    pub fn set_control_frame_filter(&self, config: &ControlFrameFilterConfig) {
        self.ll_driver
            .set_control_frame_filter(self.interface, config);
    }
    /// Get the index of the RX interface, this is associated with.
    pub const fn interface(&self) -> usize {
        self.interface
    }
}
impl HasLowLevelDriver for RxInterfaceController<'_> {
    unsafe fn ll_driver_ref(&self) -> &LowLevelDriver {
        self.ll_driver
    }
}

/// The channel, we're currently on.
///
/// We initialize this to one, as we also go to channel one during init, so this is valid the first
/// time it's accessed.
static CURRENT_CHANNEL: AtomicU8 = AtomicU8::new(1);

/// Controls all characeristics of the Wi-Fi peripheral, that can't reasonably be split up.
pub struct ChannelController<'res> {
    ll_driver: &'res LowLevelDriver,
}
impl HasLowLevelDriver for ChannelController<'_> {
    unsafe fn ll_driver_ref(&self) -> &LowLevelDriver {
        self.ll_driver
    }
}
impl ChannelControl for ChannelController<'_> {}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
/// What should be done, if an error occurs, while transmitting.
pub enum TxErrorBehaviour<'a> {
    /// Retry as many times, as specified.
    ///
    /// With the same rate.
    RetryUntil(u8),
    /// Retry with the specified rates.
    ///
    /// Rate control algorithms like this.
    MultiRateRetry(&'a [TxPhyRate]),
    /// Drop the MPDU.
    #[default]
    Drop,
}
impl<'a> TxErrorBehaviour<'a> {
    #[inline]
    /// Get an iterator over the rates for each TX attempts.
    pub fn tx_attempt_rate_iter<'b>(
        &'b self,
        initial_rate: TxPhyRate,
    ) -> impl Iterator<Item = TxPhyRate> + Send + Sync + use<'b>
    where
        'b: 'a,
    {
        let tx_attempts = match self {
            Self::RetryUntil(retries) => *retries as usize + 1,
            Self::MultiRateRetry(rates) => rates.len(),
            Self::Drop => 1,
        };
        (0..tx_attempts).map(move |i| match self {
            Self::RetryUntil(_) => initial_rate,
            Self::MultiRateRetry(rates) => {
                if i == 0 {
                    initial_rate
                } else {
                    rates[i]
                }
            }
            Self::Drop => initial_rate,
        })
    }
}
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
/// Physical Layer Convergence Protocol (PLCP) parameters
///
/// These parameters influence actual physical parameters of the transmission, like the data rate
/// or medium access.
pub struct TxPlcpParameters {
    /// The transmission data rate.
    pub rate: TxPhyRate,
    /// Requires using `..Default::default()` initialization, to ensure future backwards
    /// compatibility.
    pub _ne: NonExhaustive,
}
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
/// Parameters for the MAC.
///
/// These parameters control the MAC peripherals handling of the frame, like wait-for-ACK, frame
/// encryption, transmission retries etc.
pub struct TxMacParameters {
    /// Which key slot should be used for encrypting the transmission.
    ///
    /// If the frame shouldn't be encrypted set this to [None].
    pub key_slot_index: Option<u8>,
    /// Is an ACK expected for the transmission.
    ///
    /// The ACK is expected to be addressed to the interface specified during transmission.
    pub wait_for_ack: bool,
    /// Should the sequence number be replaced, with one tracked by the driver.
    ///
    /// Usually you want this to be enabled, however it's not the default, as it may not be the
    /// expected behaviour for the driver to change your frames.
    ///
    /// NOTE: This will change the buffer!
    pub override_seq_num: bool,
    /// Requires using `..Default::default()` initialization, to ensure future backwards
    /// compatibility.
    pub _ne: NonExhaustive,
}

/// Endpoint for a hardware transmit queue.
///
/// This allows you to transmit frames on the queue this corresponds to.
pub struct TxQueueEndpoint<'res> {
    ll_driver: &'res LowLevelDriver,
    /// The hardware TX queue corresponding to this endpoint.
    queue: HardwareTxQueue,
    /// TX DMA descriptor used for the slot.
    dma_descriptor: &'res mut DmaDescriptor,
}
impl TxQueueEndpoint<'_> {
    /// Transmit a frame.
    ///
    /// Returns the amount of retries.
    ///
    /// The buffer doesn't need to have room for an FCS, even though the hardware requires this.
    /// This limitation is bypassed, by just adding 4 to the length passed to the hardware, since
    /// we're 99% sure, the hardware never reads those bytes.
    /// The reason a mutable reference is required, is because for retransmissions, we need to set
    /// an extra bit in the FCS flags and may have to override the sequence number. This also
    /// means, that the buffer will be different afterwards.
    ///
    /// You must set a [TxErrorBehaviour], so the driver knows what to do in case of a TX error.
    /// The advantage of using this instead of bit banging a higher layer fix is, that we don't
    /// have to reacquire a TX slot every time TX fails.
    pub fn transmit<'a>(
        &'a mut self,
        interface: usize,
        plcp_parameters: &'a TxPlcpParameters,
        mac_parameters: &'a TxMacParameters,
        error_behaviour: TxErrorBehaviour<'a>,
        mpdu_buf: &'a mut [u8],
    ) -> impl Future<Output = Result<u8, TxError>> + Send + 'a {
        self.ll_driver.transmit_with_retry(
            interface,
            plcp_parameters,
            mac_parameters,
            error_behaviour,
            self.queue,
            Pin::new(self.dma_descriptor),
            mpdu_buf,
        )
    }
    /// Transmit a frame and don't attempt to retransmit it.
    ///
    /// Unlike [Self::transmit], this will only try to transmit the frame once, and start the
    /// transmission, before the future is returned. This makes it more suited for transmitting
    /// frames in "real-time", since the time difference, between calling [Self::transmit_oneshot]
    /// and the hardware being configured with the frame, is very small and close to constant. The
    /// asynchronous part solely performs the waiting.
    pub fn transmit_oneshot<'a>(
        &'a mut self,
        interface: usize,
        plcp_parameters: &'a TxPlcpParameters,
        mac_parameters: &'a TxMacParameters,
        mpdu_buf: &'a mut [u8],
    ) -> impl Future<Output = Result<(), TxError>> + Send + Sync + 'a {
        self.ll_driver.transmit_oneshot(
            interface,
            plcp_parameters,
            mac_parameters,
            self.queue,
            Pin::new(self.dma_descriptor),
            mpdu_buf,
        )
    }
    /// The hardware TX queue, to which this endpoint responds.
    pub const fn hardware_tx_queue(&self) -> HardwareTxQueue {
        self.queue
    }
}
impl HasLowLevelDriver for TxQueueEndpoint<'_> {
    unsafe fn ll_driver_ref(&self) -> &LowLevelDriver {
        self.ll_driver
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
/// Errors that can occur in the context of transmissions.
pub enum TxError {
    /// Errors related to accessing the channel in the first place.
    ChannelAccess(ChannelAccessError),
    /// Errors with the protocol of the MAC.
    MacProtocol(MacProtocolError),
    /// The provided buffer is too short.
    ///
    /// NOTE: This can happen, if you want to transmit a control frame, but enable
    /// [TxMacParameters::override_seq_num], since control frames do not have a sequence number.
    BufferTooShort,
    /// The provided key slot is disabled.
    DisabledKeySlot,
    /// The provided interface or key slot is out of bounds.
    OutOfBounds,
}
/// The current sequence number tracked by the driver.
static CURRENT_SEQUENCE_NUMBER: AtomicU16 = AtomicU16::new(0);

// Access to the DMA list, shouldn't really be public.
mod private {
    use core::{cell::RefCell, future::Future, ops::DerefMut, pin::Pin, sync::atomic::Ordering};

    use embassy_sync::blocking_mutex;
    use esp_hal::dma::{DmaDescriptor, Owner};
    use macro_bits::bit;

    use crate::{
        DefaultRawMutex,
        async_driver::{
            CURRENT_SEQUENCE_NUMBER, FRAMES_SINCE_LAST_TXPWR_CTRL, HARDWARE_TX_RESULT_SIGNALS,
            HasLowLevelDriver, TxError, TxMacParameters, TxPlcpParameters, WiFi,
        },
        dma_list::DmaList,
        ll::{HardwareTxQueue, LowLevelDriver},
        prelude::TxErrorBehaviour,
        rates::TxPhyRate,
        sync::DropGuard,
    };

    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
    /// Forces a struct to be initialized with `..Default::default()`.
    pub struct NonExhaustive;

    /// Grants access to the DMA list.
    pub trait HasDmaList<'res> {
        /// Get the DMA list.
        fn dma_list_ref(&self) -> &'res blocking_mutex::Mutex<DefaultRawMutex, RefCell<DmaList>>;
    }
    /// Extends the low level driver with the asynchronous transmission system.
    pub trait AsyncTransmitExt: HasLowLevelDriver {
        /// Sets up a frame for transmission.
        fn setup_tx(
            &self,
            interface: usize,
            rate: TxPhyRate,
            mac_parameters: &TxMacParameters,
            queue: HardwareTxQueue,
            dma_descriptor: Pin<&DmaDescriptor>,
            duration: u16,
        ) {
            // To setup a frame for transmission, we have to do a few things with the hardware.
            // 1. Configure channel access parameters, like the medium access timeout (unsure), as
            //    well as the number of AIFS and backoff slots.
            // 2. Give it a pointer to our DMA descriptor.
            // 3. Give it a bunch of parameters, like data rate, frame length, interface, key slot
            //    and whether it should wait for an ACK.
            // 4. Optionally configure parameters for HT frames. (This will need to be extended for
            //    HE and VHT.)
            let ll_driver = unsafe { self.ll_driver_ref() };

            ll_driver.set_channel_access_parameters(queue, 10, 2, 2);
            ll_driver.set_plcp0(queue, dma_descriptor, mac_parameters.wait_for_ack);
            ll_driver.set_plcp1(
                queue,
                rate,
                dma_descriptor.len(),
                interface,
                mac_parameters.key_slot_index,
            );
            ll_driver.set_plcp2(queue);
            ll_driver.set_duration(queue, duration);

            if let TxPhyRate::Ht(ht_rate) = rate {
                ll_driver.set_ht_parameters(
                    queue,
                    ht_rate.mcs_index(),
                    ht_rate.short_gi(),
                    dma_descriptor.len(),
                );
            }
        }
        /// Transmit a frame and wait for the transmission to be completed.
        ///
        /// No retransmissions are performed here.
        fn transmit_raw<'a>(
            &'a self,
            interface: usize,
            rate: TxPhyRate,
            mac_parameters: &'a TxMacParameters,
            queue: HardwareTxQueue,
            dma_descriptor: Pin<&DmaDescriptor>,
            duration: Result<u16, TxError>,
        ) -> impl Future<Output = Result<(), TxError>> + Send + Sync + 'a {
            let ll_driver = unsafe { self.ll_driver_ref() };
            let hardware_tx_result_signal = &HARDWARE_TX_RESULT_SIGNALS[queue.hardware_slot()];

            // This will reset the slot, once the drop guard goes out of scope, or the future is
            // dropped.
            let tx_done_wait_drop_guard = DropGuard::new(move || {
                ll_driver.tx_done(queue);
                hardware_tx_result_signal.reset();
            });
            // If any error was forwarded through the duration, we don't do anything here and
            // return an error on the first poll.
            if let Ok(duration) = duration {
                // Setup the frame for transmission. We'll start TX a little later.
                self.setup_tx(
                    interface,
                    rate,
                    mac_parameters,
                    queue,
                    dma_descriptor.as_ref(),
                    duration,
                );

                // This compensates for other slots being marked as done, without it being used at all.
                hardware_tx_result_signal.reset();
                ll_driver.start_tx_queue(queue);
            }

            // This weird construction ensures, that the transmission gets setup and started
            // basically in real-time, and only the waiting is actually done in async.
            //
            // Note that self isn't used anywhere in this block, thus allowing the future to be
            // Send + Sync.
            async move {
                duration?;
                // We wait for the transmission to complete here.
                HARDWARE_TX_RESULT_SIGNALS[queue.hardware_slot()]
                    .wait()
                    .await
                    .inspect(|_| {
                        if FRAMES_SINCE_LAST_TXPWR_CTRL.fetch_add(1, Ordering::Relaxed) == 4 {
                            ll_driver.run_power_control();
                            FRAMES_SINCE_LAST_TXPWR_CTRL.store(0, Ordering::Relaxed);
                        }
                    })
                    .map_err(TxError::ChannelAccess)?;

                tx_done_wait_drop_guard.detonate();

                ll_driver
                    .get_tx_mac_protocol_result(queue)
                    .map_err(TxError::MacProtocol)
            }
        }
        fn prepare_frame_for_tx(
            &self,
            interface: usize,
            mac_parameters: &TxMacParameters,
            mpdu_buf: &mut [u8],
        ) -> Result<u16, TxError> {
            let Some(duration) = mpdu_buf
                .get(2..4)
                .map(|bytes| u16::from_le_bytes(bytes.try_into().unwrap()))
            else {
                return Err(TxError::BufferTooShort);
            };

            if mac_parameters.override_seq_num {
                let seq_num = CURRENT_SEQUENCE_NUMBER.fetch_add(1, Ordering::Relaxed);
                if let Some(sequence_number) = mpdu_buf.get_mut(22..24) {
                    sequence_number.copy_from_slice((seq_num << 4).to_le_bytes().as_slice());
                } else {
                    return Err(TxError::BufferTooShort);
                }
            }
            if let Some(key_slot_index) = mac_parameters.key_slot_index {
                WiFi::validate_key_slot(key_slot_index as usize)
                    .map_err(|_| TxError::OutOfBounds)?;
                if !unsafe { self.ll_driver_ref() }.key_slot_enabled(key_slot_index as usize) {
                    return Err(TxError::DisabledKeySlot)?;
                }
            }
            WiFi::validate_interface(interface).map_err(|_| TxError::OutOfBounds)?;

            Ok(duration)
        }
        fn prepare_dma_descriptor_for_tx(mpdu_buf: &mut [u8], dma_descriptor: &mut DmaDescriptor) {
            // Attach length for FCS.
            let frame_length = mpdu_buf.len() + 4;

            // We create and pin the DMA descriptor on the stack.
            dma_descriptor.set_size(frame_length);
            dma_descriptor.set_length(frame_length);
            dma_descriptor.set_owner(Owner::Dma);
            dma_descriptor.set_suc_eof(true);
            dma_descriptor.buffer = mpdu_buf.as_mut_ptr();
        }
        fn transmit_oneshot<'a>(
            &'a self,
            interface: usize,
            plcp_parameters: &'a TxPlcpParameters,
            mac_parameters: &'a TxMacParameters,
            queue: HardwareTxQueue,
            mut dma_descriptor: Pin<&'a mut DmaDescriptor>,
            mpdu_buf: &mut [u8],
        ) -> impl Future<Output = Result<(), TxError>> + Send + Sync + 'a {
            let duration = self.prepare_frame_for_tx(interface, mac_parameters, mpdu_buf);
            Self::prepare_dma_descriptor_for_tx(mpdu_buf, dma_descriptor.deref_mut());

            self.transmit_raw(
                interface,
                plcp_parameters.rate,
                mac_parameters,
                queue,
                dma_descriptor.into_ref(),
                duration,
            )
        }
        #[allow(clippy::too_many_arguments)]
        #[inline(never)]
        /// Transmit a frame and correctly handle errors.
        ///
        /// This will also read the duration from the buffer and configure the hardware with it.
        async fn transmit_with_retry<'a>(
            &'a self,
            interface: usize,
            plcp_parameters: &'a TxPlcpParameters,
            mac_parameters: &'a TxMacParameters,
            error_behaviour: TxErrorBehaviour<'a>,
            queue: HardwareTxQueue,
            mut dma_descriptor: Pin<&'a mut DmaDescriptor>,
            mpdu_buf: &mut [u8],
        ) -> Result<u8, TxError> {
            let duration = self.prepare_frame_for_tx(interface, mac_parameters, mpdu_buf)?;
            Self::prepare_dma_descriptor_for_tx(mpdu_buf, dma_descriptor.deref_mut());

            // We start transmitting and adapt the rate as required. As soon, as a transmission
            // succeeds, we return.
            let mut last_res = Ok::<u8, TxError>(0);

            for (i, tx_attempt_rate) in error_behaviour
                .tx_attempt_rate_iter(plcp_parameters.rate)
                .enumerate()
            {
                // The transmission will already be started, before the first poll, which ensures
                // that TX happens almost in real time.
                last_res = self
                    .transmit_raw(
                        interface,
                        tx_attempt_rate,
                        mac_parameters,
                        queue,
                        dma_descriptor.as_ref(),
                        Ok(duration),
                    )
                    .await
                    .map(|_| i as u8);
                if let Err(err) = last_res {
                    trace!("Retransmitting MPDU due to: {:?}", err);
                } else {
                    break;
                }
            }
            if let Some(byte) = mpdu_buf.get_mut(1) {
                *byte &= !bit!(3);
            }
            if last_res.is_err() {
                trace!("Transmission of MPDU failed.");
            }
            last_res
        }
    }
    // Yeah this is a little weird, I know.
    impl HasLowLevelDriver for LowLevelDriver {
        unsafe fn ll_driver_ref(&self) -> &LowLevelDriver {
            self
        }
    }
    impl AsyncTransmitExt for LowLevelDriver {}
}
/// A trait implemented by structs, that allow asynchronously receiving frames.
pub trait AsyncReceive<'res>: HasDmaList<'res> {
    /// Receive a frame.
    ///
    /// Since this always returns a buffer, it will block until one is available. This includes, if
    /// RX is stopped.
    ///
    /// NOTE: The received frame will not contain an FCS or MIC.
    fn receive(&mut self) -> impl Future<Output = BorrowedBuffer<'res>> {
        async {
            // Sometimes the DMA list descriptors don't contain any data, even though the hardware indicated reception.
            // We loop until we get something.
            let dma_list_item = loop {
                WIFI_RX_SIGNAL_QUEUE.next().await;
                if let Some(current) = self
                    .dma_list_ref()
                    .lock(|dma_list| dma_list.borrow_mut().take_first())
                    && current.len() >= BorrowedBuffer::RX_CONTROL_HEADER_LENGTH
{
                        trace!("Received packet. len: {}", current.len());
                        break current;
                    }
                trace!("Received empty packet.");
            };

            BorrowedBuffer {
                dma_list: self.dma_list_ref(),
                dma_descriptor: dma_list_item,
            }
        }
    }
    /// Clear all currently pending frames in the RX queue.
    fn clear_rx_queue(&mut self) {
        self.dma_list_ref()
            .lock(|rx_dma_list| rx_dma_list.borrow_mut().clear());

        // If we don't reset the RX signal queue, we might get a bunch of false alarms the next
        // time RX is called.
        WIFI_RX_SIGNAL_QUEUE.reset();
    }
    /// Temporarily pause RX, or resume it.
    ///
    /// If you call this with `false`, RX will be paused, until you call this again with `true`.
    ///
    /// DEVELOPER'S NOTE: I'm 99% sure, that resuming RX this way should work just fine, even if
    /// the RX DMA list was completely empty when pausing RX. My tests seem to confirm that, but if
    /// you see any weird behavior, please open an issue.
    fn set_rx_status(&mut self, enabled: bool) {
        unsafe {
            LowLevelDriver::set_rx_enable(enabled);
        }
    }
    /// Log stats about the DMA list.
    fn log_dma_list_stats(&self) {
        self.dma_list_ref()
            .lock(|dma_list| dma_list.borrow().log_stats())
    }
}

/// Shovels received frames out of the hardware asynchronously.
pub struct AsyncRxEndpoint<'res> {
    dma_list: &'res blocking_mutex::Mutex<DefaultRawMutex, RefCell<DmaList>>,
}
impl<'res> HasDmaList<'res> for AsyncRxEndpoint<'res> {
    fn dma_list_ref(&self) -> &'res blocking_mutex::Mutex<DefaultRawMutex, RefCell<DmaList>> {
        self.dma_list
    }
}
impl<'res> AsyncReceive<'res> for AsyncRxEndpoint<'res> {}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
/// The provided key slot or interface was out of bounds.
pub struct OutOfBounds;

/// Split up components of the Wi-Fi driver.
///
/// This allows safely managing control over the hardware, by letting certain aspects only be
/// controlled by the code, that has a reference to the entity controlling those aspects.
/// For example, an [RxInterfaceController] can only control the RX interface, that it represents,
/// so it can be passed to other code, with the guarantee, that it can only modify its own
/// interface.
///
/// NOTE: Most of the structs only contain one or two references, so the extra space needed is fairly
/// manageable.
pub struct SplitDriverComponents<'res> {
    /// Provides control over the RX interfaces.
    pub rx_interface_controllers: [RxInterfaceController<'res>; INTERFACE_COUNT],
    /// Endpoint for receiving frames.
    pub rx_endpoint: AsyncRxEndpoint<'res>,
    /// Controllers for all hardware transmit queues.
    ///
    /// These are in the order of: `Beacon`, `Background`, `Best Effort`, `Video` and `Voice`.
    pub tx_queue_endpoints: [TxQueueEndpoint<'res>; 5],
    /// Controls the channel, on which the Wi-Fi peripheral operates.
    pub channel_controller: ChannelController<'res>,
    /// Controls the cryptographic hardware.
    pub crypto_controller: CryptoController<'res>,
}

/// Driver for the Wi-Fi peripheral.
///
/// WARNING: Currently dropping the driver is not properly implemented.
pub struct WiFi<'res> {
    dma_list: &'res blocking_mutex::Mutex<DefaultRawMutex, RefCell<DmaList>>,
    ll_driver: &'res LowLevelDriver,
    tx_dma_descriptors: Option<&'res mut [DmaDescriptor; 5]>,
}
impl<'res> WiFi<'res> {
    /// Initialize the WiFi peripheral.
    pub fn new<const BUFFER_COUNT: usize>(
        wifi: WIFI<'res>,
        wifi_resources: &'res mut WiFiResources<BUFFER_COUNT>,
    ) -> Self {
        trace!("Initializing WiFi.");
        let ll_driver = LowLevelDriver::new(wifi);

        ll_driver.configure_interrupt(
            WiFiInterrupt::Mac,
            CpuInterrupt::Interrupt0LevelPriority1,
            mac_handler,
        );

        #[cfg(pwr_interrupt_present)]
        ll_driver.configure_interrupt(
            WiFiInterrupt::Pwr,
            CpuInterrupt::Interrupt2LevelPriority1,
            pwr_handler,
        );

        let (dma_list, ll_driver, tx_dma_descriptors) = unsafe { wifi_resources.init(ll_driver) };

        let mut temp = Self {
            ll_driver,
            dma_list,
            tx_dma_descriptors: Some(tx_dma_descriptors),
        };
        temp.set_channel(1).unwrap();
        temp
    }
    /// Split the driver into logical units, that can safely control their portion of the hardware.
    pub fn split(mut self) -> SplitDriverComponents<'res> {
        let mut i = 0;
        let tx_queue_endpoints =
            self.tx_dma_descriptors
                .take()
                .unwrap()
                .each_mut()
                .map(|dma_descriptor| {
                    i += 1;
                    TxQueueEndpoint {
                        ll_driver: self.ll_driver,
                        queue: HardwareTxQueue::from_hardware_slot(i - 1).unwrap(),
                        dma_descriptor,
                    }
                });
        SplitDriverComponents {
            rx_interface_controllers: core::array::from_fn(|i| RxInterfaceController {
                ll_driver: self.ll_driver,
                interface: i,
            }),
            channel_controller: ChannelController {
                ll_driver: self.ll_driver,
            },
            rx_endpoint: AsyncRxEndpoint {
                dma_list: self.dma_list,
            },
            tx_queue_endpoints,
            crypto_controller: CryptoController {
                ll_driver: self.ll_driver,
            },
        }
    }
    #[inline(never)]
    /// Transmit a frame.
    ///
    /// Returns the amount of retries.
    ///
    /// The buffer doesn't need to have room for an FCS, even though the hardware requires this.
    /// This limitation is bypassed, by just adding 4 to the length passed to the hardware, since
    /// we're 99% sure, the hardware never reads those bytes.
    /// The reason a mutable reference is required, is because for retransmissions, we need to set
    /// an extra bit in the FCS flags and may have to override the sequence number. This also
    /// means, that the buffer will be different afterwards.
    ///
    /// You must set a [TxErrorBehaviour], so the driver knows what to do in case of a TX error.
    /// The advantage of using this instead of bit banging a higher layer fix is, that we don't
    /// have to reacquire a TX slot every time TX fails.
    pub fn transmit<'a>(
        &'a mut self,
        interface: usize,
        plcp_parameters: &'a TxPlcpParameters,
        mac_parameters: &'a TxMacParameters,
        error_behaviour: TxErrorBehaviour<'a>,
        queue: HardwareTxQueue,
        mpdu_buf: &'a mut [u8],
    ) -> impl Future<Output = Result<u8, TxError>> + 'a {
        self.ll_driver.transmit_with_retry(
            interface,
            plcp_parameters,
            mac_parameters,
            error_behaviour,
            queue,
            Pin::new(self.tx_dma_descriptors.as_mut().unwrap().each_mut()[queue.hardware_slot()]),
            mpdu_buf,
        )
    }
    /// Transmit a frame and don't attempt to retransmit it.
    ///
    /// Unlike [WiFi::transmit], this will only try to transmit the frame once.
    ///
    /// # Timing
    /// The transmission is set up and started, before returning the future, which only performs
    /// the waiting. Thus the time between calling this function and the hardware being handed the
    /// frame is essentially constant (except for very minor variations, with different TX
    /// parameters).
    ///
    pub fn transmit_oneshot<'a>(
        &'a mut self,
        interface: usize,
        plcp_parameters: &'a TxPlcpParameters,
        mac_parameters: &'a TxMacParameters,
        queue: HardwareTxQueue,
        mpdu_buf: &'a mut [u8],
    ) -> impl Future<Output = Result<(), TxError>> + Send + Sync + 'a {
        self.ll_driver.transmit_oneshot(
            interface,
            plcp_parameters,
            mac_parameters,
            queue,
            Pin::new(self.tx_dma_descriptors.as_mut().unwrap().each_mut()[queue.hardware_slot()]),
            mpdu_buf,
        )
    }
    /// Check if that interface is valid.
    pub const fn validate_interface(interface: usize) -> Result<(), OutOfBounds> {
        if interface < INTERFACE_COUNT {
            Ok(())
        } else {
            Err(OutOfBounds)
        }
    }
    /// Specifiy whether the BSSID is checked or not.
    ///
    /// This is used to control a special behaviour of the hardware. If the RA filter is enabled,
    /// but the BSSID is disabled, it will assume that `BSSID == RA`. However setting this to
    /// `false` will stop this behaviour.
    pub fn set_filter_bssid_check(
        &mut self,
        interface: usize,
        enabled: bool,
    ) -> Result<(), OutOfBounds> {
        Self::validate_interface(interface)
            .inspect(|_| self.ll_driver.set_bssid_check_enable(interface, enabled))
    }
    /// Configure a filter.
    ///
    /// This will default the mask to an all ones mask, and enable the filter.
    pub fn set_filter(
        &mut self,
        interface: usize,
        filter_bank: RxFilterBank,
        address: [u8; 6],
    ) -> Result<(), OutOfBounds> {
        Self::validate_interface(interface).inspect(|_| {
            self.ll_driver
                .set_filter_address(interface, filter_bank, &address);
            self.ll_driver
                .set_filter_mask(interface, filter_bank, &[0xff; 6]);
            self.ll_driver
                .set_filter_enable(interface, filter_bank, true);
        })
    }
    /// Override the mask of a filter.
    ///
    /// This doesn't check, if the filter is actually enabled.
    pub fn override_filter_mask(
        &mut self,
        interface: usize,
        filter_bank: RxFilterBank,
        mask: [u8; 6],
    ) -> Result<(), OutOfBounds> {
        Self::validate_interface(interface).inspect(|_| {
            self.ll_driver
                .set_filter_mask(interface, filter_bank, &mask)
        })
    }

    /// Clear and disable a filter for an interface and bank.
    ///
    /// This will return the filter into its default state.
    pub fn clear_filter(
        &mut self,
        interface: usize,
        filter_bank: RxFilterBank,
    ) -> Result<(), OutOfBounds> {
        Self::validate_interface(interface)
            .inspect(|_| self.ll_driver.clear_filter(interface, filter_bank))
    }
    /// Enable or disable scanning mode.
    pub fn set_scanning_mode(
        &mut self,
        interface: usize,
        scanning_mode: ScanningMode,
    ) -> Result<(), OutOfBounds> {
        Self::validate_interface(interface).inspect(|_| {
            self.ll_driver.set_scanning_mode_parameters(
                interface,
                scanning_mode == ScanningMode::BeaconsOnly,
                scanning_mode == ScanningMode::ManagementAndData,
            );
        })
    }
/// Set the type of addresses that should be filtered.
    ///
    /// If an address type is unfiltered, frames with RAs matching that address type will pass the
    /// filter unconditionally. Otherwise the frame will be filtered as usual.
    pub fn set_filtered_address_types(
        &self,
        interface: usize,
        unicast: bool,
        multicast: bool,
    ) -> Result<(), OutOfBounds> {
        Self::validate_interface(interface).inspect(|_| {
            self.ll_driver
                .set_filtered_address_types(interface, unicast, multicast);
        })
    }
    /// Configure which control frames pass the filter.
    pub fn set_control_frame_filter(
        &self,
        interface: usize,
        config: &ControlFrameFilterConfig,
    ) -> Result<(), OutOfBounds> {
        Self::validate_interface(interface).inspect(|_| {
            self.ll_driver.set_control_frame_filter(interface, config);
        })
    }
    /// Check if they key slot is valid.
    pub const fn validate_key_slot(key_slot: usize) -> Result<(), OutOfBounds> {
        if key_slot < KEY_SLOT_COUNT {
            Ok(())
        } else {
            Err(OutOfBounds)
        }
    }
}
impl HasLowLevelDriver for WiFi<'_> {
    unsafe fn ll_driver_ref(&self) -> &LowLevelDriver {
        self.ll_driver
    }
}
impl ChannelControl for WiFi<'_> {}
impl CryptoControl for WiFi<'_> {}

impl<'res> HasDmaList<'res> for WiFi<'res> {
    fn dma_list_ref(&self) -> &'res blocking_mutex::Mutex<DefaultRawMutex, RefCell<DmaList>> {
        self.dma_list
    }
}
impl AsyncTransmitExt for WiFi<'_> {}
impl<'res> AsyncReceive<'res> for WiFi<'res> {}

impl Drop for WiFi<'_> {
    fn drop(&mut self) {
        // We don't have proper drop handling yet.
    }
}
