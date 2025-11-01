use core::{
    cell::RefCell,
    pin::{pin, Pin},
};
use portable_atomic::{AtomicU16, AtomicU64, AtomicU8, Ordering};

use crate::{
    ll::{
        EdcaAccessCategory, HardwareTxQueue, HardwareTxResult, KeySlotParameters, LowLevelDriver, RxFilterBank, TxSlotStatus, WiFiInterrupt
    },
    rates::RATE_LUT,
    sync::{DropGuard, TxSlotQueue},
    CipherParameters, WiFiRate, INTERFACE_COUNT, KEY_SLOT_COUNT,
};
use embassy_sync::blocking_mutex::{self};
use esp_hal::{
    dma::{DmaDescriptor, Owner},
    handler,
    interrupt::CpuInterrupt,
    peripherals::{ADC2, WIFI},
};
use esp_wifi_sys::include::wifi_pkt_rx_ctrl_t;
use macro_bits::{bit, check_bit};

use crate::{
    dma_list::DMAList,
    sync::{HardwareTxResultSignal, SignalQueue},
    DefaultRawMutex, WiFiResources,
};

static WIFI_RX_SIGNAL_QUEUE: SignalQueue = SignalQueue::new();

/// These are for knowing, when transmission has finished, and with which result.
static HARDWARE_TX_RESULT_SIGNALS: [HardwareTxResultSignal; 5] =
    [const { HardwareTxResultSignal::new() }; 5];

// We run tx_pwctrl_background every four transmissions.
static FRAMES_SINCE_LAST_TXPWR_CTRL: AtomicU8 = AtomicU8::new(0);

#[handler]
fn mac_handler() {
    let cause = unsafe { LowLevelDriver::get_and_clear_mac_interrupt_cause() };
    if cause.is_emtpy() {
        return;
    }

    if cause.rx() {
        WIFI_RX_SIGNAL_QUEUE.put();
    }
    if cause.tx_success() {
        unsafe {
            LowLevelDriver::process_tx_status(HardwareTxResult::Success, |queue| {
                HARDWARE_TX_RESULT_SIGNALS[queue.hardware_slot()].signal(HardwareTxResult::Success);
            })
        };
    }
    if cause.tx_timeout() {
        unsafe {
            LowLevelDriver::process_tx_status(HardwareTxResult::Timeout, |queue| {
                HARDWARE_TX_RESULT_SIGNALS[queue.hardware_slot()].signal(HardwareTxResult::Timeout);
                LowLevelDriver::set_tx_queue_status(queue, TxSlotStatus::Disabled);
            })
        };
    }
    if cause.tx_collision() {
        unsafe {
            LowLevelDriver::process_tx_status(HardwareTxResult::Collision, |queue| {
                HARDWARE_TX_RESULT_SIGNALS[queue.hardware_slot()]
                    .signal(HardwareTxResult::Collision);
                LowLevelDriver::set_tx_queue_status(queue, TxSlotStatus::Disabled);
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

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
/// What should be done, if an error occurs, while transmitting.
pub enum TxErrorBehaviour {
    /// Retry as many times, as specified.
    RetryUntil(usize),
    /// Drop the MPDU.
    #[default]
    Drop,
}

/// A buffer borrowed from the DMA list.
pub struct BorrowedBuffer<'res> {
    dma_list: &'res blocking_mutex::Mutex<DefaultRawMutex, RefCell<DMAList>>,
    dma_descriptor: &'res mut DmaDescriptor,
}
impl BorrowedBuffer<'_> {
    /// The length of the hardware header.
    pub const RX_CONTROL_HEADER_LENGTH: usize = size_of::<wifi_pkt_rx_ctrl_t>();

    /// This returns the raw buffer, which is still padded with zeros to the nearest word boundary.
    /// Apparently the Wi-Fi MAC only does transfers with word aligned lengths, so the remaining
    /// bytes are filled with zeros. This buffer contains the RX control header and MPDU without
    /// FCS or MIC.
    pub fn padded_buffer(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(self.dma_descriptor.buffer, self.dma_descriptor.len())
        }
    }
    /// See [Self::padded_buffer] for docs.
    pub fn padded_buffer_mut(&mut self) -> &mut [u8] {
        unsafe {
            core::slice::from_raw_parts_mut(self.dma_descriptor.buffer, self.dma_descriptor.len())
        }
    }
    /// Get a pointer to the RX control header.
    pub fn raw_header(&self) -> &wifi_pkt_rx_ctrl_t {
        unsafe {
            (self.dma_descriptor.buffer as *mut wifi_pkt_rx_ctrl_t)
                .as_ref()
                .unwrap()
        }
    }
    /// Calculate the length of the trailer.
    ///
    /// This includes MIC and FCS, which are conveniently all a multiple of 32 bits in length.
    /// And all of that, since it apparently wasn't possible to put the correct length in the DMA
    /// descriptor...
    /// Frostie314159: This caused me such a fucking headache.
    pub fn trailer_length(&self) -> usize {
        let padded_len = self.dma_descriptor.len() - Self::RX_CONTROL_HEADER_LENGTH;
        let delta = self.raw_header().sig_len() as usize - padded_len;
        delta.div_ceil(4) * 4
    }
    /// Calculate the actual unpadded length of the buffer.
    pub fn unpadded_buffer_len(&self) -> usize {
        self.raw_header().sig_len() as usize - self.trailer_length()
            + Self::RX_CONTROL_HEADER_LENGTH
    }
    /// Returns the raw buffer returned by the hardware.
    ///
    /// This includes the header added by the hardware.
    pub fn raw_buffer(&self) -> &[u8] {
        self.padded_buffer()
            .get(..self.unpadded_buffer_len())
            .unwrap_or_else(|| {
                defmt_or_log::warn!(
                    "Buffer was shorter then calculated length. Corrected len: {} Padded len: {} SIG len: {} Header: {:02x?}",
                    self.unpadded_buffer_len(),
                    self.dma_descriptor.len(),
                    self.raw_header().sig_len(),
                    &self.padded_buffer()[..Self::RX_CONTROL_HEADER_LENGTH]
                );
                self.dma_list.lock(|dma_list| dma_list.borrow().log_stats());
                self.padded_buffer()
            })
    }
    /// Same as [Self::raw_buffer], but mutable.
    pub fn raw_buffer_mut(&mut self) -> &mut [u8] {
        let unpadded_len = self.unpadded_buffer_len();
        &mut self.padded_buffer_mut()[..unpadded_len]
    }
    /// Returns the actual MPDU from the buffer excluding the prepended RX control header.
    pub fn mpdu_buffer(&self) -> &[u8] {
        &self.raw_buffer()[Self::RX_CONTROL_HEADER_LENGTH..]
    }
    /// Same as [Self::mpdu_buffer], but mutable.
    pub fn mpdu_buffer_mut(&mut self) -> &mut [u8] {
        &mut self.raw_buffer_mut()[Self::RX_CONTROL_HEADER_LENGTH..]
    }
    /// Returns the header attached by the hardware.
    pub fn header_buffer(&self) -> &[u8] {
        &self.padded_buffer()[..Self::RX_CONTROL_HEADER_LENGTH]
    }
    /// Same as [Self::header_buffer], but mutable.
    pub fn header_buffer_mut(&mut self) -> &mut [u8] {
        &mut self.padded_buffer_mut()[..Self::RX_CONTROL_HEADER_LENGTH]
    }
    /// The Received Signal Strength Indicator (RSSI).
    pub fn rssi(&self) -> i8 {
        let rssi = self.header_buffer()[0] as i8;
        if cfg!(feature = "esp32") {
            rssi - 96
        } else {
            rssi
        }
    }
    /// The time at which the packet was received in µs.
    pub fn timestamp(&self) -> u32 {
        self.raw_header().timestamp()
    }
    /// The time the packet was received, corrected for the difference between MAC and system
    /// clock.
    pub fn corrected_timestamp(&self) -> embassy_time::Instant {
        embassy_time::Instant::from_micros(
            self.timestamp() as u64 + MAC_SYSTEM_TIME_DELTA.load(Ordering::Relaxed),
        )
    }
    /// Check if the frame is an A-MPDU.
    pub fn aggregation(&self) -> bool {
        check_bit!(self.header_buffer()[7], bit!(3))
    }
    /// The phy rate, at which this frame was transmitted.
    pub fn phy_rate(&self) -> Option<WiFiRate> {
        let rate_and_sig_mode = self.header_buffer()[1];
        let (rate, sig_mode) = (rate_and_sig_mode & 0x1f, rate_and_sig_mode >> 6);
        let rate_idx = if sig_mode == 1 {
            0x10 + (self.header_buffer()[4] & 0x7f)
                + if self.header_buffer()[7] >> 7 == 1 {
                    8 // Add eight for short GI rates.
                } else {
                    0
                }
        } else {
            rate
        };
        RATE_LUT.get(rate_idx as usize).copied()
    }
    /// Check if the frame is for the specified interface.
    pub fn is_frame_for_interface(&self, interface: usize) -> bool {
        WiFi::validate_interface(interface).is_ok()
            && check_bit!(self.header_buffer()[3], bit!(interface + 4))
    }
    /// Get an iterator over the interfaces, to which this frame is addressed.
    pub fn interface_iterator(&self) -> impl Iterator<Item = usize> + '_ {
        let byte = self.header_buffer()[3];
        (0..INTERFACE_COUNT).filter(move |interface| check_bit!(byte, bit!(interface + 4)))
    }
    /// Unknown RX state.
    ///
    /// We don't really know what this means, but it is presumably a bitmap. Currently this is only
    /// intended for debugging purposes.
    pub fn rx_state(&self) -> u8 {
        self.header_buffer()[Self::RX_CONTROL_HEADER_LENGTH - 1]
    }
}
impl Drop for BorrowedBuffer<'_> {
    fn drop(&mut self) {
        self.dma_list.lock(|dma_list| {
            dma_list.borrow_mut().recycle(self.dma_descriptor);
        });
    }
}
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
/// Errors returned by the Wi-Fi driver.
pub enum WiFiError {
    /// The provided channel was invalid.
    InvalidChannel,
    /// The provided interface index was out of bounds.
    InterfaceOutOfBounds,
    /// A timeout occured during transmission.
    TxTimeout,
    /// A collision occured during transmission.
    ///
    /// NOTE: The meaning of this isn't fully understood yet.
    TxCollision,
    /// No ACK was received within the specified timeout.
    AckTimeout,
    /// No CTS was received within the specified timeout.
    CtsTimeout,
    /// No RTS was received within the specified timeout.
    RtsTimeout,
    /// The provided buffer is to short.
    BufferTooShort,
    /// The provided key slot was out of bounds.
    KeySlotOutOfBounds,
    /// The multicast bit was set for the address provided with the key.
    MulticastBitSet,
    /// The provided key ID was larger than three.
    KeyIdOutOfBounds,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
/// Parameters for the transmission of an MPDU.
pub struct TxParameters {
    /// The rate at which to tranmsit the packet.
    pub rate: WiFiRate,
    /// Override the sequence number, with the one maintained by the driver.
    ///
    /// This is recommended.
    pub override_seq_num: bool,
    /// What to do in case an error occurs.
    pub tx_error_behaviour: TxErrorBehaviour,
    /// The maximum amount of time an ACK can take to arrive.
    pub ack_timeout: usize,
    /// The key slot to be used for encryption.
    pub key_slot: Option<u8>,
}
impl Default for TxParameters {
    fn default() -> Self {
        Self {
            rate: WiFiRate::default(),
            override_seq_num: false,
            tx_error_behaviour: TxErrorBehaviour::Drop,
            ack_timeout: 10,
            key_slot: None,
        }
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

/// A [Result] returned by the Wi-Fi driver.
pub type WiFiResult<T> = Result<T, WiFiError>;

/// The time the MAC clock was enabled in microseconds.
static MAC_SYSTEM_TIME_DELTA: AtomicU64 = AtomicU64::new(0);

/// Driver for the Wi-Fi peripheral.
///
/// WARNING: Currently dropping the driver is not properly implemented.
pub struct WiFi<'res> {
    dma_list: &'res blocking_mutex::Mutex<DefaultRawMutex, RefCell<DMAList>>,
    ll_driver: &'res LowLevelDriver,
    current_channel: AtomicU8,
    sequence_number: AtomicU16,
    tx_slot_queue: TxSlotQueue,
}
impl<'res> WiFi<'res> {
    /// Initialize the WiFi peripheral.
    pub fn new<const BUFFER_COUNT: usize>(
        wifi: WIFI<'res>,
        _adc2: ADC2,
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

        let (dma_list, ll_driver) = unsafe { wifi_resources.init(ll_driver) };

        let temp = Self {
            current_channel: AtomicU8::new(1),
            ll_driver,
            dma_list,
            sequence_number: AtomicU16::new(0),
            tx_slot_queue: TxSlotQueue::new(0..5),
        };
        temp.set_channel(1).unwrap();
        temp
    }
    /// Clear all currently pending frames in the RX queue.
    pub fn clear_rx_queue(&self) {
        self.dma_list
            .lock(|rx_dma_list| rx_dma_list.borrow_mut().clear());
        WIFI_RX_SIGNAL_QUEUE.reset();
    }
    /// Receive a frame.
    ///
    /// NOTE: The received frame will not contain an FCS or MIC.
    pub async fn receive(&self) -> BorrowedBuffer<'res> {
        // Sometimes the DMA list descriptors don't contain any data, even though the hardware indicated reception.
        // We loop until we get something.
        let dma_list_item = loop {
            WIFI_RX_SIGNAL_QUEUE.next().await;
            if let Some(current) = self
                .dma_list
                .lock(|dma_list| dma_list.borrow_mut().take_first())
            {
                if current.len() >= BorrowedBuffer::RX_CONTROL_HEADER_LENGTH {
                    trace!("Received packet. len: {}", current.len());
                    break current;
                }
            }
            trace!("Received empty packet.");
        };

        BorrowedBuffer {
            dma_list: self.dma_list,
            dma_descriptor: dma_list_item,
        }
    }
    /// Wait for TX to complete on a queue.
    async fn wait_tx_done(&self, queue: HardwareTxQueue) -> WiFiResult<()> {
        match HARDWARE_TX_RESULT_SIGNALS[queue.hardware_slot()]
            .wait()
            .await
        {
            HardwareTxResult::Success => Ok(()),
            HardwareTxResult::Timeout => Err(WiFiError::TxTimeout),
            HardwareTxResult::Collision => Err(WiFiError::TxCollision),
        }
    }
    /// Set the packet for transmission.
    async fn transmit_internal(
        &self,
        dma_list_item: Pin<&DmaDescriptor>,
        tx_parameters: &TxParameters,
        duration: u16,
        queue: HardwareTxQueue,
        ack_for_interface: Option<usize>,
    ) -> WiFiResult<()> {
        let wifi = WIFI::regs();

        let hardware_tx_result_signal = &HARDWARE_TX_RESULT_SIGNALS[queue.hardware_slot()];

        self.ll_driver.set_channel_access_parameters(queue, tx_parameters.ack_timeout, 0, 0);
        self.ll_driver
            .set_plcp0(queue, dma_list_item, ack_for_interface.is_some());
        self.ll_driver.set_plcp1(
            queue,
            tx_parameters.rate,
            dma_list_item.len(),
            ack_for_interface.unwrap_or_default(),
            tx_parameters.key_slot,
        );
        self.ll_driver.set_plcp2(queue);
        self.ll_driver.set_duration(queue, duration);

        if let Some((mcs, is_short_gi)) = tx_parameters.rate.ht_paramters() {
            self.ll_driver
                .set_ht_parameters(queue, mcs, is_short_gi, dma_list_item.len());
        }
        // This compensates for other slots being marked as done, without it being used at all.
        hardware_tx_result_signal.reset();
        self.ll_driver.start_tx_queue(queue);

        // This will reset the slot, once the drop guard goes out of scope, or the future is
        // dropped.
        let tx_done_wait_drop_guard = DropGuard::new(|| {
            self.ll_driver.tx_done(queue);
            hardware_tx_result_signal.reset();
        });
        // We wait for the transmission to complete here.
        self.wait_tx_done(queue).await.inspect(|_| {
            if FRAMES_SINCE_LAST_TXPWR_CTRL.fetch_add(1, Ordering::Relaxed) == 4 {
                self.ll_driver.run_power_control();
                FRAMES_SINCE_LAST_TXPWR_CTRL.store(0, Ordering::Relaxed);
            }
        })?;
        tx_done_wait_drop_guard.defuse();

        match wifi.pmd(queue.hardware_slot()).read().bits() >> 0xc {
            1 => Err(WiFiError::RtsTimeout),
            2 => Err(WiFiError::CtsTimeout),
            5 => Err(WiFiError::AckTimeout),
            _ => Ok(()),
        }
    }
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
    pub async fn transmit(
        &self,
        buffer: &mut [u8],
        tx_parameters: &TxParameters,
        ack_for_interface: Option<usize>,
    ) -> WiFiResult<usize> {
        self.transmit_with_hook(buffer, tx_parameters, ack_for_interface, |_| {})
            .await
    }
    /// Transmit a frame and execute the provided hook before transmission.
    ///
    /// This is the same as [transmit](Self::transmit), but the provided closure will be executed
    /// right before the frame is set for transmission. This is useful for timing critical
    /// protocols, since waiting for a TX slot takes time, which can cause data to become out
    /// dated. The hook may be called multiple times, if a frame is retransmitted.
    pub async fn transmit_with_hook(
        &self,
        buffer: &mut [u8],
        tx_parameters: &TxParameters,
        ack_for_interface: Option<usize>,
        mut pre_transmit_hook: impl FnMut(&mut [u8]),
    ) -> WiFiResult<usize> {
        let slot = self.tx_slot_queue.wait_for_slot().await;
        trace!("Acquired slot {}.", *slot);

        let Some(duration) = buffer
            .get(2..4)
            .map(|bytes| u16::from_le_bytes(bytes.try_into().unwrap()))
        else {
            return Err(WiFiError::BufferTooShort);
        };

        if tx_parameters.override_seq_num {
            let seq_num = self.sequence_number.load(Ordering::Relaxed).wrapping_add(1);
            self.sequence_number.store(seq_num, Ordering::Relaxed);
            if let Some(sequence_number) = buffer.get_mut(22..24) {
                sequence_number.copy_from_slice((seq_num << 4).to_le_bytes().as_slice());
            } else {
                return Err(WiFiError::BufferTooShort);
            }
        }

        // We initialize and pin the DMA descriptor.
        let length = buffer.len() + 4;
        let mut dma_descriptor: Pin<&mut DmaDescriptor> = pin!(DmaDescriptor::EMPTY);
        dma_descriptor.set_owner(Owner::Dma);
        dma_descriptor.set_size(length);
        dma_descriptor.set_length(length);
        dma_descriptor.set_suc_eof(true);
        dma_descriptor.buffer = buffer.as_ptr() as *mut u8;

        let dma_descriptor_ref = dma_descriptor.into_ref();

        let tx_attempts =
            if let TxErrorBehaviour::RetryUntil(retries) = tx_parameters.tx_error_behaviour {
                retries + 1
            } else {
                1
            };
        let mut res = Ok(());

        // Begin TX attempts.
        for i in 0..tx_attempts {
            // Execute the pre transmit hook.
            (pre_transmit_hook)(buffer);

            // Attempt transmission.
            res = self
                .transmit_internal(
                    dma_descriptor_ref,
                    tx_parameters,
                    duration,
                    HardwareTxQueue::from_hardware_slot(4 - *slot).unwrap(),
                    ack_for_interface,
                )
                .await;
            if tx_parameters.tx_error_behaviour == TxErrorBehaviour::Drop {
                break;
            }

            match res {
                Ok(_) => return Ok(i),
                Err(err @ (WiFiError::TxTimeout | WiFiError::TxCollision)) => {
                    trace!(
                        "Retransmitting MPDU due to an error while transmitting: {:?}.",
                        err
                    );
                }
                Err(err) => {
                    if let Some(byte) = buffer.get_mut(1) {
                        *byte |= bit!(3);
                    }
                    trace!(
                        "Retransmitting MPDU due to a MAC protocol timeout: {:?}",
                        err
                    );
                }
            }
        }
        if let Some(byte) = buffer.get_mut(1) {
            *byte &= !bit!(3);
        }
        // Return the last result with the amount of transmissions.
        res.map(|_| tx_attempts)
    }
    /// Set the channel on which to operate.
    ///
    /// NOTE:
    /// This uses the proprietary blob.
    /// It also accepts channel 14, which is only allowed in Japan with the DSSS PHY. However, the
    /// word "allowed" is used very deliberately here. Make of that what you will...
    pub fn set_channel(&self, channel_number: u8) -> WiFiResult<()> {
        if !(1..=14).contains(&channel_number) {
            return Err(WiFiError::InvalidChannel);
        }
        trace!("Changing channel to {}.", channel_number);
        self.ll_driver.set_channel(channel_number);
        self.current_channel
            .store(channel_number, Ordering::Relaxed);
        Ok(())
    }
    /// Returns the current channel.
    pub fn get_channel(&self) -> u8 {
        self.current_channel.load(Ordering::Relaxed)
    }
    /// Get the current MAC time in µs.
    pub fn mac_time(&self) -> u32 {
        // We hardcode the addresses here, until PAC support is merged.
        WIFI::regs().mac_time().read().bits()
    }
    /// Check if that interface is valid.
    pub const fn validate_interface(interface: usize) -> WiFiResult<()> {
        if interface < INTERFACE_COUNT {
            Ok(())
        } else {
            Err(WiFiError::InterfaceOutOfBounds)
        }
    }
    /// Specifiy whether the BSSID is checked or not.
    ///
    /// This is used to control a special behaviour of the hardware. If the RA filter is enabled,
    /// but the BSSID is disabled, it will assume that `BSSID == RA`. However setting this to
    /// `false` will stop this behaviour.
    pub fn set_filter_bssid_check(&self, interface: usize, enabled: bool) -> WiFiResult<()> {
        Self::validate_interface(interface)?;
        WIFI::regs()
            .filter_control(interface)
            .modify(|_, w| w.bssid_check().bit(enabled));
        Ok(())
    }
    /// Write raw value to the `INTERFACE_RX_CONTROL` register.
    ///
    /// NOTE: This exists mostly for debugging purposes, so if you find yourself using this for
    /// something else than that, you probably want to use one of the other functions.
    pub fn write_rx_policy_raw(&self, interface: usize, val: u32) -> WiFiResult<()> {
        Self::validate_interface(interface)?;
        WIFI::regs()
            .filter_control(interface)
            .write(|w| unsafe { w.bits(val) });
        Ok(())
    }
    /// Read a raw value from the `INTERFACE_RX_CONTROL` register.
    ///
    /// NOTE: This exists mostly for debugging purposes, so if you find yourself using this for
    /// something else than that, you probably want to use one of the other functions.
    pub fn read_rx_policy_raw(&self, interface: usize) -> WiFiResult<u32> {
        Self::validate_interface(interface)?;
        Ok(WIFI::regs().filter_control(interface).read().bits())
    }
    /// Configure the filter.
    pub fn set_filter(
        &self,
        filter_bank: RxFilterBank,
        interface: usize,
        address: [u8; 6],
    ) -> WiFiResult<()> {
        Self::validate_interface(interface)?;

        self.ll_driver
            .set_filter_address(interface, filter_bank, &address);
        self.ll_driver
            .set_filter_mask(interface, filter_bank, &[0xff; 6]);
        self.ll_driver
            .set_filter_enable(interface, filter_bank, true);

        Ok(())
    }
    /// Enable or disable scanning mode.
    pub fn set_scanning_mode(
        &self,
        interface: usize,
        scanning_mode: ScanningMode,
    ) -> WiFiResult<()> {
        Self::validate_interface(interface)?;
        WIFI::regs()
            .filter_control(interface)
            .modify(|_, w| match scanning_mode {
                ScanningMode::Disabled => {
                    w.scan_mode().clear_bit().data_and_mgmt_mode().clear_bit()
                }
                ScanningMode::BeaconsOnly => {
                    w.scan_mode().set_bit().data_and_mgmt_mode().clear_bit()
                }
                ScanningMode::ManagementAndData => {
                    w.scan_mode().clear_bit().data_and_mgmt_mode().set_bit()
                }
            });
        Ok(())
    }
    /// Check if they key slot is valid.
    pub const fn validate_key_slot(key_slot: usize) -> WiFiResult<()> {
        if key_slot < KEY_SLOT_COUNT {
            Ok(())
        } else {
            Err(WiFiError::KeySlotOutOfBounds)
        }
    }
    /// Check if the key slot is active.
    ///
    /// If `key_slot` is larger than or equal to [WiFi::KEY_SLOT_COUNT], an error will be returned.
    pub fn key_slot_in_use(&self, key_slot: usize) -> WiFiResult<bool> {
        Self::validate_key_slot(key_slot).map(|_| self.ll_driver.key_slot_enabled(key_slot))
    }
    /// Set a cryptographic key.
    ///
    /// This is equivalent to MLME-SETKEYS.request with one key descriptor.
    /// An error will be returned, if the key slot, key ID or interface are out of bounds, or the
    /// address is multicast. If the key slot is currently in use, this will overwrite the current
    /// entry.
    pub fn set_key(
        &self,
        key_slot: usize,
        interface: usize,
        key_id: u8,
        address: [u8; 6],
        cipher_parameters: CipherParameters<'_>,
    ) -> WiFiResult<()> {
        // If anyone wonders about all of the asserts here, I got bored and optimized the shit out
        // of this function. Sadly the compiler still fails to prove some other stuff.

        let key = cipher_parameters.key();

        Self::validate_key_slot(key_slot)?;
        Self::validate_interface(interface)?;
        if key_id >= 4 {
            return Err(WiFiError::KeyIdOutOfBounds);
        }

        if check_bit!(address[0], bit!(0)) {
            return Err(WiFiError::MulticastBitSet);
        }

        self.ll_driver.set_key_slot_parameters(
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
        self.ll_driver.set_key(key_slot, key);
        self.ll_driver.set_key_slot_enable(key_slot, true);

        let (protect_management_frames, protect_signaling_and_payload) =
            if let Some(aes_cipher_parameters) = cipher_parameters.aes_cipher_parameters() {
                (
                    aes_cipher_parameters.mfp_enabled,
                    aes_cipher_parameters.spp_enabled,
                )
            } else {
                (false, false)
            };
        self.ll_driver.set_interface_crypto_parameters(
            interface,
            protect_management_frames,
            protect_signaling_and_payload,
            cipher_parameters.is_aead(),
        );

        Ok(())
    }
    /// Delete a cryptographic key.
    ///
    /// This is equivalent to MLME-DELETEKEYS.request with one key descriptor.
    pub fn delete_key(&self, key_slot: usize) -> WiFiResult<()> {
        WiFi::validate_key_slot(key_slot).map(|_| {
            self.ll_driver.set_key_slot_enable(key_slot, false);
            self.ll_driver.clear_key_slot(key_slot);
        })
    }
    /// Dump the entire contents of a key slot.
    pub fn dump_key_slot(&self, key_slot: usize) -> WiFiResult<()> {
        Self::validate_key_slot(key_slot)?;

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
    pub fn dump_crypto_config(&self) {
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
    /// Log stats about the DMA list.
    pub fn log_dma_list_stats(&self) {
        self.dma_list.lock(|dma_list| dma_list.borrow().log_stats())
    }
}
impl Drop for WiFi<'_> {
    fn drop(&mut self) {
        // We don't have proper drop handling yet.
    }
}
