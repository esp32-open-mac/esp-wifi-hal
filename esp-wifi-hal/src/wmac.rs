use core::{
    cell::RefCell,
    mem::forget,
    ops::Deref,
    pin::{pin, Pin},
};
use portable_atomic::{AtomicU16, AtomicU64, AtomicU8, Ordering};

use crate::{
    esp_pac::wifi::TX_SLOT_CONFIG,
    ll::{self, RxFilterBank},
    rates::RATE_LUT,
    sync::{BorrowedTxSlot, TxSlotQueue, TxSlotStatus},
    CipherParameters, WiFiRate, INTERFACE_COUNT, KEY_SLOT_COUNT,
};
use embassy_sync::blocking_mutex::{self};
use embassy_time::Instant;
use esp_hal::{
    clock::{ModemClockController, PhyClockGuard},
    dma::{DmaDescriptor, Owner},
    interrupt::{bind_interrupt, enable, map, CpuInterrupt, Priority},
    peripherals::{Interrupt, ADC2, LPWR, WIFI},
    ram,
    system::Cpu,
};
use esp_wifi_sys::include::{
    esp_phy_calibration_data_t, esp_phy_calibration_mode_t_PHY_RF_CAL_FULL, register_chipv7_phy,
    wifi_pkt_rx_ctrl_t,
};
use macro_bits::{bit, check_bit};

use crate::{
    dma_list::DMAList,
    ffi::{disable_wifi_agc, enable_wifi_agc, hal_init, tx_pwctrl_background},
    phy_init_data::PHY_INIT_DATA_DEFAULT,
    sync::{SignalQueue, TxSlotStateSignal},
    DefaultRawMutex, WiFiResources,
};

static WIFI_RX_SIGNAL_QUEUE: SignalQueue = SignalQueue::new();

#[allow(clippy::declare_interior_mutable_const)]
const EMPTY_SLOT: TxSlotStateSignal = TxSlotStateSignal::new();
/// These are for knowing, when transmission has finished.
static WIFI_TX_SLOTS: [TxSlotStateSignal; 5] = [EMPTY_SLOT; 5];

// We run tx_pwctrl_background every four transmissions.
static FRAMES_SINCE_LAST_TXPWR_CTRL: AtomicU8 = AtomicU8::new(0);

#[ram]
extern "C" fn interrupt_handler() {
    // We don't want to have to steal this all the time.
    let wifi = WIFI::regs();

    let cause = wifi.mac_interrupt().wifi_int_status().read().bits();
    if cause == 0 {
        return;
    }
    #[cfg(pwr_interrupt_present)]
    {
        // We ignore the WIFI_PWR interrupt for now...
        let cause = wifi.pwr_interrupt().pwr_int_status().read().bits();
        wifi.pwr_interrupt()
            .pwr_int_clear()
            .write(|w| unsafe { w.bits(cause) });
    }
    wifi.mac_interrupt()
        .wifi_int_clear()
        .write(|w| unsafe { w.bits(cause) });
    if cause & 0x1000024 != 0 {
        WIFI_RX_SIGNAL_QUEUE.put();
    }
    if cause & 0x80 != 0 {
        unsafe {
            ll::process_tx_completions(|slot| {
                WIFI_TX_SLOTS[slot].signal(TxSlotStatus::Done);
            })
        };
    }
    if cause & 0x80000 != 0 {
        unsafe {
            ll::process_tx_timeouts(|slot| {
                WIFI_TX_SLOTS[slot].signal(TxSlotStatus::Timeout);
                ll::set_tx_slot_validity(slot, false);
                ll::set_tx_slot_enabled(slot, false);
            })
        };
    }
    if cause & 0x100 != 0 {
        unsafe {
            ll::process_tx_collisions(|slot| {
                WIFI_TX_SLOTS[slot].signal(TxSlotStatus::Collision);
                ll::set_tx_slot_validity(slot, false);
                ll::set_tx_slot_enabled(slot, false);
            })
        };
    }
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
    dma_list: &'res blocking_mutex::Mutex<DefaultRawMutex, RefCell<DMAList<'static>>>,
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
                defmt_or_log::panic!(
                    "Corrected len: {} Padded len: {} SIG len: {}",
                    self.unpadded_buffer_len(),
                    self.dma_descriptor.len(),
                    self.raw_header().sig_len()
                );
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
        u32::from_le_bytes(self.header_buffer()[12..16].try_into().unwrap())
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
    pub key_slot: Option<usize>,
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
    dma_list: &'res blocking_mutex::Mutex<DefaultRawMutex, RefCell<DMAList<'static>>>,
    current_channel: AtomicU8,
    sequence_number: AtomicU16,
    tx_slot_queue: TxSlotQueue,
    _phy_clock_guard: PhyClockGuard<'static>,
}
impl<'res> WiFi<'res> {
    /// Enable the Wi-Fi power domain.
    fn enable_wifi_power_domain() {
        unsafe {
            let rtc_cntl = &*LPWR::ptr();
            trace!("Enabling wifi power domain.");
            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.wifi_force_pd().clear_bit());

            rtc_cntl
                .dig_iso()
                .modify(|_, w| w.wifi_force_iso().clear_bit());
        }
    }
    fn enable_wifi_modem_clock() {
        unsafe { WIFI::steal() }.enable_modem_clock(true);
    }
    /// Enable the PHY.
    fn phy_enable() -> PhyClockGuard<'static> {
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
    /// Reset the MAC.
    fn reset_mac() {
        let mut wifi = unsafe { WIFI::steal() };
        trace!("Reseting MAC.");
        wifi.reset_wifi_mac();
        wifi.register_block()
            .ctrl()
            .modify(|r, w| unsafe { w.bits(r.bits() & 0x7fffffff) });
    }
    /// Initialize the MAC.
    unsafe fn init_mac() {
        trace!("Initializing MAC.");
        WIFI::regs()
            .ctrl()
            .modify(|r, w| unsafe { w.bits(r.bits() & 0xffffe800) });
    }
    /// Deinitialize the MAC.
    unsafe fn deinit_mac() {
        trace!("Deinitializing MAC.");
        WIFI::regs().ctrl().modify(|r, w| unsafe {
            w.bits(r.bits() | 0x17ff);
            while r.bits() & 0x2000 != 0 {}
            w
        });
    }
    /// Set the interrupt handler.
    fn set_isr() {
        trace!("Setting interrupt handler.");
        #[cfg(target_arch = "xtensa")]
        let cpu_interrupt = CpuInterrupt::Interrupt0LevelPriority1;
        #[cfg(target_arch = "riscv32")]
        let cpu_interrupt = CpuInterrupt::Interrupt1;
        unsafe {
            map(Cpu::current(), Interrupt::WIFI_MAC, cpu_interrupt);
            bind_interrupt(Interrupt::WIFI_MAC, interrupt_handler);
            #[cfg(pwr_interrupt_present)]
            {
                map(Cpu::current(), Interrupt::WIFI_PWR, cpu_interrupt);
                bind_interrupt(Interrupt::WIFI_PWR, interrupt_handler);
            }
        };
        enable(Interrupt::WIFI_MAC, Priority::Priority1).unwrap();
        #[cfg(pwr_interrupt_present)]
        enable(Interrupt::WIFI_PWR, Priority::Priority1).unwrap();
    }
    fn ic_enable() {
        trace!("ic_enable");
        unsafe {
            hal_init();
        }
        Self::set_isr();
    }
    fn crypto_init() {
        let wifi = unsafe { WIFI::steal() };
        // We enable hardware crypto for all interfaces.
        wifi.register_block()
            .crypto_control()
            .interface_crypto_control_iter()
            .for_each(|interface_crypto_control| {
                interface_crypto_control.write(|w| unsafe { w.bits(0x0003_0000) });
            });
        wifi.register_block()
            .crypto_control()
            .general_crypto_control()
            .reset();
    }
    /// Initialize the WiFi peripheral.
    pub fn new<const BUFFER_COUNT: usize>(
        _wifi: WIFI,
        _adc2: ADC2,
        wifi_resources: &'res mut WiFiResources<BUFFER_COUNT>,
    ) -> Self {
        trace!("Initializing WiFi.");
        Self::enable_wifi_power_domain();
        Self::enable_wifi_modem_clock();
        let phy_clock_guard = Self::phy_enable();
        let start_time = Instant::now();
        Self::reset_mac();
        unsafe {
            Self::init_mac();
        }
        Self::ic_enable();

        // This is technically already done in `hal_init`, but I want to replace that eventually,
        // so I'm trying to gradually move more and more parts out.
        Self::crypto_init();
        unsafe { ll::set_rx_enabled(true) };

        let temp = Self {
            current_channel: AtomicU8::new(1),
            dma_list: unsafe { wifi_resources.init() },
            sequence_number: AtomicU16::new(0),
            tx_slot_queue: TxSlotQueue::new(0..5),
            _phy_clock_guard: phy_clock_guard,
        };
        // System should always be ahead of MAC time, since the MAC timer gets started after the
        // System timer.
        MAC_SYSTEM_TIME_DELTA.store(
            esp_hal::time::Instant::now()
                .duration_since_epoch()
                .as_micros()
                - temp.mac_time() as u64,
            Ordering::Relaxed,
        );
        temp.set_channel(1).unwrap();
        trace!(
            "WiFi MAC init complete. Took {} µs",
            start_time.elapsed().as_micros()
        );
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
    /// Set the packet for transmission.
    async fn transmit_internal(
        &self,
        dma_list_item: Pin<&DmaDescriptor>,
        tx_parameters: &TxParameters,
        duration: u16,
        slot: &BorrowedTxSlot<'_>,
        ack_for_interface: Option<usize>,
    ) -> WiFiResult<()> {
        let length = dma_list_item.len();
        let reversed_slot = 4 - slot.deref();

        let wifi = WIFI::regs();

        let tx_slot_config = wifi.tx_slot_config(reversed_slot);
        tx_slot_config
            .config()
            .write(|w| unsafe { w.timeout().bits(tx_parameters.ack_timeout as u16) });

        tx_slot_config.plcp0().modify(|_, w| unsafe {
            w.dma_addr()
                .bits(dma_list_item.get_ref() as *const _ as u32)
                .wait_for_ack()
                .bit(ack_for_interface.is_some())
        });

        let rate = tx_parameters.rate;

        wifi.plcp1(reversed_slot).write(|w| unsafe {
            let w = if let Some(interface) = ack_for_interface {
                w.interface_id().bits(interface as u8)
            } else {
                w
            }
            .len()
            .bits(length as u16)
            .is_80211_n()
            .bit(rate.is_ht())
            .rate()
            .bits(rate as u8);
            if let Some(key_slot) = tx_parameters.key_slot {
                w.key_slot_id().bits(key_slot as u8)
            } else {
                w
            }
        });
        wifi.plcp2(reversed_slot).write(|w| w.unknown().bit(true));
        let duration = duration as u32;
        wifi.duration(reversed_slot)
            .write(|w| unsafe { w.bits(duration | (duration << 0x10)) });
        if rate.is_ht() {
            wifi.ht_sig(reversed_slot).write(|w| unsafe {
                w.bits(
                    (rate as u32 & 0b111)
                        | ((length as u32 & 0xffff) << 8)
                        | (0b111 << 24)
                        | ((rate.is_short_gi() as u32) << 31),
                )
            });
            wifi.ht_unknown(reversed_slot)
                .write(|w| unsafe { w.length().bits(length as u32 | 0x50000) });
        }
        // This also compensates for other slots being marked as done, without it being used at
        // all.
        WIFI_TX_SLOTS[**slot].reset();
        unsafe {
            ll::set_tx_slot_validity(**slot, true);
            ll::set_tx_slot_enabled(**slot, true);
        }

        // Since this is the first and only await point, all the transmit parameters will have been
        // set on the first poll. If the future gets dropped after the first poll, this would leave
        // the slot in an invalid state, so we use this construction to reset the slot in that
        // case.
        struct CancelOnDrop<'a, 'b> {
            tx_slot_config: &'a TX_SLOT_CONFIG,
            slot: &'a BorrowedTxSlot<'b>,
        }
        impl CancelOnDrop<'_, '_> {
            async fn wait_for_tx_complete(self) -> WiFiResult<()> {
                // Wait for the hardware to confirm transmission.
                let res = WIFI_TX_SLOTS[**self.slot].wait().await;
                // NOTE: This isn't done in the proprietary stack, but seems to prevent retransmissions.
                self.tx_slot_config.plcp0().reset();
                let res = match res {
                    TxSlotStatus::Done => {
                        if FRAMES_SINCE_LAST_TXPWR_CTRL.fetch_add(1, Ordering::Relaxed) == 4 {
                            unsafe { tx_pwctrl_background(1, 0) };
                            FRAMES_SINCE_LAST_TXPWR_CTRL.store(0, Ordering::Relaxed);
                        }
                        Ok(())
                    }
                    TxSlotStatus::Collision => Err(WiFiError::TxCollision),
                    TxSlotStatus::Timeout => Err(WiFiError::TxTimeout),
                };
                WIFI_TX_SLOTS[**self.slot].reset();
                forget(self);
                res
            }
        }
        impl Drop for CancelOnDrop<'_, '_> {
            fn drop(&mut self) {
                unsafe {
                    ll::set_tx_slot_validity(**self.slot, false);
                    ll::set_tx_slot_enabled(**self.slot, false);
                }
                WIFI_TX_SLOTS[**self.slot].reset();
            }
        }
        let cancel_on_drop = CancelOnDrop {
            tx_slot_config,
            slot,
        };
        cancel_on_drop.wait_for_tx_complete().await?;

        match wifi.pmd(reversed_slot).read().bits() >> 0xc {
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
                    &slot,
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
        unsafe {
            Self::deinit_mac();
            #[cfg(nomac_channel_set)]
            crate::ffi::chip_v7_set_chan_nomac(channel_number, 0);
            #[cfg(not(nomac_channel_set))]
            crate::ffi::chip_v7_set_chan(channel_number, 0);
            disable_wifi_agc();
            Self::init_mac();
            enable_wifi_agc();
        }
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
    /// Enable or disable the filter.
    pub fn set_filter_status(
        &self,
        bank: RxFilterBank,
        interface: usize,
        enabled: bool,
    ) -> WiFiResult<()> {
        Self::validate_interface(interface)?;
        WIFI::regs()
            .filter_bank(bank.into_bits())
            .mask_high(interface)
            .modify(|_, w| w.enabled().bit(enabled));
        Ok(())
    }
    /// Specifiy whether the BSSID is checked or not.
    ///
    /// This is used to control a special behaviour of the hardware. If the RA filter is enabled,
    /// but the BSSID is disabled, it will assume that `BSSID == RA`. However setting this to
    /// `false` will stop this behaviour.
    pub fn set_filter_bssid_check(&self, interface: usize, enabled: bool) -> WiFiResult<()> {
        Self::validate_interface(interface)?;
        WIFI::regs()
            .interface_rx_control(interface)
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
            .interface_rx_control(interface)
            .write(|w| unsafe { w.bits(val) });
        Ok(())
    }
    /// Read a raw value from the `INTERFACE_RX_CONTROL` register.
    ///
    /// NOTE: This exists mostly for debugging purposes, so if you find yourself using this for
    /// something else than that, you probably want to use one of the other functions.
    pub fn read_rx_policy_raw(&self, interface: usize) -> WiFiResult<u32> {
        Self::validate_interface(interface)?;
        Ok(WIFI::regs().interface_rx_control(interface).read().bits())
    }
    /// Set the parameters for the filter.
    pub fn set_filter(
        &self,
        bank: RxFilterBank,
        interface: usize,
        address: [u8; 6],
        mask: [u8; 6],
    ) -> WiFiResult<()> {
        Self::validate_interface(interface)?;
        let wifi = WIFI::regs();
        let bank = wifi.filter_bank(bank.into_bits());
        bank.addr_low(interface)
            .write(|w| unsafe { w.bits(u32::from_le_bytes(address[..4].try_into().unwrap())) });
        bank.addr_high(interface).write(|w| unsafe {
            w.addr()
                .bits(u16::from_le_bytes(address[4..].try_into().unwrap()))
        });
        bank.mask_low(interface)
            .write(|w| unsafe { w.bits(u32::from_le_bytes(mask[..4].try_into().unwrap())) });
        bank.mask_high(interface).write(|w| unsafe {
            w.mask()
                .bits(u16::from_le_bytes(mask[4..].try_into().unwrap()))
        });
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
            .interface_rx_control(interface)
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
        Self::validate_key_slot(key_slot).map(|_| {
            WIFI::regs()
                .crypto_control()
                .crypto_key_slot_state()
                .read()
                .key_slot_enable(key_slot as u8)
                .bit()
        })
    }
    /// Get an iterator over the current state of all key slots.
    ///
    /// If the value returned by [Iterator::next] is `true`, the key slot is in use.
    /// NOTE: The returned iterator only reflects the state, when the function was called. If you
    /// call either [WiFi::set_key] or [WiFi::delete_key], between calling this function and
    /// [Iterator::next], the result may no longer be accurate.
    pub fn key_slot_state_iter(&self) -> impl Iterator<Item = bool> {
        let key_slot_state = WIFI::regs()
            .crypto_control()
            .crypto_key_slot_state()
            .read()
            .bits();
        (0..KEY_SLOT_COUNT).map(move |i| check_bit!(key_slot_state, bit!(i)))
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
        assert!(key.len() <= 32);

        Self::validate_key_slot(key_slot)?;
        Self::validate_interface(interface)?;
        if key_id >= 4 {
            return Err(WiFiError::KeyIdOutOfBounds);
        }

        if check_bit!(address[0], bit!(0)) {
            return Err(WiFiError::MulticastBitSet);
        }

        let wifi = WIFI::regs();

        let crypto_key_slot = wifi.crypto_key_slot(key_slot);

        crypto_key_slot
            .addr_low()
            .write(|w| unsafe { w.bits(u32::from_le_bytes(address[..4].try_into().unwrap())) });
        crypto_key_slot.addr_high().write(|w| unsafe {
            w.addr()
                .bits(u16::from_le_bytes(address[4..].try_into().unwrap()))
                .algorithm()
                .bits(cipher_parameters.algorithm())
                .wep_104()
                .bit(cipher_parameters.is_wep_104())
                .bits_256()
                .bit(cipher_parameters.is_256_bit_key())
                .pairwise_key()
                .bit(cipher_parameters.is_pairwise())
                .group_key()
                .bit(cipher_parameters.is_group())
                .unknown()
                .set_bit()
                .interface_id()
                .bits(interface as u8)
                .key_id()
                .bits(key_id)
        });

        wifi.crypto_control()
            .crypto_key_slot_state()
            .modify(|_, w| w.key_slot_enable(key_slot as u8).set_bit());

        let is_key_len_word_aligned = (key.len() & 0b11) == 0;

        // Copy the key into the slot.
        for (i, key_chunk) in key.chunks(4).enumerate().take(8) {
            let chunk_len = key_chunk.len();
            assert!(chunk_len <= 4);

            let key_chunk = if !is_key_len_word_aligned && chunk_len != 4 {
                let mut temp = [0u8; 4];
                temp[..chunk_len].copy_from_slice(key_chunk);
                temp
            } else {
                key_chunk.try_into().unwrap()
            };
            crypto_key_slot
                .key_value(i)
                .write(|w| unsafe { w.bits(u32::from_le_bytes(key_chunk)) });
        }
        wifi.crypto_control()
            .general_crypto_control()
            .modify(|r, w| unsafe { w.bits(r.bits() & 0xff0000ff) });
        wifi.crypto_control()
            .interface_crypto_control(interface)
            .modify(|_, w| {
                w.spp_enable()
                    .bit(cipher_parameters.is_spp_enabled())
                    .pmf_disable()
                    .bit(!cipher_parameters.is_mfp_enabled())
                    .sms4()
                    .clear_bit()
                    .aead_cipher()
                    .bit(cipher_parameters.is_aead())
            });
        wifi.crypto_control()
            .interface_crypto_control(interface)
            .modify(|r, w| unsafe { w.bits(r.bits() | 0x10103 & 0x3fff_ffff) });

        Ok(())
    }
    /// Delete a cryptographic key.
    ///
    /// This is equivalent to MLME-DELETEKEYS.request with one key descriptor.
    pub fn delete_key(&self, key_slot: usize) -> WiFiResult<()> {
        WiFi::validate_key_slot(key_slot)?;

        let wifi = WIFI::regs();
        wifi.crypto_control()
            .crypto_key_slot_state()
            .modify(|_, w| w.key_slot_enable(key_slot as u8).clear_bit());

        let crypto_key_slot = wifi.crypto_key_slot(key_slot);
        crypto_key_slot.addr_low().reset();
        crypto_key_slot.addr_high().reset();
        crypto_key_slot
            .key_value_iter()
            .for_each(|key_value| key_value.reset());

        Ok(())
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
}
impl Drop for WiFi<'_> {
    fn drop(&mut self) {
        // We don't have proper drop handling yet.
    }
}
