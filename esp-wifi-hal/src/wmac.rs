use core::{
    cell::RefCell,
    mem::forget,
    ops::{Deref, Range},
    pin::{pin, Pin},
};
use portable_atomic::{AtomicU16, AtomicU8, Ordering};

use crate::{esp_pac::wifi::TX_SLOT_CONFIG, sync::TxSlotStatus};
use embassy_sync::{
    blocking_mutex::{self},
    channel::{Channel, DynamicSender},
};
use embassy_time::Instant;
use esp_hal::{
    interrupt::{bind_interrupt, enable, map, CpuInterrupt, Priority},
    peripherals::{Interrupt, ADC2, LPWR, RADIO_CLK, WIFI},
    ram,
    system::{RadioClockController, RadioPeripherals},
    Cpu,
};
use esp_wifi_sys::include::{
    esp_phy_calibration_data_t, esp_phy_calibration_mode_t_PHY_RF_CAL_FULL, register_chipv7_phy,
    wifi_pkt_rx_ctrl_t,
};
use macro_bits::{bit, check_bit, serializable_enum};

use crate::{
    dma_list::{DMAList, DMAListItem, RxDMAListItem, TxDMAListItem},
    ffi::{disable_wifi_agc, enable_wifi_agc, hal_init, tx_pwctrl_background},
    phy_init_data::PHY_INIT_DATA_DEFAULT,
    sync::{SignalQueue, TxSlotStateSignal},
    DMAResources, DefaultRawMutex,
};

serializable_enum! {
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
    /// The rate used by the PHY.
    pub enum WiFiRate: u8 {
        #[default]
        PhyRate1ML => 0x00,
        PhyRate2ML => 0x01,
        PhyRate5ML => 0x02,
        PhyRate11ML => 0x03,
        PhyRate2MS => 0x05,
        PhyRate5MS => 0x06,
        PhyRate11MS => 0x07,
        PhyRate48M => 0x08,
        PhyRate24M => 0x09,
        PhyRate12M => 0x0a,
        PhyRate6M => 0x0b,
        PhyRate54M => 0x0c,
        PhyRate36M => 0x0d,
        PhyRate18M => 0x0e,
        PhyRate9M => 0x0f,
        PhyRateMCS0LGI => 0x10,
        PhyRateMCS1LGI => 0x11,
        PhyRateMCS2LGI => 0x12,
        PhyRateMCS3LGI => 0x13,
        PhyRateMCS4LGI => 0x14,
        PhyRateMCS5LGI => 0x15,
        PhyRateMCS6LGI => 0x16,
        PhyRateMCS7LGI => 0x17,
        PhyRateMCS0SGI => 0x18,
        PhyRateMCS1SGI => 0x19,
        PhyRateMCS2SGI => 0x1a,
        PhyRateMCS3SGI => 0x1b,
        PhyRateMCS4SGI => 0x1c,
        PhyRateMCS5SGI => 0x1d,
        PhyRateMCS6SGI => 0x1e,
        PhyRateMCS7SGI => 0x1f
    }
}
impl WiFiRate {
    /// Check if the rate is using the HT PHY.
    pub const fn is_ht(&self) -> bool {
        self.into_bits() >= 0x10
    }
    /// Check if the rate uses a short guard interval.
    pub const fn is_short_gi(&self) -> bool {
        self.into_bits() >= 0x18
    }
}

/// This is a slot borrowed from the slot queue.
///
/// It is used, to make sure that access to a slot is exclusive.
/// It will return the slot back into the slot queue once dropped.
struct BorrowedTxSlot<'a> {
    dyn_sender: DynamicSender<'a, usize>,
    slot: usize,
}
impl Deref for BorrowedTxSlot<'_> {
    type Target = usize;
    fn deref(&self) -> &Self::Target {
        &self.slot
    }
}
impl Drop for BorrowedTxSlot<'_> {
    fn drop(&mut self) {
        // We can ignore the result here, because we know that this slot was taken from the queue,
        // and therefore the queue must have space for it.
        let _ = self.dyn_sender.try_send(self.slot);
        trace!("Slot {} is now free again.", self.slot);
    }
}
/// This keeps track of all the TX slots available, by using a queue of slot numbers in the
/// background, which makes it possible to await a slot becoming free.
struct TxSlotQueue {
    slots: Channel<DefaultRawMutex, usize, 5>,
}
impl TxSlotQueue {
    /// Create a new slot manager.
    pub fn new(slots: Range<usize>) -> Self {
        let temp = Self {
            slots: Channel::new(),
        };
        for slot in slots {
            let _ = temp.slots.try_send(slot);
        }
        temp
    }
    /// Asynchronously wait for a new slot to become available.
    pub async fn wait_for_slot(&self) -> BorrowedTxSlot {
        BorrowedTxSlot {
            dyn_sender: self.slots.dyn_sender(),
            slot: self.slots.receive().await,
        }
    }
}

static WIFI_RX_SIGNAL_QUEUE: SignalQueue = SignalQueue::new();

#[allow(clippy::declare_interior_mutable_const)]
const EMPTY_SLOT: TxSlotStateSignal = TxSlotStateSignal::new();
/// These are for knowing, when transmission has finished.
static WIFI_TX_SLOTS: [TxSlotStateSignal; 5] = [EMPTY_SLOT; 5];

// We run tx_pwctrl_background every four transmissions.
static FRAMES_SINCE_LAST_TXPWR_CTRL: AtomicU8 = AtomicU8::new(0);

#[inline(always)]
fn process_tx_complete(slot: usize) {
    let wifi = unsafe { WIFI::steal() };
    wifi.txq_state()
        .tx_complete_clear()
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << slot)) });
    if slot < 5 {
        WIFI_TX_SLOTS[slot].signal(TxSlotStatus::Done);
    }
}
#[inline(always)]
fn process_tx_timeout(slot: usize) {
    let wifi = unsafe { WIFI::steal() };

    wifi.txq_state()
        .tx_error_clear()
        .modify(|r, w| unsafe { w.slot_timeout().bits(r.slot_timeout().bits() | (1 << slot)) });
    if slot < 5 {
        let tx_slot_config = wifi.tx_slot_config(4 - slot);
        WiFi::invalidate_tx_slot(tx_slot_config);
        WiFi::disable_tx_slot(tx_slot_config);
        WIFI_TX_SLOTS[slot].signal(TxSlotStatus::Timeout);
    }
}
#[inline(always)]
fn process_collision(slot: usize) {
    let wifi = unsafe { WIFI::steal() };
    wifi.txq_state().tx_error_clear().modify(|r, w| unsafe {
        w.slot_collision()
            .bits(r.slot_collision().bits() | (1 << slot))
    });
    if slot < 5 {
        let tx_slot_config = wifi.tx_slot_config(4 - slot);
        WiFi::invalidate_tx_slot(tx_slot_config);
        WiFi::disable_tx_slot(tx_slot_config);
        WIFI_TX_SLOTS[slot].signal(TxSlotStatus::Collision);
    }
}

#[ram]
extern "C" fn interrupt_handler() {
    // We don't want to have to steal this all the time.
    let wifi = unsafe { WIFI::steal() };

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
    } else if cause & 0x80 != 0 {
        let mut txq_complete_status = wifi.txq_state().tx_complete_status().read().bits();
        while txq_complete_status != 0 {
            let slot = txq_complete_status.trailing_zeros();
            process_tx_complete(slot as usize);
            // We mask away, the bit for our slot.
            txq_complete_status &= !(1 << slot);
        }
    } else if cause & 0x80000 != 0 {
        // Timeout
        let mut tx_error_status = wifi
            .txq_state()
            .tx_error_status()
            .read()
            .slot_timeout()
            .bits();
        while tx_error_status != 0 {
            let slot = tx_error_status.trailing_zeros();
            process_tx_timeout(slot as usize);
            // We mask away, the bit for our slot.
            tx_error_status &= !(1 << slot);
        }
    } else if cause & 0x100 != 0 {
        // Timeout
        let mut tx_error_status = wifi
            .txq_state()
            .tx_error_status()
            .read()
            .slot_collision()
            .bits();
        while tx_error_status != 0 {
            let slot = tx_error_status.trailing_zeros();
            process_collision(slot as usize);
            // We mask away, the bit for our slot.
            tx_error_status &= !(1 << slot);
        }
    }
}

/// What should be done, if an error occurs, while transmitting.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
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
    dma_list_item: &'res mut RxDMAListItem,
}
impl BorrowedBuffer<'_> {
    /// Returns the complete buffer returned by the hardware.
    ///
    /// This includes the header added by the hardware.
    pub fn raw_buffer(&self) -> &[u8] {
        self.dma_list_item.buffer()
    }
    /// Returns the actual MPDU from the buffer excluding the prepended [wifi_pkt_rx_ctrl_t].
    pub fn mpdu_buffer(&self) -> &[u8] {
        &self.raw_buffer()[size_of::<wifi_pkt_rx_ctrl_t>()..]
    }
    /// Returns the header attached by the hardware.
    pub fn header_buffer(&self) -> &[u8] {
        &self.raw_buffer()[0..size_of::<wifi_pkt_rx_ctrl_t>()]
    }
    /// The Received Signal Strength Indicator (RSSI).
    pub fn rssi(&self) -> i8 {
        self.header_buffer()[0] as i8 - 96
    }
    /// Check if the frame is for the specified interface.
    pub fn is_frame_for_interface(&self, interface: usize) -> bool {
        WiFi::is_valid_interface(interface).is_ok()
            && check_bit!(self.header_buffer()[3], bit!(interface + 4))
    }
    pub fn interface_iterator(&self) -> impl Iterator<Item = usize> + '_ {
        let byte = self.header_buffer()[3];
        (0..WiFi::INTERFACE_COUNT).filter(move |interface| check_bit!(byte, bit!(interface + 4)))
    }
}
impl Drop for BorrowedBuffer<'_> {
    fn drop(&mut self) {
        self.dma_list
            .lock(|dma_list| dma_list.borrow_mut().recycle(self.dma_list_item));
    }
}
/// The bank of the rx filter.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum RxFilterBank {
    BSSID,
    ReceiverAddress,
}
impl RxFilterBank {
    const fn into_bits(self) -> usize {
        match self {
            Self::BSSID => 0,
            Self::ReceiverAddress => 1,
        }
    }
}
/// Errors returned by the Wi-Fi driver.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum WiFiError {
    InvalidChannel,
    InterfaceOutOfBounds,
    TxTimeout,
    TxCollision,
    AckTimeout,
    CtsTimeout,
    RtsTimeout,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
/// Parameters for the transmission of an MPDU.
pub struct TxParameters {
    /// The rate at which to tranmsit the packet.
    pub rate: WiFiRate,
    /// The duration of the TXOP.
    pub duration: u16,
    /// Override the sequence number, with the one maintained by the driver.
    ///
    /// This is recommended.
    pub override_seq_num: bool,
    /// What to do in case an error occurs.
    pub tx_error_behaviour: TxErrorBehaviour,
    pub tx_timeout: usize,
}
impl Default for TxParameters {
    fn default() -> Self {
        Self {
            rate: WiFiRate::default(),
            duration: 0,
            override_seq_num: false,
            tx_error_behaviour: TxErrorBehaviour::Drop,
            tx_timeout: 10,
        }
    }
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
/// This determines what frames bypass the RX filter.
pub enum ScanningMode {
    #[default]
    Disabled,
    BeaconsOnly,
    ManagementAndData,
}

/// A [Result] returned by the Wi-Fi driver.
pub type WiFiResult<T> = Result<T, WiFiError>;

/// Driver for the Wi-Fi peripheral.
pub struct WiFi<'res> {
    radio_clock: RADIO_CLK,
    wifi: WIFI,
    dma_list: &'res blocking_mutex::Mutex<DefaultRawMutex, RefCell<DMAList<'static>>>,
    current_channel: AtomicU8,
    sequence_number: AtomicU16,
    tx_slot_queue: TxSlotQueue,
}
impl<'res> WiFi<'res> {
    #[cfg(any(feature = "esp32", feature = "esp32s2"))]
    pub const INTERFACE_COUNT: usize = 4;
    /// Returns the name of a radio peripheral.
    fn radio_peripheral_name(radio_peripheral: &RadioPeripherals) -> &'static str {
        match radio_peripheral {
            #[cfg(not(feature = "esp32s2"))]
            RadioPeripherals::Bt => "BT",
            RadioPeripherals::Phy => "PHY",
            RadioPeripherals::Wifi => "WiFi",
        }
    }
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
    /// Disable the Wi-Fi power domain.
    fn disable_wifi_power_domain() {
        unsafe {
            let rtc_cntl = &*LPWR::ptr();
            trace!("Disabling WiFi power domain.");
            rtc_cntl
                .dig_iso()
                .modify(|_, w| w.wifi_force_iso().set_bit());
            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.wifi_force_pd().set_bit());
        }
    }
    /// Enable the specified clock.
    fn enable_clock(radio_clock: &mut RADIO_CLK, radio_peripheral: RadioPeripherals) {
        trace!(
            "Enabling {} clock.",
            Self::radio_peripheral_name(&radio_peripheral)
        );
        radio_clock.enable(radio_peripheral);
    }
    /// Disable the specified clock.
    fn disable_clock(radio_clock: &mut RADIO_CLK, radio_peripheral: RadioPeripherals) {
        trace!(
            "Disabling {} clock.",
            Self::radio_peripheral_name(&radio_peripheral)
        );
        radio_clock.disable(radio_peripheral);
    }
    /// Enable the PHY.
    fn phy_enable(radio_clock: &mut RADIO_CLK) {
        Self::enable_clock(radio_clock, RadioPeripherals::Phy);
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
    }
    /// Reset the MAC.
    fn reset_mac(radio_clock: &mut RADIO_CLK, wifi: &WIFI) {
        trace!("Reseting MAC.");
        radio_clock.reset_mac();
        wifi.ctrl()
            .modify(|r, w| unsafe { w.bits(r.bits() & 0x7fffffff) });
    }
    /// Initialize the MAC.
    fn init_mac(wifi: &WIFI) {
        trace!("Initializing MAC.");
        wifi.ctrl()
            .modify(|r, w| unsafe { w.bits(r.bits() & 0xffffe800) });
    }
    /// Deinitialize the MAC.
    fn deinit_mac(wifi: &WIFI) {
        trace!("Deinitializing MAC.");
        wifi.ctrl().modify(|r, w| unsafe {
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
    fn set_rx_status(wifi: &WIFI, enabled: bool) {
        trace!("Enabling RX.");
        wifi.rx_ctrl().write(|w| w.rx_enable().bit(enabled));
    }
    fn chip_enable(wifi: &WIFI) {
        trace!("chip_enable");
        Self::set_rx_status(wifi, true);
    }
    /// Initialize the WiFi peripheral.
    pub fn new<const BUFFER_SIZE: usize, const BUFFER_COUNT: usize>(
        wifi: WIFI,
        mut radio_clock: RADIO_CLK,
        _adc2: ADC2,
        dma_resources: &'res mut DMAResources<BUFFER_SIZE, BUFFER_COUNT>,
    ) -> Self {
        trace!("Initializing WiFi.");
        Self::enable_wifi_power_domain();
        Self::enable_clock(&mut radio_clock, RadioPeripherals::Wifi);
        Self::phy_enable(&mut radio_clock);
        let start_time = Instant::now();
        Self::reset_mac(&mut radio_clock, &wifi);
        Self::init_mac(&wifi);
        Self::ic_enable();
        Self::chip_enable(&wifi);

        let temp = Self {
            wifi,
            radio_clock,
            current_channel: AtomicU8::new(1),
            dma_list: unsafe { dma_resources.init() },
            sequence_number: AtomicU16::new(0),
            tx_slot_queue: TxSlotQueue::new(0..5),
        };
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
    pub async fn receive(&self) -> BorrowedBuffer<'res> {
        let dma_list_item;

        // Sometimes the DMA list descriptors don't contain any data, even though the hardware indicated reception.
        // We loop until we get something.
        loop {
            WIFI_RX_SIGNAL_QUEUE.next().await;
            if let Some(current) = self
                .dma_list
                .lock(|dma_list| dma_list.borrow_mut().take_first())
            {
                trace!("Received packet. len: {}", current.buffer().len());
                dma_list_item = current;
                break;
            }
            trace!("Received empty packet.");
        }

        BorrowedBuffer {
            dma_list: self.dma_list,
            dma_list_item,
        }
    }
    /// Enable the transmission slot.
    fn enable_tx_slot(tx_slot_config: &TX_SLOT_CONFIG) {
        tx_slot_config
            .plcp0()
            .modify(|_, w| w.slot_valid().set_bit().slot_enabled().set_bit());
    }
    /// Disable the transmission slot.
    fn disable_tx_slot(tx_slot_config: &TX_SLOT_CONFIG) {
        tx_slot_config
            .plcp0()
            .modify(|_, w| w.slot_valid().clear_bit().slot_enabled().clear_bit());
    }
    /// Invalidate the transmission slot.
    fn invalidate_tx_slot(tx_slot_config: &TX_SLOT_CONFIG) {
        tx_slot_config
            .plcp0()
            .modify(|_, w| w.slot_valid().clear_bit());
    }
    /// Set the packet for transmission.
    async fn transmit_internal(
        &self,
        dma_list_item: Pin<&TxDMAListItem>,
        tx_parameters: &TxParameters,
        ack_for_interface: Option<usize>,
        slot: usize,
    ) -> WiFiResult<()> {
        let length = dma_list_item.buffer().len();
        let reversed_slot = 4 - slot;

        let tx_slot_config = self.wifi.tx_slot_config(reversed_slot);
        tx_slot_config
            .config()
            .write(|w| unsafe { w.timeout().bits(tx_parameters.tx_timeout as u16) });

        tx_slot_config.plcp0().write(|w| unsafe {
            w.dma_addr()
                .bits(dma_list_item.get_ref() as *const _ as u32)
        });
        // I hacked the wait for ACK bit in here, since I messed it up in the SVD.
        tx_slot_config.plcp0().modify(|r, w| unsafe {
            w.bits(r.bits() | 0x600000 | ((ack_for_interface.is_some() as u32) << 27))
        });
        let rate = tx_parameters.rate;

        self.wifi.plcp1(reversed_slot).write(|w| unsafe {
            if let Some(interface) = ack_for_interface {
                w.interface_id().bits(interface as u8)
            } else {
                w
            }
            .len()
            .bits(length as u16)
            .is_80211_n()
            .bit(rate.is_ht())
            .rate()
            .bits(rate.into_bits())
        });
        self.wifi
            .plcp2(reversed_slot)
            .write(|w| w.unknown().bit(true));
        let duration = tx_parameters.duration as u32;
        self.wifi
            .duration(reversed_slot)
            .write(|w| unsafe { w.bits(duration | (duration << 0x10)) });
        if rate.is_ht() {
            self.wifi.ht_sig(reversed_slot).write(|w| unsafe {
                w.bits(
                    (rate.into_bits() as u32 & 0b111)
                        | ((length as u32 & 0xffff) << 8)
                        | (0b111 << 24)
                        | ((rate.is_short_gi() as u32) << 31),
                )
            });
            self.wifi
                .ht_unknown(reversed_slot)
                .write(|w| unsafe { w.length().bits(length as u32 | 0x50000) });
        }
        // This also compensates for other slots being marked as done, without it being used at
        // all.
        WIFI_TX_SLOTS[slot].reset();
        Self::enable_tx_slot(tx_slot_config);
        struct CancelOnDrop<'a> {
            tx_slot_config: &'a TX_SLOT_CONFIG,
            slot: usize,
        }
        impl CancelOnDrop<'_> {
            async fn wait_for_tx_complete(self) -> WiFiResult<()> {
                // Wait for the hardware to confirm transmission.
                let res = WIFI_TX_SLOTS[self.slot].wait().await;
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
                WIFI_TX_SLOTS[self.slot].reset();
                forget(self);
                trace!("TX complete {res:?}");
                res
            }
        }
        impl Drop for CancelOnDrop<'_> {
            fn drop(&mut self) {
                WiFi::invalidate_tx_slot(self.tx_slot_config);
                WIFI_TX_SLOTS[self.slot].reset();
            }
        }
        let cancel_on_drop = CancelOnDrop {
            tx_slot_config,
            slot,
        };
        cancel_on_drop.wait_for_tx_complete().await?;

        match self.wifi.pmd(reversed_slot).read().bits() >> 0xc {
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
    /// an extra bit in the FCS flags and may have to override the sequence number.
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
        let slot = self.tx_slot_queue.wait_for_slot().await;
        trace!("Acquired slot {}.", *slot);

        if tx_parameters.override_seq_num {
            let seq_num = self.sequence_number.load(Ordering::Relaxed).wrapping_add(1);
            self.sequence_number.store(seq_num, Ordering::Relaxed);
            if let Some(sequence_number) = buffer.get_mut(22..24) {
                sequence_number.copy_from_slice((seq_num << 4).to_le_bytes().as_slice());
            }
        }

        // We initialize the DMA list item.
        let mut dma_list_item = unsafe { DMAListItem::new_for_tx(buffer) };

        // And then pin it, before passing it to hardware.
        let dma_list_item = pin!(dma_list_item);
        let dma_list_ref = dma_list_item.into_ref();

        match tx_parameters.tx_error_behaviour {
            TxErrorBehaviour::Drop => {
                self.transmit_internal(dma_list_ref, tx_parameters, ack_for_interface, *slot)
                    .await?;
                Ok(0)
            }
            TxErrorBehaviour::RetryUntil(retries) => {
                let mut res = self
                    .transmit_internal(dma_list_ref, tx_parameters, ack_for_interface, *slot)
                    .await;
                for i in 0..retries {
                    match res {
                        Ok(()) => return Ok(i),
                        Err(WiFiError::TxTimeout | WiFiError::TxCollision) => {}
                        _ => {
                            if let Some(byte) = buffer.get_mut(1) {
                                *byte |= bit!(3);
                            }
                            debug!("Retransmitting MPDU.");
                        }
                    }
                    res = self
                        .transmit_internal(dma_list_ref, tx_parameters, ack_for_interface, *slot)
                        .await;
                }
                res.map(|_| 0)
            }
        }
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
        trace!("Changing channel to {channel_number}");
        Self::deinit_mac(&self.wifi);
        unsafe {
            #[cfg(nomac_channel_set)]
            crate::ffi::chip_v7_set_chan_nomac(channel_number, 0);
            #[cfg(not(nomac_channel_set))]
            crate::ffi::chip_v7_set_chan(channel_number, 0);
            disable_wifi_agc();
        }
        Self::init_mac(&self.wifi);
        unsafe {
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
    /// Check if that interface is valid.
    pub const fn is_valid_interface(interface: usize) -> WiFiResult<()> {
        if interface < WiFi::INTERFACE_COUNT {
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
        Self::is_valid_interface(interface)?;
        self.wifi
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
        Self::is_valid_interface(interface)?;
        self.wifi
            .interface_rx_control(interface)
            .modify(|_, w| w.bssid_check().bit(enabled));
        Ok(())
    }
    /// Write raw value to the `INTERFACE_RX_CONTROL` register.
    ///
    /// NOTE: This exists mostly for debugging purposes, so if you find yourself using this for
    /// something else than that, you probably want to use one of the other functions.
    pub fn write_rx_policy_raw(&self, interface: usize, val: u32) -> WiFiResult<()> {
        Self::is_valid_interface(interface)?;
        self.wifi
            .interface_rx_control(interface)
            .write(|w| unsafe { w.bits(val) });
        Ok(())
    }
    /// Read a raw value from the `INTERFACE_RX_CONTROL` register.
    ///
    /// NOTE: This exists mostly for debugging purposes, so if you find yourself using this for
    /// something else than that, you probably want to use one of the other functions.
    pub fn read_rx_policy_raw(&self, interface: usize) -> WiFiResult<u32> {
        Self::is_valid_interface(interface)?;
        Ok(self.wifi.interface_rx_control(interface).read().bits())
    }
    /// Set the parameters for the filter.
    pub fn set_filter(
        &self,
        bank: RxFilterBank,
        interface: usize,
        address: [u8; 6],
        mask: [u8; 6],
    ) -> WiFiResult<()> {
        Self::is_valid_interface(interface)?;
        let bank = self.wifi.filter_bank(bank.into_bits());
        bank.addr_low(interface)
            .write(|w| unsafe { w.bits(u32::from_le_bytes(address[..4].try_into().unwrap())) });
        bank.addr_high(interface).write(|w| unsafe {
            w.addr()
                .bits(u16::from_le_bytes(address[4..6].try_into().unwrap()))
        });
        bank.mask_low(interface)
            .write(|w| unsafe { w.bits(u32::from_le_bytes(mask[..4].try_into().unwrap())) });
        bank.mask_high(interface).write(|w| unsafe {
            w.mask()
                .bits(u16::from_le_bytes(mask[4..6].try_into().unwrap()))
        });
        Ok(())
    }
    /// Enable or disable scanning mode.
    pub fn set_scanning_mode(
        &self,
        interface: usize,
        scanning_mode: ScanningMode,
    ) -> WiFiResult<()> {
        Self::is_valid_interface(interface)?;
        self.wifi
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
}
impl Drop for WiFi<'_> {
    fn drop(&mut self) {
        // Ensure, that the radio clocks and the power domain are disabled.
        Self::disable_clock(&mut self.radio_clock, RadioPeripherals::Wifi);
        Self::disable_clock(&mut self.radio_clock, RadioPeripherals::Phy);
        Self::disable_wifi_power_domain();
    }
}
