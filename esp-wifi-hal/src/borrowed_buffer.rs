use core::cell::RefCell;

use crate::esp_wifi_sys::include::wifi_pkt_rx_ctrl_t;
use embassy_sync::blocking_mutex;
use esp_hal::dma::DmaDescriptor;
use macro_bits::{bit, check_bit};

use crate::{
    DefaultRawMutex,
    async_driver::WiFi,
    dma_list::DmaList,
    ll::{INTERFACE_COUNT, LowLevelDriver},
    rates::{HrDsssRate, HtRate, OfdmRate, RxPhyRate},
};

/// A buffer borrowed from the DMA list.
pub struct BorrowedBuffer<'res> {
    pub(crate) dma_list: &'res blocking_mutex::Mutex<DefaultRawMutex, RefCell<DmaList>>,
    pub(crate) dma_descriptor: &'res mut DmaDescriptor,
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
    pub const fn raw_header(&self) -> &wifi_pkt_rx_ctrl_t {
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
        if delta & 0b11 == 0 {
            delta
        } else {
            (delta & !0b11) + 4
        }
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
                /*
                defmt_or_log::warn!(
                    "Buffer was shorter then calculated length. Corrected len: {} Padded len: {} SIG len: {} Header: {:02x}",
                    self.unpadded_buffer_len(),
                    self.dma_descriptor.len(),
                    self.raw_header().sig_len(),
                    &self.padded_buffer()[..Self::RX_CONTROL_HEADER_LENGTH]
                );*/
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
        let rssi = self.raw_header().rssi() as i8;
        let offset = cfg_select! {
            feature = "esp32" => {
                -96
            }
            _ => {
                0
            }
        };
        rssi + offset
    }
    /// The time at which the packet was received in µs.
    pub fn timestamp(&self) -> u32 {
        self.raw_header().timestamp()
    }
    /// The time the packet was received, corrected for the difference between MAC and system
    /// clock.
    pub fn corrected_timestamp(&self) -> embassy_time::Instant {
        embassy_time::Instant::from_micros(
            self.timestamp() as u64 + LowLevelDriver::mac_time_offset().as_micros(),
        )
    }
    /// Check if the frame is an A-MPDU.
    pub fn aggregation(&self) -> bool {
        check_bit!(self.header_buffer()[7], bit!(3))
    }
    /// The phy rate, at which this frame was transmitted.
    pub fn phy_rate(&self) -> Option<RxPhyRate> {
        let raw_header = self.raw_header();
        let cbw40 = raw_header.cwb() == 1;
        let short_gi = raw_header.sgi() == 1;
        let mcs = raw_header.mcs() as u8;
        let rate = raw_header.rate() as u8;
        match raw_header.sig_mode() {
            0 => {
                if rate <= 7 {
                    HrDsssRate::new(rate % 4, rate / 4 == 1).map(RxPhyRate::HrDsss)
                } else if rate <= 15 {
                    Some(RxPhyRate::Ofdm(match rate {
                        0x0b => OfdmRate::Mbits6,
                        0x0f => OfdmRate::Mbits9,
                        0x0a => OfdmRate::Mbits12,
                        0x0e => OfdmRate::Mbits18,
                        0x09 => OfdmRate::Mbits24,
                        0x0d => OfdmRate::Mbits36,
                        0x08 => OfdmRate::Mbits48,
                        0x0c => OfdmRate::Mbits54,
                        _ => unreachable!(),
                    }))
                } else {
                    None
                }
            }
            1 => HtRate::new(mcs, short_gi, cbw40).map(RxPhyRate::Ht),
            _ => None,
        }
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
    /// Check if the contained MPDU is a fragment.
    pub fn is_fragment(&self) -> bool {
        self.raw_buffer()
            .get(Self::RX_CONTROL_HEADER_LENGTH + 1)
            .map(|byte| byte & 4 != 0)
            .unwrap_or_default()
    }
}
impl Drop for BorrowedBuffer<'_> {
    fn drop(&mut self) {
        self.dma_list.lock(|dma_list| {
            dma_list.borrow_mut().recycle(self.dma_descriptor);
        });
    }
}
