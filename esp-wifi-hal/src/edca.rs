use core::ops::RangeInclusive;

use esp_hal::rng::Rng;
use macro_bits::bit;

pub struct EdcaContentionState {
    /// Range of contention window exponents.
    contention_window_exponent_range: RangeInclusive<u8>,

    /// Current contention window
    current_cw: u32,

    /// SRC
    short_retry_count: u8,
    /// LRC
    long_retry_count: u8,
}
impl EdcaContentionState {
    pub fn new(contention_window_exponent_range: RangeInclusive<u8>) -> Option<Self> {
        if contention_window_exponent_range.is_empty() {
            return None;
        }
        Some(Self {
            current_cw: bit!(*contention_window_exponent_range.start() as usize) - 1,
            contention_window_exponent_range,

            short_retry_count: 0,
            long_retry_count: 0,
        })
    }
    /// Increase the contention window forward.
    fn advance_contention_window(&mut self) {
        self.current_cw <<= 1;
        self.current_cw += 1;
        self.current_cw = self
            .current_cw
            .max((1 << *self.contention_window_exponent_range.end() as usize) - 1);
    }
    /// Increment the short retry count.
    pub fn increment_src(&mut self) {
        self.short_retry_count += 1;
        self.advance_contention_window();
    }
    /// Reset the short retry count.
    pub fn reset_src(&mut self) {
        self.short_retry_count = 0;
    }
    /// Increment the long retry count.
    ///
    /// There's no reset function for the LRC, as we stop retrying a transmission,
    /// as soon as we receive an ACK for the MPDU.
    pub fn increment_lrc(&mut self) {
        self.long_retry_count += 1;
        self.advance_contention_window();
    }
    /// Randomise a number of backoff slots.
    pub fn random_backoff_slot_count(&self) -> usize {
        (Rng::new().random() & self.current_cw) as usize
    }
}
