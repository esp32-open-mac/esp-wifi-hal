#![no_std]
//! # Type safe Wi-Fi PHY rate handling for the ESP32 chips.
//! This is used by `esp-wifi-hal` and is largely tailored for its needs.
//! However, as it may be useful outside of that context, and to make testing easier, it became its own crate.

use core::marker::PhantomData;

use num_rational::Ratio;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// A Wi-Fi PHY rate.
pub enum PhyRate<D: RateDirection> {
    /// High Rate / Direct Sequence Spread Spectrum rate
    HrDsss(HrDsssRate),
    /// Orthogonal Frequency Division Multiplexing rate
    Ofdm(OfdmRate),
    /// High Throughput rate
    Ht(HtRate<D>),
}
impl<D: RateDirection> PhyRate<D> {
    /// Get the rate in hardware encoding.
    pub const fn as_hardware_rate(&self) -> u8 {
        match self {
            Self::HrDsss(hr_dsss) => hr_dsss.rate_index + 4 * (hr_dsss.short_preamble as u8),
            Self::Ofdm(ofdm) => *ofdm as u8,
            Self::Ht(ht) => 0x10 + ht.mcs_index * (1 + ht.short_gi as u8),
        }
    }
}
impl<D: RateDirection> Default for PhyRate<D> {
    /// The default PHY rate is OFDM 6 Mbit/s.
    fn default() -> Self {
        Self::Ofdm(OfdmRate::default())
    }
}
impl<D: RateDirection> From<HrDsssRate> for PhyRate<D> {
    fn from(value: HrDsssRate) -> Self {
        Self::HrDsss(value)
    }
}
impl<D: RateDirection> From<OfdmRate> for PhyRate<D> {
    fn from(value: OfdmRate) -> Self {
        Self::Ofdm(value)
    }
}
impl<D: RateDirection> From<HtRate<D>> for PhyRate<D> {
    fn from(value: HtRate<D>) -> Self {
        Self::Ht(value)
    }
}
impl<D: RateDirection> Rate for PhyRate<D> {
    fn bandwidth(&self) -> usize {
        match self {
            Self::HrDsss(rate) => rate.bandwidth(),
            Self::Ofdm(rate) => rate.bandwidth(),
            Self::Ht(rate) => rate.bandwidth(),
        }
    }
    fn constellation(&self) -> Constellation {
        match self {
            Self::HrDsss(rate) => rate.constellation(),
            Self::Ofdm(rate) => rate.constellation(),
            Self::Ht(rate) => rate.constellation(),
        }
    }
    fn symbol_interval(&self) -> usize {
        match self {
            Self::HrDsss(rate) => rate.symbol_interval(),
            Self::Ofdm(rate) => rate.symbol_interval(),
            Self::Ht(rate) => rate.symbol_interval(),
        }
    }
    fn data_rate(&self) -> usize {
        match self {
            Self::HrDsss(rate) => rate.data_rate(),
            Self::Ofdm(rate) => rate.data_rate(),
            Self::Ht(rate) => rate.data_rate(),
        }
    }
    fn airtime(&self, ppdu_length: usize) -> usize {
        match self {
            Self::HrDsss(rate) => rate.airtime(ppdu_length),
            Self::Ofdm(rate) => rate.airtime(ppdu_length),
            Self::Ht(rate) => rate.airtime(ppdu_length),
        }
    }
}

/// A Wi-Fi PHY rate for transmission.
pub type TxPhyRate = PhyRate<Tx>;
/// A Wi-Fi PHY rate for reception.
pub type RxPhyRate = PhyRate<Rx>;

const KILO: usize = 1_000;
const MEGA: usize = 1_000_000;
const GIGA: usize = 1_000_000_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// An enumeration of the different constellations used by 802.11.
///
/// The index assigned to each variant is the number of coded bits per symbol per subcarrier.
pub enum Constellation {
    /// Binary Phase-Shift Keying
    Bpsk = 1,
    /// Quadrature Phase-Shift Keying
    Qpsk = 2,
    /// Quadrature Amplitude Modulation with 16 constellation points
    Qam16 = 4,
    /// Quadrature Amplitude Modulation with 64 constellation points
    Qam64 = 6,
}
impl core::fmt::Display for Constellation {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Self::Bpsk => "BPSK",
            Self::Qpsk => "QPSK",
            Self::Qam16 => "QAM-16",
            Self::Qam64 => "QAM-64",
        })
    }
}
#[cfg(feature = "defmt")]
impl defmt::Format for Constellation {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(
            fmt,
            "{}",
            match self {
                Self::Bpsk => "BPSK",
                Self::Qpsk => "QPSK",
                Self::Qam16 => "QAM-16",
                Self::Qam64 => "QAM-64",
            }
        );
    }
}
/// A trait implemented by all Wi-Fi rates.
pub trait Rate {
    /// Bandwidth occupied in Hz.
    fn bandwidth(&self) -> usize;
    /// The constellation used by the rate.
    ///
    /// You can use this to query the number of coded bits per symbol per subcarrier.
    fn constellation(&self) -> Constellation;
    /// Duration of a data symbol in ns.
    ///
    /// NOTE: This is not equal to the chip duration for DSSS.
    fn symbol_interval(&self) -> usize;
    /// Symbol rate in Hz.
    ///
    /// For HT short GI, this is a rational number.
    ///
    /// NOTE: This is not equal to the chipping rate for DSSS.
    fn symbol_rate(&self) -> usize {
        GIGA / self.symbol_interval()
    }
    /// Data rate in bits per second.
    ///
    /// FUN FACT: The rates are calculated and not looked up, so their accuracy is
    /// pretty much absolute. This creates an interesting problem for testing, as
    /// our values are more accurate than the ones in the spec...
    fn data_rate(&self) -> usize;
    /// Calculate the airtime of a PPDU in ns.
    fn airtime(&self, ppdu_length: usize) -> usize;
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// A rate of the HR/DSSS PHY.
#[derive(Default)]
pub struct HrDsssRate {
    rate_index: u8,
    short_preamble: bool,
}
impl HrDsssRate {
    /// Long preamble 1 Mbit/s.
    pub const LONG_1M: Self = Self::new(0, false).unwrap();
    /// Long preamble 2 Mbit/s.
    pub const LONG_2M: Self = Self::new(1, false).unwrap();
    /// Long preamble 5.5 Mbit/s.
    pub const LONG_5M5: Self = Self::new(2, false).unwrap();
    /// Long preamble 11 Mbit/s.
    pub const LONG_11M: Self = Self::new(3, false).unwrap();
    /// Long preamble 2 Mbit/s.
    pub const SHORT_2M: Self = Self::new(1, true).unwrap();
    /// Long preamble 5.5 Mbit/s.
    pub const SHORT_5M5: Self = Self::new(2, true).unwrap();
    /// Long preamble 11 Mbit/s.
    pub const SHORT_11M: Self = Self::new(3, true).unwrap();

    /// Construct a new HR/DSSS rate.
    ///
    /// The `rate` parameter is a value between zero and three, which maps to the rates
    /// 1 Mbit/s, 2 Mbit/s, 5.5 Mbit/s and 11 Mbit/s, in that order.
    ///
    /// [None] is returned, if `rate` is out of bounds, or 1 Mbit/s with short preamble
    /// is requested.
    pub const fn new(rate_index: u8, short_preamble: bool) -> Option<Self> {
        if rate_index > 3 || (rate_index == 0 && short_preamble) {
            None
        } else {
            Some(Self {
                rate_index,
                short_preamble,
            })
        }
    }
    /// Set the data rate.
    ///
    /// See [Self::new] for the meaning of `rate`.
    ///
    /// If you try to set 1 Mbit/s short preamble, this will be a noop, as that config
    /// is invalid.
    pub const fn set_rate(&mut self, rate_index: u8) {
        if (rate_index == 0 && !self.short_preamble) && rate_index <= 3 {
            self.rate_index = rate_index;
        }
    }
    /// Set if a short preamble is used.
    ///
    /// If you try to set 1 Mbit/s short preamble, this will be a noop, as that config
    /// is invalid.
    pub const fn set_short_preamble(&mut self, short_preamble: bool) {
        if self.rate_index != 0 {
            self.short_preamble = short_preamble;
        }
    }
    /// Is Complementary Code Keying (CCK) used.
    pub const fn is_cck(&self) -> bool {
        self.rate_index >= 2
    }
}
impl Rate for HrDsssRate {
    fn constellation(&self) -> Constellation {
        if self.rate_index.is_multiple_of(2) {
            Constellation::Bpsk
        } else {
            Constellation::Qpsk
        }
    }
    fn bandwidth(&self) -> usize {
        22 * MEGA
    }
    fn symbol_interval(&self) -> usize {
        if self.is_cck() { KILO * 8 / 11 } else { KILO }
    }
    fn symbol_rate(&self) -> usize {
        if self.is_cck() { 11 * MEGA / 8 } else { MEGA }
    }
    fn data_rate(&self) -> usize {
        match self.rate_index {
            0 => MEGA,
            1 => 2 * MEGA,
            2 => 11 * MEGA / 2,
            3 => 11 * MEGA,
            _ => unreachable!(),
        }
    }
    fn airtime(&self, ppdu_length: usize) -> usize {
        let preamble_duration = if self.short_preamble { 96 } else { 192 } * KILO;
        // Self::data_rate always returns an integer for this PHY.
        let data_bits = ppdu_length * 8;

        let data_duration = match self.rate_index {
            0 => data_bits * KILO,
            1 => data_bits * KILO / 2,
            2 => data_bits * KILO / 11,
            3 => data_bits * KILO / 22,
            _ => unreachable!(),
        };

        preamble_duration + data_duration
    }
}

/// A rate associated with an OFDM based PHY.
///
/// Subcarrier are numbered from lowest to highest frequency starting at zero.
pub trait OfdmBasedRate: Rate {
    /// Number of data subcarriers
    fn data_subcarriers(&self) -> usize;
    /// Number of pilot subcarriers
    fn pilot_subcarriers(&self) -> usize;
    /// Number of modulated subcarriers (data + pilot)
    fn modulated_subcarriers(&self) -> usize {
        self.data_subcarriers() + self.pilot_subcarriers()
    }
    /// Total number of subcarriers, modulated and unmodulated.
    fn total_subcarriers(&self) -> usize;
    /// The distance between the centers of two subcarriers in Hz.
    fn subcarrier_spacing(&self) -> usize {
        self.bandwidth() / self.total_subcarriers()
    }
    /// The coding ratio.
    fn coding_rate(&self) -> Ratio<usize>;
    /// Duration of the guard interval (GI) in ns.
    fn guard_interval_duration(&self) -> usize;
    /// Duration of one DFT period in ns.
    fn dft_period(&self) -> usize {
        // This code assumes that the bandwidth is always an integer number of MHz.
        (self.total_subcarriers() * KILO) / (self.bandwidth() / MEGA)
    }
    /// Number of spatial streams.
    ///
    /// See N_SS in IEEE 802.11-2024.
    fn spatial_streams(&self) -> usize;
    /// Number of coded bits per OFDM symbol.
    ///
    /// See N_CBPS in IEEE 802.11-2024.
    fn coded_bits_per_ofdm_symbol(&self) -> usize {
        self.constellation() as usize * self.data_subcarriers() * self.spatial_streams()
    }
    /// Number of data bits per OFDM symbol.
    ///
    /// See N_DBPS in IEEE 802.11-2024.
    fn data_bits_per_ofdm_symbol(&self) -> usize {
        // The coding rates are always chosen in a way, so that this number is an integer.
        let coding_rate = self.coding_rate();
        self.coded_bits_per_ofdm_symbol() * coding_rate.numer() / coding_rate.denom()
    }
}
#[allow(missing_docs)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// A rate used with the OFDM PHY.
pub enum OfdmRate {
    #[default]
    Mbits6 = 0x0b,
    Mbits9 = 0x0f,
    Mbits12 = 0x0a,
    Mbits18 = 0x0e,
    Mbits24 = 0x09,
    Mbits36 = 0x0d,
    Mbits48 = 0x08,
    Mbits54 = 0x0c,
}
impl OfdmRate {}
impl Rate for OfdmRate {
    fn constellation(&self) -> Constellation {
        match 3 - ((*self as usize - 8) & 0b11) {
            0 => Constellation::Bpsk,
            1 => Constellation::Qpsk,
            2 => Constellation::Qam16,
            3 => Constellation::Qam64,
            _ => unreachable!(),
        }
    }
    fn bandwidth(&self) -> usize {
        20 * MEGA
    }
    fn symbol_interval(&self) -> usize {
        self.dft_period() + self.guard_interval_duration()
    }
    fn data_rate(&self) -> usize {
        self.symbol_rate() * self.data_bits_per_ofdm_symbol()
    }
    fn airtime(&self, ppdu_length: usize) -> usize {
        /// Number of short training symbols.
        const SHORT_TRAINING_SEQUENCE_SYMBOLS: usize = 10;
        /// Number of long training symbols.
        const LONG_TRAINING_SEQUENCE_SYMBOLS: usize = 2;
        /// Number of signal field symbols.
        const SIGNAL_SYMBOLS: usize = 1;

        let preamble_duration = SHORT_TRAINING_SEQUENCE_SYMBOLS * (self.dft_period() / 4)
            + LONG_TRAINING_SEQUENCE_SYMBOLS * (self.dft_period())
            + self.dft_period() / 2;
        assert_eq!(preamble_duration, 16 * KILO);

        let data_bits = 16 + ppdu_length * 8 + 6;
        let data_symbols = data_bits.div_ceil(self.data_bits_per_ofdm_symbol());
        let data_duration = (data_symbols + SIGNAL_SYMBOLS) * self.symbol_interval();

        preamble_duration + data_duration
    }
}
impl OfdmBasedRate for OfdmRate {
    fn data_subcarriers(&self) -> usize {
        48
    }
    fn pilot_subcarriers(&self) -> usize {
        4
    }
    fn total_subcarriers(&self) -> usize {
        64
    }
    fn coding_rate(&self) -> Ratio<usize> {
        let (numer, denom) = if *self == Self::Mbits48 {
            (2, 3)
        } else if (*self as usize - 8) & 0b100 != 0 {
            (3, 4)
        } else {
            (1, 2)
        };
        Ratio::new_raw(numer, denom)
    }
    fn spatial_streams(&self) -> usize {
        1
    }
    fn guard_interval_duration(&self) -> usize {
        800
    }
}
/// Trait indicating the direction (TX/RX) of a rate.
///
/// The purpose of this is to reflect asymmetries in the supported rate sets,
/// for TX and RX, in the type system.
pub trait RateDirection {
    /// Maximum Rate supported by HT in this direction.
    const MAX_HT_RATE: u8;
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// Indicates that whatever this is attached to is used for TX.
pub struct Tx;
impl RateDirection for Tx {
    const MAX_HT_RATE: u8 = 7;
}
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// Indicates that whatever this is attached to is used for RX.
pub struct Rx;
impl RateDirection for Rx {
    /// This is a deduction from the datasheet.
    const MAX_HT_RATE: u8 = 32;
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// A rate of the High Throughput PHY.
///
/// See IEEE 802.11-2024 Clause 19.
pub struct HtRate<D: RateDirection> {
    /// MCS index
    mcs_index: u8,
    /// Is the 400 ns guard interval used.
    short_gi: bool,
    /// Is the channel bandwidth 40 MHz.
    cbw40: bool,
    _phantom: PhantomData<D>,
}
impl<D: RateDirection> HtRate<D> {
    /// Construct a new [HtRate].
    ///
    /// The maximum TX HT-MCS index, the ESP32 can do, is 7, so [None] will be returned,
    /// if `mcs` is greater than that. For RX, this is 32.
    ///
    /// `cbw40` indicates if the transmission uses 40 MHz. This is currently ignored during TX.
    ///
    /// NOTE: While there are more MCS indices, they only differ in number of spatial streams,
    /// bandwidth and unequal modulation (god knows whoever implemented that), and the only
    /// configuration supported by the ESP32 for TX is one spatial stream with equal modulation.
    pub const fn new(mcs_index: u8, short_gi: bool, cbw40: bool) -> Option<Self> {
        if mcs_index <= D::MAX_HT_RATE {
            Some(Self {
                mcs_index,
                short_gi,
                cbw40,
                _phantom: PhantomData,
            })
        } else {
            None
        }
    }

    /// Construct a new [HtRate] with Long GI and 20 MHz channel bandwidth.
    ///
    /// Shorthand for [Self::new] with fixed parameters.
    pub const fn new_lgi_ht20(mcs_index: u8) -> Option<Self> {
        Self::new(mcs_index, false, false)
    }
    /// The MCS index.
    pub const fn mcs_index(&self) -> u8 {
        self.mcs_index
    }
    /// Set the MCS index of this rate.
    ///
    /// If the MCS index is out of bounds, this is a noop.
    pub const fn set_mcs_index(&mut self, mcs_index: u8) {
        if mcs_index <= D::MAX_HT_RATE {
            self.mcs_index = mcs_index;
        }
    }
    /// Is a short guard interval used.
    pub const fn short_gi(&self) -> bool {
        self.short_gi
    }
    /// Set if this rate uses a short guard interval.
    pub const fn set_short_gi(&mut self, short_gi: bool) {
        self.short_gi = short_gi;
    }
    /// Is a 40 MHz channel used.
    pub const fn cbw40(&self) -> bool {
        self.cbw40
    }
    /// Set if this rate uses a 40 MHz channel.
    pub const fn set_cbw40(&mut self, cbw40: bool) {
        self.cbw40 = cbw40;
    }
}
impl<D: RateDirection> Default for HtRate<D> {
    /// The default HT rate is MCS 0, long GI and 20 MHz channels.
    fn default() -> Self {
        Self {
            mcs_index: 0,
            short_gi: false,
            cbw40: false,
            _phantom: PhantomData,
        }
    }
}
impl<D: RateDirection> Rate for HtRate<D> {
    fn constellation(&self) -> Constellation {
        if self.mcs_index == 32 {
            return Constellation::Bpsk;
        }
        match self.mcs_index % 8 {
            0 => Constellation::Bpsk,
            1 | 2 => Constellation::Qpsk,
            3 | 4 => Constellation::Qam16,
            5..=7 => Constellation::Qam64,
            _ => {
                unreachable!();
            }
        }
    }
    fn symbol_interval(&self) -> usize {
        self.dft_period() + self.guard_interval_duration()
    }
    fn bandwidth(&self) -> usize {
        (if self.cbw40 { 40 } else { 20 }) * MEGA
    }
    fn data_rate(&self) -> usize {
        self.symbol_rate() * self.data_bits_per_ofdm_symbol()
    }
    fn airtime(&self, ppdu_length: usize) -> usize {
        const LONG_GI_SYMBOL_INTERVAL: usize = 4 * KILO;
        const L_TF_SYMBOLS: usize = 2;
        const L_SIG_SYMBOLS: usize = 1;
        const HT_SIG_SYMBOLS: usize = 2;
        const HT_TF_SYMBOLS: usize = 1;

        let preamble_symbols = L_TF_SYMBOLS * 2
            + L_SIG_SYMBOLS
            + HT_SIG_SYMBOLS
            + HT_TF_SYMBOLS * (1 + self.spatial_streams());

        let data_bits = 16 + ppdu_length * 8 + 6;
        let data_symbols = data_bits.div_ceil(self.data_bits_per_ofdm_symbol());
        preamble_symbols * LONG_GI_SYMBOL_INTERVAL + data_symbols * self.symbol_interval()
    }
}
impl<D: RateDirection> OfdmBasedRate for HtRate<D> {
    fn data_subcarriers(&self) -> usize {
        if self.cbw40 {
            108
        } else if self.mcs_index == 32 {
            48
        } else {
            52
        }
    }
    fn pilot_subcarriers(&self) -> usize {
        if self.cbw40 { 6 } else { 4 }
    }
    fn total_subcarriers(&self) -> usize {
        if self.cbw40 && self.mcs_index != 32 {
            128
        } else {
            64
        }
    }
    fn coding_rate(&self) -> Ratio<usize> {
        if self.mcs_index == 32 {
            return Ratio::new_raw(1, 2);
        }
        let (numer, denom) = match self.mcs_index % 8 {
            0 | 1 | 3 => (1, 2),
            5 => (2, 3),
            2 | 4 | 6 => (3, 4),
            7 => (5, 6),
            _ => {
                unreachable!();
            }
        };
        Ratio::new_raw(numer, denom)
    }
    fn spatial_streams(&self) -> usize {
        if self.mcs_index < 32 {
            self.mcs_index as usize / 8 + 1
        } else {
            match self.mcs_index {
                32 => 1,
                33..=38 => 2,
                39..=52 => 3,
                53..=76 => 4,
                _ => unreachable!(),
            }
        }
    }
    fn guard_interval_duration(&self) -> usize {
        if self.short_gi { 400 } else { 800 }
    }
}
