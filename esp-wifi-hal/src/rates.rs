use core::marker::PhantomData;

use num::rational::Ratio;

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
            Self::HrDsss(hr_dsss) => hr_dsss.rate + 4 * (hr_dsss.short_preamble as u8),
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

/// A Wi-Fi PHY rate for transmission.
pub type TxPhyRate = PhyRate<Tx>;
/// A Wi-Fi PHY rate for reception.
pub type RxPhyRate = PhyRate<Rx>;

const KILO: usize = 1_000;
const MEGA: usize = 1_000_000;
const GIGA: usize = 1_000_000_000;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
/// A trait implemented by all Wi-Fi rates.
pub trait Rate {
    /// Bandwidth occupied in Hz.
    fn bandwidth(&self) -> usize;
    /// The constellation used by the rate.
    ///
    /// You can use this to query the number of coded bits per symbol per subcarrier.
    fn constellation(&self) -> Constellation;
    /// Data rate in bits per second.
    ///
    /// Rational numbers are used to prevent unexpected rounding behaviour.
    fn data_rate(&self) -> Ratio<usize>;
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// A rate of the HR/DSSS PHY.
#[derive(Default)]
pub struct HrDsssRate {
    rate: u8,
    short_preamble: bool,
}
impl HrDsssRate {
    /// Construct a new HR/DSSS rate.
    ///
    /// The `rate` parameter is a value between zero and three, which maps to the rates
    /// 1 Mbit/s, 2 Mbit/s, 5.5 Mbit/s and 11 Mbit/s, in that order.
    ///
    /// [None] is returned, if `rate` is out of bounds, or 1 Mbit/s with short preamble
    /// is requested.
    pub const fn new(rate: u8, short_preamble: bool) -> Option<Self> {
        if rate > 3 || (rate == 0 && short_preamble) {
            None
        } else {
            Some(Self {
                rate,
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
    pub const fn set_rate(&mut self, rate: u8) {
        if (rate == 0 && !self.short_preamble) || rate <= 3 {
            self.rate = rate;
        }
    }
    /// Set if a short preamble is used.
    ///
    /// If you try to set 1 Mbit/s short preamble, this will be a noop, as that config
    /// is invalid.
    pub const fn set_short_preamble(&mut self, short_preamble: bool) {
        if self.rate != 0 {
            self.short_preamble = short_preamble;
        }
    }
}
impl Rate for HrDsssRate {
    fn constellation(&self) -> Constellation {
        if self.rate.is_multiple_of(2) {
            Constellation::Bpsk
        } else {
            Constellation::Qpsk
        }
    }
    fn bandwidth(&self) -> usize {
        22 * MEGA
    }
    fn data_rate(&self) -> Ratio<usize> {
        (match self.rate {
            0 => MEGA,
            1 => 2 * MEGA,
            2 => (11 * MEGA) / 2,
            3 => 11 * MEGA,
            _ => unreachable!(),
        })
        .into()
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
        (self.total_subcarriers() * KILO) / 20
    }
    /// Duration of an OFDM data symbol in ns.
    fn symbol_duration(&self) -> usize {
        self.dft_period() + self.guard_interval_duration()
    }
    /// OFDM symbol rate in Hz.
    ///
    /// For HT short GI, this is a rational number.
    fn symbol_rate(&self) -> Ratio<usize> {
        Ratio::new(GIGA, self.symbol_duration())
    }
    /// Number of coded bits per OFDM symbol.
    ///
    /// See N_CBPS in IEEE 802.11-2024.
    fn coded_bits_per_ofdm_symbol(&self) -> usize {
        self.constellation() as usize * self.data_subcarriers()
    }
    /// Number of data bits per OFDM symbol.
    ///
    /// See N_DBPS in IEEE 802.11-2024.
    fn data_bits_per_ofdm_symbol(&self) -> usize {
        // The coding rates are always chosen in a way, so that this number is an integer.
        (self.coding_rate() * self.coded_bits_per_ofdm_symbol()).to_integer()
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
impl Rate for OfdmRate {
    fn constellation(&self) -> Constellation {
        match (*self as usize) / 2 {
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
    fn data_rate(&self) -> Ratio<usize> {
        self.symbol_rate() * self.data_bits_per_ofdm_symbol()
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
        if *self == Self::Mbits48 {
            Ratio::new(2, 3)
        } else if (*self as usize).is_multiple_of(4) {
            Ratio::new(1, 2)
        } else {
            Ratio::new(3, 4)
        }
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
    // This is a deduction from the datasheet.
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
    /// The maximum HT-MCS index, the ESP32 can transmit, is 7, so [None] will be returned,
    /// if `mcs` is greater than that.
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
    /// Get the number of spatial streams for this rate.
    pub const fn spatial_streams(&self) -> usize {
        if self.mcs_index < 32 {
            self.mcs_index as usize / 8
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
    fn bandwidth(&self) -> usize {
        (if self.cbw40 { 40 } else { 20 }) * MEGA
    }
    fn data_rate(&self) -> Ratio<usize> {
        self.symbol_rate() * self.data_bits_per_ofdm_symbol()
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
        if self.cbw40 {
            6
        } else {
            4
        }
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
            return Ratio::new(1, 2);
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
        Ratio::new(numer, denom)
    }
    fn guard_interval_duration(&self) -> usize {
        if self.short_gi {
            400
        } else {
            800
        }
    }
}
