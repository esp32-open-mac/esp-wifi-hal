#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
/// Physical layer mode.
pub enum PhyMode {
    /// Direct-Sequence-Spread-Spectrum (DSSS)
    ///
    /// 1 Mbit/s (DBPSK) and 2 Mbit/s (DQPSK), with long and short preamble.
    /// See IEEE 802.11-2020 Clause 15.
    Dsss,
    /// Complementary Code Keying (CCK)
    ///
    /// 5.5 Mbit/s (4 bits per CCK chip) and 11 Mbit/s (8 bits per CCK chip) at a chipping rate of
    /// 11 Mchip/s, with long and short preamble.
    /// See IEEE 802.11-2020 Clause 16.
    Cck,
    /// Orthogonal Frequency Division Multiplexing (OFDM)
    ///
    /// 6, 9, 12, 18, 24, 36, 48 and 54 Mbit/s, using 48 data subcarriers modulated with BPSK, QPSK,
    /// 16-QAM and 64-QAM, at various code rates. Only the "full-clocked" mode with 20 MHz channel
    /// spacing is supported.
    /// See IEEE 802.11-2020 Clause 17.
    Ofdm,
    /// High Throughput (HT)
    ///
    /// Based on the OFDM PHY, with MCS indices 0-7. Long and Short GI rates are supported.
    /// See IEEE 802.11-2020 Clause 19.
    Ht,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[repr(u8)]
#[allow(missing_docs)]
/// The rate used by the PHY.
pub enum WiFiRate {
    #[default]
    PhyRate1ML = 0x00,
    PhyRate2ML = 0x01,
    PhyRate5ML = 0x02,
    PhyRate11ML = 0x03,
    PhyRate2MS = 0x05,
    PhyRate5MS = 0x06,
    PhyRate11MS = 0x07,
    PhyRate48M = 0x08,
    PhyRate24M = 0x09,
    PhyRate12M = 0x0a,
    PhyRate6M = 0x0b,
    PhyRate54M = 0x0c,
    PhyRate36M = 0x0d,
    PhyRate18M = 0x0e,
    PhyRate9M = 0x0f,
    PhyRateMCS0LGI = 0x10,
    PhyRateMCS1LGI = 0x11,
    PhyRateMCS2LGI = 0x12,
    PhyRateMCS3LGI = 0x13,
    PhyRateMCS4LGI = 0x14,
    PhyRateMCS5LGI = 0x15,
    PhyRateMCS6LGI = 0x16,
    PhyRateMCS7LGI = 0x17,
    PhyRateMCS0SGI = 0x18,
    PhyRateMCS1SGI = 0x19,
    PhyRateMCS2SGI = 0x1a,
    PhyRateMCS3SGI = 0x1b,
    PhyRateMCS4SGI = 0x1c,
    PhyRateMCS5SGI = 0x1d,
    PhyRateMCS6SGI = 0x1e,
    PhyRateMCS7SGI = 0x1f,
}
impl WiFiRate {
    #[inline]
    /// Check if the rate is using the HT PHY.
    pub const fn is_ht(&self) -> bool {
        *self as u8 >= 0x10
    }
    #[inline]
    /// Check if the rate uses a short guard interval.
    pub const fn is_short_gi(&self) -> bool {
        *self as u8 >= 0x18
    }
    /// Get the PHY mode of the rate.
    pub const fn phy_mode(&self) -> PhyMode {
        match *self as u8 {
            0..=1 | 4..=5 => PhyMode::Dsss,
            2..=3 | 6..=7 => PhyMode::Cck,
            8..=15 => PhyMode::Ofdm,
            // All remaining valid rates are HT.
            _ => PhyMode::Ht,
        }
    }
    /// Get the parameters of an HT rate.
    ///
    /// The returned tuple is the MCS index and a [bool] indicating short GI.
    pub const fn ht_paramters(&self) -> Option<(u8, bool)> {
        if self.is_ht() {
            Some((*self as u8, self.is_short_gi()))
        } else {
            None
        }
    }
}
// This is a lookup-table for matching index->WiFiRate.
pub(crate) static RATE_LUT: &[WiFiRate] = &[
    WiFiRate::PhyRate1ML,
    WiFiRate::PhyRate2ML,
    WiFiRate::PhyRate5ML,
    WiFiRate::PhyRate11ML,
    WiFiRate::PhyRate2MS,
    WiFiRate::PhyRate5MS,
    WiFiRate::PhyRate11MS,
    WiFiRate::PhyRate48M,
    WiFiRate::PhyRate24M,
    WiFiRate::PhyRate12M,
    WiFiRate::PhyRate6M,
    WiFiRate::PhyRate54M,
    WiFiRate::PhyRate36M,
    WiFiRate::PhyRate18M,
    WiFiRate::PhyRate9M,
    WiFiRate::PhyRateMCS0LGI,
    WiFiRate::PhyRateMCS1LGI,
    WiFiRate::PhyRateMCS2LGI,
    WiFiRate::PhyRateMCS3LGI,
    WiFiRate::PhyRateMCS4LGI,
    WiFiRate::PhyRateMCS5LGI,
    WiFiRate::PhyRateMCS6LGI,
    WiFiRate::PhyRateMCS7LGI,
    WiFiRate::PhyRateMCS0SGI,
    WiFiRate::PhyRateMCS1SGI,
    WiFiRate::PhyRateMCS2SGI,
    WiFiRate::PhyRateMCS3SGI,
    WiFiRate::PhyRateMCS4SGI,
    WiFiRate::PhyRateMCS5SGI,
    WiFiRate::PhyRateMCS6SGI,
    WiFiRate::PhyRateMCS7SGI,
];
