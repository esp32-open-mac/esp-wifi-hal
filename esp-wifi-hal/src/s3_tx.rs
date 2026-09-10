//! S3 HT20 encodings checked against `mac_tx_set_plcp1` and `mac_tx_set_htsig`.
//! Keep these independent of MMIO so the encoding boundaries can be tested on a host.

pub(crate) const fn ht_rate(mcs: u8, short_gi: bool) -> u8 {
    assert!(mcs < 8);
    0x10 + mcs + 8 * short_gi as u8
}

pub(crate) const fn ht_sig(mcs: u8, short_gi: bool, length: usize) -> u32 {
    assert!(mcs < 8);
    // Bit 7 is not the short-GI bit on S3. The blob selects 0x87/0x07 in
    // the high byte for rate codes 24..31/16..23 respectively (HT20, no STBC).
    mcs as u32 | ((length as u32 & 0xffff) << 8) | 0x07000000 | ((short_gi as u32) << 31)
}

pub(crate) const fn tx_misc(current: u32, response_rate: u8, interface: usize) -> u32 {
    assert!(interface < 4);
    (current & !((0xff << 6) | (0x3 << 28)))
        | (1 << 5)
        | ((response_rate as u32) << 6)
        | ((interface as u32) << 28)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn long_and_short_gi_use_disjoint_rate_ranges() {
        for mcs in 0..8 {
            assert_eq!(ht_rate(mcs, false), 16 + mcs);
            assert_eq!(ht_rate(mcs, true), 24 + mcs);
        }
    }

    #[test]
    fn short_gi_only_changes_the_high_sig_bit() {
        assert_eq!(ht_sig(0, false, 1500), 0x0705dc00);
        assert_eq!(ht_sig(7, true, 1500), 0x8705dc07);
        for mcs in 0..8 {
            for length in [0, 24, 1500, 4095, 65535] {
                let long = ht_sig(mcs, false, length);
                let short = ht_sig(mcs, true, length);
                assert_eq!(long ^ short, 1 << 31);
                assert_eq!(short & (1 << 7), 0);
                assert_eq!((short >> 8) & 0xffff, length as u32);
            }
        }
    }

    #[test]
    fn replacing_response_and_interface_preserves_other_slot_bits() {
        let changed = (0xff << 6) | (3 << 28) | (1 << 5);
        for current in [0, u32::MAX, 0x00400020, 0xa5a5a5a5] {
            for interface in 0..4 {
                for rate in [0, 3, 9, 11] {
                    let value = tx_misc(current, rate, interface);
                    assert_eq!(value & !changed, current & !changed);
                    assert_eq!((value >> 6) & 0xff, rate as u32);
                    assert_eq!((value >> 28) & 3, interface as u32);
                    assert_ne!(value & (1 << 5), 0);
                }
            }
        }
    }
}
