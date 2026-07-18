#[cfg(test)]
mod constellation {
    use esp_wifi_rates::{Constellation, HrDsssRate, HtRate, OfdmRate, Rate, Tx};

    #[test]
    fn test_hr_dsss() {
        let constellations = [
            Constellation::Bpsk,
            Constellation::Qpsk,
            Constellation::Bpsk,
            Constellation::Qpsk,
        ];
        for (rate_index, constellation) in constellations.into_iter().enumerate() {
            assert_eq!(
                HrDsssRate::new(rate_index as u8, false)
                    .unwrap()
                    .constellation(),
                constellation,
                "Constellations don't match."
            );
        }
    }
    #[test]
    fn test_ofdm() {
        let constellations = [
            (OfdmRate::Mbits6, Constellation::Bpsk),
            (OfdmRate::Mbits9, Constellation::Bpsk),
            (OfdmRate::Mbits12, Constellation::Qpsk),
            (OfdmRate::Mbits18, Constellation::Qpsk),
            (OfdmRate::Mbits24, Constellation::Qam16),
            (OfdmRate::Mbits36, Constellation::Qam16),
            (OfdmRate::Mbits48, Constellation::Qam64),
            (OfdmRate::Mbits54, Constellation::Qam64),
        ];
        for (rate, constellation) in constellations {
            assert_eq!(
                rate.constellation(),
                constellation,
                "Constellations don't match."
            );
        }
    }
    #[test]
    fn test_ht() {
        let constellations = [
            Constellation::Bpsk,
            Constellation::Qpsk,
            Constellation::Qpsk,
            Constellation::Qam16,
            Constellation::Qam16,
            Constellation::Qam64,
            Constellation::Qam64,
            Constellation::Qam64,
        ];
        for (mcs_index, constellation) in constellations.into_iter().enumerate() {
            assert_eq!(
                HtRate::<Tx>::new_lgi_ht20(mcs_index as u8)
                    .unwrap()
                    .constellation(),
                constellation,
                "Constellations don't match."
            );
        }
    }
}
