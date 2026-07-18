use esp_wifi_rates::{HrDsssRate, HtRate, OfdmRate, Rate, Rx, Tx};

const KILO: usize = 1_000;
const MEGA: usize = 1_000_000;

#[test]
fn test_hr_dsss_data_rate_correct() {
    let correct = [1 * MEGA, 2 * MEGA, 5500 * KILO, 11 * MEGA];
    for i in 0..3 {
        assert_eq!(
            HrDsssRate::new(i as u8, false).unwrap().data_rate(),
            correct[i]
        );
    }
}
#[test]
fn test_ofdm_data_rate_correct() {
    let rates = [
        (OfdmRate::Mbits6, 6),
        (OfdmRate::Mbits9, 9),
        (OfdmRate::Mbits12, 12),
        (OfdmRate::Mbits18, 18),
        (OfdmRate::Mbits24, 24),
        (OfdmRate::Mbits36, 36),
        (OfdmRate::Mbits48, 48),
        (OfdmRate::Mbits54, 54),
    ];
    for (rate, correct_data_rate) in rates {
        assert_eq!(rate.data_rate(), correct_data_rate * MEGA);
    }
}
/// As data rates in the spec are rounded to 100 kbit/s, we use this to check that we're close enough.
fn assert_rate_close_enough(left: usize, right: usize, max_deviation: usize) {
    let delta = (left as isize - right as isize).abs() as usize;
    assert!(
        delta < max_deviation,
        "Calculated rate deviated by more than 50 kbit/s from spec value.\n|{left}-{right}|={delta}>={max_deviation}"
    );
}
#[test]
fn test_ht_data_rate_correct() {
    let correct_rates = [
        (6_500_000, 7_200_000),
        (13_000_000, 14_400_000),
        (19_500_000, 21_700_000),
        (26_000_000, 28_900_000),
        (39_000_000, 43_300_000),
        (52_000_000, 57_800_000),
        (58_500_000, 65_000_000),
        (65_000_000, 72_200_000),
    ];
    // 50 kbit/s maximum deviation for one stream.
    // We multiply this by the stream count, as higher stream count rates are technically more precise in the spec.
    const BASE_MAX_DEVIATION: usize = 50 * KILO;
    for stream_count in 1..5 {
        for (mcs_index, (lgi_rate, sgi_rate)) in correct_rates.into_iter().enumerate() {
            let mut rate =
                HtRate::<Rx>::new_lgi_ht20((mcs_index + 8 * (stream_count - 1)) as u8).unwrap();

            let max_deviation = BASE_MAX_DEVIATION * stream_count;
            assert_rate_close_enough(rate.data_rate(), lgi_rate * stream_count, max_deviation);
            rate.set_short_gi(true);
            assert_rate_close_enough(rate.data_rate(), sgi_rate * stream_count, max_deviation);
        }
    }
    // We run only one test with a 40 MHz channel width, since all that should change is the number of subcarriers.
    assert_rate_close_enough(
        HtRate::<Rx>::new(7, true, true).unwrap().data_rate(),
        150 * MEGA,
        BASE_MAX_DEVIATION,
    );

    assert_rate_close_enough(
        HtRate::<Rx>::new_lgi_ht20(32).unwrap().data_rate(),
        6 * MEGA,
        BASE_MAX_DEVIATION,
    );
}
