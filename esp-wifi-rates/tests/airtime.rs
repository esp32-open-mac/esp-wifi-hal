use esp_wifi_rates::{HrDsssRate, HtRate, OfdmRate, Rate, Tx};

const KILO: usize = 1_000;
const MEGA: usize = 1_000_000;
#[test]
fn test_dsss_airtime() {
    assert_eq!(HrDsssRate::LONG_1M.airtime(14), (192 + 14 * 8) * KILO);
}
#[test]
fn test_ofdm_airtime() {
    assert_eq!(OfdmRate::Mbits6.airtime(28), 64 * KILO);
}
#[test]
fn test_ht_airtime() {
    assert_eq!(
        HtRate::<Tx>::new_lgi_ht20(0).unwrap().airtime(1500),
        1888 * KILO
    );
}
