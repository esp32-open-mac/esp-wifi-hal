use esp_wifi_rates::{HrDsssRate, HtRate, Rx, Tx};

#[test]
fn test_dsss_1m_short_rejected() {
    assert!(
        HrDsssRate::new(0, true).is_none(),
        "HR/DSSS: 1 Mbit/s short preamble is invalid."
    );

    let mut temp = HrDsssRate::LONG_1M;
    temp.set_short_preamble(true);
    assert_eq!(
        HrDsssRate::LONG_1M,
        temp,
        "HR/DSSS: Setting short preamble on a 1 Mbit/s rate should fail."
    );

    temp = HrDsssRate::SHORT_2M;
    temp.set_rate(0);
    assert_eq!(
        HrDsssRate::SHORT_2M,
        temp,
        "HR/DSSS: Setting the rate index, for 2 Mbit/s short preamble, to 0 should fail."
    );
}
#[test]
fn test_ht_tx_rate_range() {
    // We test all MCS indices defined in the standard.
    for mcs in 0..8 {
        assert!(
            HtRate::<Tx>::new_lgi_ht20(mcs).is_some(),
            "HT: MCS {mcs} should be valid for TX."
        );
    }
    for mcs in 8..33 {
        assert!(
            HtRate::<Tx>::new_lgi_ht20(mcs).is_none(),
            "HT: MCS {mcs} should be invalid for TX."
        );
    }
}
#[test]
fn test_ht_rx_rate_range() {
    // We test all MCS indices defined in the standard.
    for mcs in 0..33 {
        assert!(
            HtRate::<Rx>::new_lgi_ht20(mcs).is_some(),
            "HT: MCS {mcs} should be valid for RX."
        );
    }
    for mcs in 33..77 {
        assert!(
            HtRate::<Rx>::new_lgi_ht20(mcs).is_none(),
            "HT: MCS {mcs} should be invalid for RX."
        );
    }
}
