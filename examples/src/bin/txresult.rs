//! TX result (PMD) test.
//!
//! Alternately transmits a unicast frame to a real AP (which ACKs anything addressed to it) and
//! to a MAC address that does not exist, and prints the MAC protocol result of each attempt.
//! Expected: `Ok` for the AP and `AckTimeout` for the bogus address.
//!
//! Set `AP_MAC` (e.g. `74:4d:28:1e:be:ea`) and `CHANNEL` at build time; `ESP_LOG=trace` prints the raw PMD.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::efuse::base_mac_address;
use esp_println::println;
use esp_wifi_hal::prelude::*;
use examples::{common_init, embassy_init, get_test_channel, mk_static, wifi_init};
use ieee80211::{data_frame::builder::DataFrameBuilder, mac_parser::MACAddress, scroll::Pwrite};

fn parse_mac(s: &str) -> [u8; 6] {
    let mut mac = [0u8; 6];
    for (i, part) in s.split(':').enumerate().take(6) {
        mac[i] = u8::from_str_radix(part, 16).unwrap_or(0);
    }
    mac
}

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let peripherals = common_init();
    embassy_init(peripherals.TIMG0, peripherals.SW_INTERRUPT);
    let mut wifi = wifi_init(peripherals.WIFI);

    let channel = get_test_channel();
    let _ = wifi.set_channel(channel);
    let my_mac: [u8; 6] = base_mac_address().as_bytes().try_into().unwrap();
    let ap_mac = parse_mac(option_env!("AP_MAC").unwrap_or("74:4d:28:1e:be:ea"));
    let bogus_mac = [0x02, 0x13, 0x37, 0x42, 0x42, 0x42];

    // The ACK is expected to be addressed to interface 0, so it must carry our MAC.
    let _ = wifi.set_filter(0, RxFilterBank::ReceiverAddress, my_mac);
    let _ = wifi.set_filter_bssid_check(0, false);
    let _ = wifi.set_scanning_mode(0, ScanningMode::Disabled);

    println!("txresult: ch {channel}, ap {ap_mac:02x?}, bogus {bogus_mac:02x?}");

    let buf = mk_static!([u8; 256], [0x00u8; 256]);
    let mut round: u32 = 0;
    loop {
        for (name, dst, rts) in [
            ("ap rts", ap_mac, RtsStrategy::DriverControlled),
            ("ap norts", ap_mac, RtsStrategy::Forced(false)),
            ("bogus rts", bogus_mac, RtsStrategy::DriverControlled),
            ("bogus norts", bogus_mac, RtsStrategy::Forced(false)),
        ] {
            let written = buf
                .pwrite(
                    DataFrameBuilder::new()
                        .to_ds()
                        .category_data()
                        .payload([0x69u8; 5].as_slice())
                        .destination_address(MACAddress::new(dst))
                        .source_address(MACAddress::new(my_mac))
                        .bssid(MACAddress::new(dst))
                        .build(),
                    0,
                )
                .unwrap();
            let res = wifi
                .transmit(
                    0,
                    &TxPlcpParameters {
                        rate: OfdmRate::Mbits6.into(),
                        ..Default::default()
                    },
                    &TxMacParameters {
                        wait_for_ack: true,
                        rts_strategy: rts,
                        ..Default::default()
                    },
                    TxErrorBehaviour::Drop,
                    EdcaAccessCategory::BestEffort.into(),
                    &mut buf[..written],
                )
                .await;
            println!("#{round} {name}: {res:?}");
            Timer::after_millis(300).await;
            wifi.clear_rx_queue();
        }
        round += 1;
    }
}
