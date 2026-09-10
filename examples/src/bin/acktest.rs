//! RX + hardware auto-ACK test for the ESP32-C3.
//!
//! Installs the module's own MAC address as the receiver-address filter on interface 0 and then
//! just receives. When a unicast frame addressed to this MAC arrives, the MAC hardware is expected
//! to auto-ACK it. Inject such a frame from another device and watch for the ACK on that device.
#![no_std]
#![no_main]
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::efuse::base_mac_address;
use esp_println::println;
use esp_wifi_hal::prelude::*;
use examples::{common_init, embassy_init, get_test_channel, wifi_init};

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let peripherals = common_init();
    embassy_init(peripherals.TIMG0, peripherals.SW_INTERRUPT);
    let mut wifi = wifi_init(peripherals.WIFI);

    let mac: [u8; 6] = base_mac_address().as_bytes().try_into().unwrap();
    let channel = get_test_channel();
    let _ = wifi.set_channel(channel);

    // Only frames whose receiver address matches ours pass the filter, so the hardware treats
    // them as "for us" and ACKs them. No BSSID requirement.
    let _ = wifi.set_filter(0, RxFilterBank::ReceiverAddress, mac);
    let _ = wifi.set_filter_bssid_check(0, false);
    let _ = wifi.set_scanning_mode(0, ScanningMode::Disabled);

    println!(
        "acktest: ch {channel}, my mac {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
    );

    let mut count: u32 = 0;
    loop {
        let received = wifi.receive().await;
        count += 1;
        let mpdu = received.mpdu_buffer();
        let fc = mpdu.first().copied().unwrap_or(0);
        println!(
            "#{count} fc={fc:02x} rssi={} rate={:?} len={} hdr={:02x?}",
            received.rssi(),
            received.phy_rate(),
            mpdu.len(),
            &mpdu[..mpdu.len().min(24)]
        );
    }
}
