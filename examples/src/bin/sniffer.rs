//! Passive sniffer: puts interface 0 into scanning mode and prints every received frame.
//!
//! Used as the first hardware test of the ESP32-C3 port: if beacons show up here, the RX path
//! (PHY init, MAC init, DMA list, filters and the MAC interrupt) works.
#![no_std]
#![no_main]
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_println::println;
use esp_wifi_hal::prelude::*;
use examples::{common_init, embassy_init, get_test_channel, wifi_init};
use ieee80211::{mgmt_frame::BeaconFrame, scroll::Pread};

fn fmt_addr(b: &[u8]) -> [u8; 17] {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = [b':'; 17];
    for (i, byte) in b.iter().take(6).enumerate() {
        out[i * 3] = HEX[(byte >> 4) as usize];
        out[i * 3 + 1] = HEX[(byte & 0xf) as usize];
    }
    out
}

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let peripherals = common_init();
    embassy_init(peripherals.TIMG0, peripherals.SW_INTERRUPT);
    let mut wifi = wifi_init(peripherals.WIFI);

    let channel = get_test_channel();
    let _ = wifi.set_channel(channel);
    let _ = wifi.set_scanning_mode(0, ScanningMode::ManagementAndData);
    println!("sniffer: listening on channel {channel}");

    let mut count: u32 = 0;
    loop {
        let received = wifi.receive().await;
        count += 1;
        let mpdu = received.mpdu_buffer();
        if mpdu.len() < 24 {
            println!("#{count} short frame len={} rssi={}", mpdu.len(), received.rssi());
            continue;
        }
        let fc = mpdu[0];
        let ftype = (fc >> 2) & 3;
        let subtype = fc >> 4;
        let a1 = fmt_addr(&mpdu[4..10]);
        let a2 = fmt_addr(&mpdu[10..16]);
        let a3 = fmt_addr(&mpdu[16..22]);
        let a1 = core::str::from_utf8(&a1).unwrap();
        let a2 = core::str::from_utf8(&a2).unwrap();
        let a3 = core::str::from_utf8(&a3).unwrap();
        if ftype == 0 && subtype == 8 {
            let ssid = mpdu
                .pread::<BeaconFrame>(0)
                .ok()
                .and_then(|b| b.ssid())
                .unwrap_or("<no ssid>");
            println!(
                "#{count} beacon bssid={a3} ssid={ssid:?} rssi={} rate={:?} len={}",
                received.rssi(),
                received.phy_rate(),
                mpdu.len()
            );
        } else {
            println!(
                "#{count} type={ftype} subtype={subtype} a1={a1} a2={a2} a3={a3} rssi={} len={}",
                received.rssi(),
                mpdu.len()
            );
        }
    }
}
