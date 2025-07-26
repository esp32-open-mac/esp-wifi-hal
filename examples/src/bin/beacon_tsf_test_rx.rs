//! This is the receive side of the ACK timeout test. Here we set the ACK timeout to be 30µs, which
//! is three times the normal value.
#![no_std]
#![no_main]
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_println::println;
use esp_wifi_hal::ScanningMode;
use examples::{common_init, embassy_init, wifi_init};
use ieee80211::{
    elements::VendorSpecificElement,
    mgmt_frame::{body::HasElements, BeaconFrame},
    scroll::Pread,
};

const SSID: &str = "BEACON TSF HIL TEST";
#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = common_init();
    embassy_init(peripherals.TIMG0);
    let wifi = wifi_init(peripherals.WIFI, peripherals.ADC2);

    let _ = wifi.set_scanning_mode(0, ScanningMode::BeaconsOnly);
    let mut non_zero_timestamps = false;
    loop {
        let received = wifi.receive().await;
        let Ok(beacon_frame) = received.mpdu_buffer().pread::<BeaconFrame>(0) else {
            continue;
        };
        let Some(ssid) = beacon_frame.ssid() else {
            continue;
        };
        let Some(vendor_specific) = beacon_frame
            .get_elements()
            .get_first_element::<VendorSpecificElement>()
        else {
            continue;
        };
        let interface = vendor_specific.get_payload()[3];
        if ssid != SSID {
            continue;
        }
        let current_timestamp_non_zero = beacon_frame.timestamp != 0;
        if current_timestamp_non_zero {
            println!("Receiving non zeroed timestamps from TX interface: {interface}.");
        } else {
            println!("Receiving zeroed timestamps from TX interface {interface}.");
        }
        if current_timestamp_non_zero == non_zero_timestamps {
            continue;
        }
        non_zero_timestamps = current_timestamp_non_zero;
    }
}
