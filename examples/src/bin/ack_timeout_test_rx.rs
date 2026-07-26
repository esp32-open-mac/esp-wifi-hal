//! This is the receive side of the ACK timeout test. Here we set the ACK timeout to be 30µs, which
//! is three times the normal value.
#![no_std]
#![no_main]
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_wifi_hal::prelude::*;
use examples::{AP_ADDRESS, STA_ADDRESS, common_init, embassy_init, get_test_channel, wifi_init};

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let peripherals = common_init();
    embassy_init(peripherals.TIMG0, peripherals.SW_INTERRUPT);
    let mut wifi = wifi_init(peripherals.WIFI);

    let _ = wifi.set_channel(get_test_channel());
    let _ = wifi.set_filter(0, RxFilterBank::ReceiverAddress, STA_ADDRESS);
    let _ = wifi.set_filter(0, RxFilterBank::Bssid, AP_ADDRESS);
    loop {
        let _ = wifi.receive().await;
    }
}
