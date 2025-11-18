#![no_std]
#![no_main]
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_wifi_hal::ScanningMode;
use examples::{common_init, embassy_init, wifi_init};
use ieee80211::GenericFrame;
use log::info;

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let peripherals = common_init();
    embassy_init(peripherals.TIMG0);
    let wifi = wifi_init(peripherals.WIFI, peripherals.ADC2);
    wifi.set_scanning_mode(0, ScanningMode::BeaconsOnly);
    loop {
        let received = wifi.receive().await;
        let Ok(generic_frame) = GenericFrame::new(received.mpdu_buffer(), false) else {
            continue;
        };
        info!("IN: {:05}", received.corrected_timestamp().as_micros());
        /*
        info!(
            "Duration: {:03}s Delta: {:08}µs Type: {:?}",
            generic_frame.duration(),
            received.corrected_timestamp().as_micros() as i32 - wifi.mac_time() as i32,
            generic_frame.frame_control_field().frame_type()
        );
*/
    }
}
