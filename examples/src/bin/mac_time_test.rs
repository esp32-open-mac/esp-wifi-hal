#![no_std]
#![no_main]
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_wifi_hal::{ll::LowLevelDriver, BorrowedBuffer, ScanningMode, AsyncReceive};
use examples::{common_init, embassy_init, wifi_init};
use ieee80211::GenericFrame;
use log::info;

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let peripherals = common_init();
    embassy_init(peripherals.TIMG0);
    let mut wifi = wifi_init(peripherals.WIFI, peripherals.ADC2);
    let _ = wifi.set_scanning_mode(0, ScanningMode::BeaconsOnly);
    let mut buffers = [const { None::<BorrowedBuffer<'static>> }; 10];
    for buffer in buffers.iter_mut() {
        let received = wifi.receive().await;
        let _ = buffer.insert(received);
    }
    unsafe {
        LowLevelDriver::set_rx_enable(false);
        LowLevelDriver::set_rx_enable(true);
    }
    core::mem::drop(buffers);
    loop {
        let received = wifi.receive().await;
        info!("Received {} bytes", received.mpdu_buffer().len());
    }
}
