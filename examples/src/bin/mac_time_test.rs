#![no_std]
#![no_main]
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_wifi_hal::prelude::*;
use examples::{common_init, embassy_init, wifi_init};
use log::info;

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let peripherals = common_init();
    embassy_init(peripherals.TIMG0, peripherals.SW_INTERRUPT);
    let mut wifi = wifi_init(peripherals.WIFI);
    let _ = wifi.set_scanning_mode(0, ScanningMode::BeaconsOnly);
    let mut buffers = [const { None::<BorrowedBuffer<'static>> }; 10];
    for buffer in buffers.iter_mut() {
        let received = wifi.receive().await;
        let _ = buffer.insert(received);
    }
    core::mem::drop(buffers);
    loop {
        let received = wifi.receive().await;
        info!("Received {} bytes", received.mpdu_buffer().len());
    }
}
