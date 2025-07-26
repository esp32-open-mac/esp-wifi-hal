#![no_std]
#![no_main]
use embassy_executor::Spawner;
use esp_backtrace as _;
use examples::{common_init, embassy_init, wifi_init};

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = common_init();
    embassy_init(peripherals.TIMG0);
    let _wifi = wifi_init(peripherals.WIFI, peripherals.ADC2);
}
