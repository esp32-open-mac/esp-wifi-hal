#![no_std]
#![no_main]
use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_wifi_hal::{TxErrorBehaviour, TxParameters, WiFiError};
use examples::{common_init, embassy_init, wifi_init, AP_ADDRESS};
use ieee80211::{data_frame::builder::DataFrameBuilder, mac_parser::BROADCAST, scroll::Pwrite};
use log::info;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}
#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let peripherals = common_init();
    embassy_init(peripherals.TIMG0);
    let wifi = wifi_init(peripherals.WIFI, peripherals.ADC2);
    let buf = mk_static!([u8; 300], [0x00u8; 300]);
    let written = buf
        .pwrite(
            DataFrameBuilder::new()
                .from_ds()
                .category_data()
                .payload([0x69u8; 5].as_slice())
                .destination_address(BROADCAST)
                .source_address(AP_ADDRESS.into())
                .bssid(AP_ADDRESS.into())
                .build(),
            0,
        )
        .unwrap();
    loop {
        let res = wifi
            .transmit(
                &mut buf[..written],
                &TxParameters {
                    ack_timeout: 10,
                    tx_error_behaviour: TxErrorBehaviour::Drop,
                    ..Default::default()
                },
                Some(0),
            )
            .await;
        match res {
            Ok(_) => info!("TX success"),
            Err(WiFiError::TxTimeout) => info!("TX timeout"),
            Err(WiFiError::TxCollision) => info!("TX collision"),
            err => info!("{err:?}"),
        }
        Timer::after_millis(500).await;
        wifi.clear_rx_queue();
    }
}
