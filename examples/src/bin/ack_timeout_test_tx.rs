//! This tests, whether the timeout parameter actually configures the ACK timeout, by transmitting
//! an ACK in software well after one SIFS, so the default value would cause an ACK timeout.
//! SPOILER: It does...
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_wifi_hal::TxParameters;
use examples::{
    common_init, embassy_init, get_test_channel, setup_filters, wifi_init, AP_ADDRESS, STA_ADDRESS,
};
use ieee80211::{data_frame::builder::DataFrameBuilder, scroll::Pwrite};
use log::info;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}
#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = common_init();
    embassy_init(peripherals.TIMG0);
    let wifi = wifi_init(peripherals.WIFI, peripherals.ADC2);

    let _ = wifi.set_channel(get_test_channel());
    setup_filters(&wifi, AP_ADDRESS, AP_ADDRESS);

    let buf = mk_static!([u8; 300], [0x00u8; 300]);
    let written = buf
        .pwrite(
            DataFrameBuilder::new()
                .from_ds()
                .category_data()
                .payload([0x69u8; 5].as_slice())
                .destination_address(STA_ADDRESS.into())
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
                    ..Default::default()
                },
                Some(0),
            )
            .await;
        if let Err(err) = res {
            info!("TX error: {err:?}");
        } else {
            info!("Ack arrived in time.");
        }
        Timer::after_millis(500).await;
        wifi.clear_rx_queue();
    }
}
