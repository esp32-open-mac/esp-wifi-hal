#![no_std]
#![no_main]
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_wifi_hal::KeyType;
use examples::{
    common_init, embassy_init, get_test_channel, insert_key, setup_filters, wifi_init, AP_ADDRESS,
    GTK, GTK_KEY_SLOT, PTK, PTK_KEY_SLOT, STA_ADDRESS,
};
use ieee80211::{crypto::MicState, data_frame::DataFrame, match_frames};
use log::info;

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let peripherals = common_init();
    embassy_init(peripherals.TIMG0);
    let wifi = wifi_init(peripherals.WIFI, peripherals.ADC2);

    let _ = wifi.set_channel(get_test_channel());

    info!("RX");
    setup_filters(&wifi, STA_ADDRESS, AP_ADDRESS);
    insert_key(&wifi, &GTK, KeyType::Group, AP_ADDRESS, GTK_KEY_SLOT);
    insert_key(&wifi, &PTK, KeyType::Pairwise, AP_ADDRESS, PTK_KEY_SLOT);

    loop {
        let received = wifi.receive().await;
        let _ = match_frames! {
            received.mpdu_buffer(),
            data_frame = DataFrame => {
                let Some(payload) = data_frame.potentially_wrapped_payload(Some(MicState::NotPresent)) else {
                    continue;
                };
                info!("Payload: {:02x?}", payload.payload());
            }
        };
    }
}
