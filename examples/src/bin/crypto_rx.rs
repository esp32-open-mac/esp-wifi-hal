#![no_std]
#![no_main]
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;
use esp_wifi_hal::{KeyType, WiFi, WiFiResources};
use examples::{
    get_test_channel, insert_key, setup_filters, AP_ADDRESS, GTK, GTK_KEY_SLOT, PTK, PTK_KEY_SLOT,
    STA_ADDRESS,
};
use ieee80211::{crypto::MicState, data_frame::DataFrame, match_frames};
use log::info;
use static_cell::StaticCell;

static WIFI_RESOURCES: StaticCell<WiFiResources<10>> = StaticCell::new();

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_println::logger::init_logger_from_env();

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);
    let dma_resources = WIFI_RESOURCES.init(WiFiResources::new());
    let wifi = WiFi::new(
        peripherals.WIFI,
        peripherals.RADIO_CLK,
        peripherals.ADC2,
        dma_resources,
    );
    let _ = wifi.set_channel(get_test_channel());

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
