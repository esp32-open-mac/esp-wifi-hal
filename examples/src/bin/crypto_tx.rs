#![no_std]
#![no_main]
use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;
use esp_wifi_hal::{
    WiFiResources, KeyType, 
    TxErrorBehaviour, TxParameters, WiFi,
};
use examples::{get_test_channel, insert_key, setup_filters, AP_ADDRESS, GTK, GTK_KEY_SLOT, PTK, PTK_KEY_SLOT, STA_ADDRESS};

use ieee80211::{
    crypto::{CryptoHeader, MicState},
    data_frame::{builder::DataFrameBuilder, DataFrame},
    mac_parser::MACAddress,
    scroll::Pwrite,
};
use static_cell::StaticCell;


const PAYLOAD: &[u8] = &[0x69; 2];

const GROUP_ADDRESSED_TEMPLATE: DataFrame<'static, &[u8]> = DataFrameBuilder::new()
    .from_ds()
    .category_data()
    .payload(PAYLOAD)
    .destination_address(MACAddress([0x01, 0x00, 0x5e, 0x00, 0x00, 0x16]))
    .source_address(MACAddress(AP_ADDRESS))
    .bssid(MACAddress(AP_ADDRESS))
    .build();
const PAIRWISE_TEMPLATE: DataFrame<'static, &[u8]> = DataFrameBuilder::new()
    .from_ds()
    .category_data()
    .payload(PAYLOAD)
    .destination_address(MACAddress(STA_ADDRESS))
    .source_address(MACAddress(AP_ADDRESS))
    .bssid(MACAddress(AP_ADDRESS))
    .build();

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
    setup_filters(&wifi, AP_ADDRESS, AP_ADDRESS);

    insert_key(&wifi, &GTK, KeyType::Group, AP_ADDRESS, GTK_KEY_SLOT);
    insert_key(&wifi, &PTK, KeyType::Pairwise, STA_ADDRESS, PTK_KEY_SLOT);

    for pn in 0.. {
        let (frame, key_slot) = if pn % 2 == 0 {
            (GROUP_ADDRESSED_TEMPLATE, GTK_KEY_SLOT)
        } else {
            (PAIRWISE_TEMPLATE, PTK_KEY_SLOT)
        };
        let frame = frame.crypto_wrap(CryptoHeader::new(pn, 0).unwrap(), MicState::Short);
        let mut buf = [0x00u8; 300];
        let len = buf.pwrite(frame, 0).unwrap();
        let _ = wifi
            .transmit(
                &mut buf[..len],
                &TxParameters {
                    key_slot: Some(key_slot),
                    tx_error_behaviour: TxErrorBehaviour::Drop,
                    override_seq_num: true,
                    ..Default::default()
                },
                Some(0),
            )
            .await;
        Timer::after_secs(1).await;
    }
}
