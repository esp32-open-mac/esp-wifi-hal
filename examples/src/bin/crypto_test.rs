#![no_std]
#![no_main]
use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;
use esp_wifi_hal::{
    AesCipherParameters, CipherParameters, WiFiResources, KeyType, MultiLengthKey, RxFilterBank,
    TxErrorBehaviour, TxParameters, WiFi,
};
use ieee80211::{
    crypto::CryptoHeader,
    data_frame::{builder::DataFrameBuilder, DataFrame},
    mac_parser::MACAddress,
    scroll::Pwrite,
};
use static_cell::StaticCell;

const STA_ADDRESS: [u8; 6] = [0x00, 0x80, 0x41, 0x13, 0x37, 0x42];
const OUR_ADDRESS: [u8; 6] = [0x00, 0x80, 0x41, 0x13, 0x37, 0x69];

const PAYLOAD: &[u8] = &[0x69; 10];

const GROUP_ADDRESSED_TEMPLATE: DataFrame<'static, &[u8]> = DataFrameBuilder::new()
    .from_ds()
    .category_data()
    .payload(PAYLOAD)
    .destination_address(MACAddress([0x01, 0x00, 0x5e, 0x00, 0x00, 0x16]))
    .source_address(MACAddress(OUR_ADDRESS))
    .bssid(MACAddress(OUR_ADDRESS))
    .build();
const PAIRWISE_TEMPLATE: DataFrame<'static, &[u8]> = DataFrameBuilder::new()
    .from_ds()
    .category_data()
    .payload(PAYLOAD)
    .destination_address(MACAddress(STA_ADDRESS))
    .source_address(MACAddress(OUR_ADDRESS))
    .bssid(MACAddress(OUR_ADDRESS))
    .build();

const GTK: [u8; 16] = [0xbb; 16];
const GTK_KEY_SLOT: usize = 0;
const PTK: [u8; 16] = [0xaa; 16];
const PTK_KEY_SLOT: usize = 1;

static DMA_RESOURCES: StaticCell<WiFiResources<10>> = StaticCell::new();

fn insert_key(wifi: &WiFi, key: &[u8; 16], key_type: KeyType, address: [u8; 6], key_slot: usize) {
    let cipher_parameters = CipherParameters::Ccmp(AesCipherParameters {
        key: MultiLengthKey::Short(key),
        key_type,
        mfp_enabled: false,
        spp_enabled: false,
    });
    wifi.set_key(key_slot, 0, 0, address, cipher_parameters)
        .unwrap();
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_println::logger::init_logger_from_env();

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);
    let dma_resources = DMA_RESOURCES.init(WiFiResources::new());
    let wifi = WiFi::new(
        peripherals.WIFI,
        peripherals.RADIO_CLK,
        peripherals.ADC2,
        dma_resources,
    );

    let _ = wifi.set_filter(RxFilterBank::ReceiverAddress, 0, OUR_ADDRESS, [0xff; 6]);
    let _ = wifi.set_filter_status(RxFilterBank::ReceiverAddress, 0, true);
    insert_key(&wifi, &GTK, KeyType::Group, OUR_ADDRESS, GTK_KEY_SLOT);
    insert_key(&wifi, &PTK, KeyType::Pairwise, STA_ADDRESS, PTK_KEY_SLOT);

    for pn in 0.. {
        let (template, key_slot) = if pn % 2 == 0 {
            (GROUP_ADDRESSED_TEMPLATE, GTK_KEY_SLOT)
        } else {
            (PAIRWISE_TEMPLATE, PTK_KEY_SLOT)
        };
        let crypto_header = CryptoHeader::new(pn, 0).unwrap();
        let mut buf = [0x00u8; 300];
        let len = buf.pwrite(template.crypto_wrap(crypto_header, false), 0).unwrap();
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
