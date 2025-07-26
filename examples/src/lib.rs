//! Test parameters for the individual examples.
#![no_std]

use esp_hal::{
    peripherals::{Peripherals, ADC2, TIMG0, WIFI},
    timer::timg::TimerGroup,
};
use esp_wifi_hal::{
    AesCipherParameters, CipherParameters, KeyType, MultiLengthKey, RxFilterBank, WiFi,
    WiFiResources,
};
use ieee80211::mac_parser::MACAddress;
use log::info;
use static_cell::StaticCell;

// These are some common defaults.

pub const STA_ADDRESS: [u8; 6] = [0x00, 0x80, 0x41, 0x13, 0x37, 0x42];
pub const AP_ADDRESS: [u8; 6] = [0x00, 0x80, 0x41, 0x13, 0x37, 0x69];

pub const KEY_ID: u8 = 0;
pub const GTK: [u8; 16] = [0xbb; 16];
pub const GTK_KEY_SLOT: usize = 0;
pub const PTK: [u8; 16] = [0xaa; 16];
pub const PTK_KEY_SLOT: usize = 1;

/// Returns the channel used for testing.
pub fn get_test_channel() -> u8 {
    option_env!("CHANNEL")
        .map(str::parse::<u8>)
        .and_then(Result::ok)
        .unwrap_or(1)
}
/// Utility to print a key.
pub fn print_key<'buf>(key: &[u8; 16], buf: &'buf mut [u8; 32]) -> &'buf str {
    for (key_byte, mut buf_chunk) in key.iter().copied().zip(buf.chunks_mut(2)) {
        use embedded_io::Write;
        let _ = core::write!(buf_chunk, "{key_byte:02x}");
    }
    core::str::from_utf8(buf.as_slice()).unwrap()
}
/// Utility to set and enable the filters.
pub fn setup_filters(wifi: &WiFi, ra: [u8; 6], bssid: [u8; 6]) {
    let _ = wifi.set_filter(RxFilterBank::ReceiverAddress, 0, ra, [0xff; 6]);
    let _ = wifi.set_filter(RxFilterBank::BSSID, 0, bssid, [0xff; 6]);
    let _ = wifi.set_filter_status(RxFilterBank::ReceiverAddress, 0, true);
    let _ = wifi.set_filter_status(RxFilterBank::BSSID, 0, true);
}
/// Utility to set a key, with some basic parameters.
pub fn insert_key(
    wifi: &WiFi,
    key: &[u8; 16],
    key_type: KeyType,
    address: [u8; 6],
    key_slot: usize,
) {
    let cipher_parameters = CipherParameters::Ccmp(AesCipherParameters {
        key: MultiLengthKey::Short(key),
        key_type,
        mfp_enabled: false,
        spp_enabled: false,
    });
    wifi.set_key(key_slot, 0, 0, address, cipher_parameters)
        .unwrap();
    let mut buf = [0x00u8; 32];
    info!(
        "Using \'{}\' as {} for {}.",
        print_key(key, &mut buf),
        if key_type == KeyType::Group {
            "GTK"
        } else {
            "PTK"
        },
        MACAddress(address)
    );
}
pub fn common_init() -> Peripherals {
    esp_bootloader_esp_idf::esp_app_desc!();
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_println::logger::init_logger_from_env();

    peripherals
}
pub fn embassy_init(timg0: TIMG0<'static>) {
    let timg0 = TimerGroup::<'static>::new(timg0);
    esp_hal_embassy::init(timg0.timer0);
}
pub fn wifi_init<'a>(wifi: WIFI<'a>, adc2: ADC2<'a>) -> WiFi<'a> {
    static WIFI_RESOURCES: StaticCell<WiFiResources<10>> = StaticCell::new();
    WiFi::new(wifi, adc2, WIFI_RESOURCES.init(WiFiResources::new()))
}
