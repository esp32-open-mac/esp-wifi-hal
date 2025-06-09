//! This is the receive side of the ACK timeout test. Here we set the ACK timeout to be 30µs, which
//! is three times the normal value.
#![no_std]
#![no_main]
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;
use esp_wifi_hal::{WiFiResources, TxParameters, WiFi};
use examples::get_test_channel;
use ieee80211::{mac_parser::MACAddress, GenericFrame};
use log::info;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}
const OTHER_MAC_ADDRESS: MACAddress = MACAddress::new([0x00, 0x80, 0x41, 0x13, 0x37, 0x42]);
const OWN_MAC_ADDRESS: MACAddress = MACAddress::new([0x00, 0x80, 0x41, 0x13, 0x37, 0x41]);
#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_println::logger::init_logger_from_env();

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let wifi_resources = mk_static!(WiFiResources<10>, WiFiResources::new());

    let wifi = WiFi::new(
        peripherals.WIFI,
        peripherals.RADIO_CLK,
        peripherals.ADC2,
        wifi_resources,
    );
    let _ = wifi.set_channel(get_test_channel());
    let _ = wifi.write_rx_policy_raw(0, 0);
    loop {
        let received = wifi.receive().await;
        let Ok(generic_frame) = GenericFrame::new(received.mpdu_buffer(), false) else {
            continue;
        };
        if generic_frame.address_1() == OWN_MAC_ADDRESS {
            let _ = wifi
                .transmit(
                    &mut [
                        0xd4,
                        0x00,
                        0x00,
                        0x00,
                        OTHER_MAC_ADDRESS[0],
                        OTHER_MAC_ADDRESS[1],
                        OTHER_MAC_ADDRESS[2],
                        OTHER_MAC_ADDRESS[3],
                        OTHER_MAC_ADDRESS[4],
                        OTHER_MAC_ADDRESS[5],
                    ],
                    &TxParameters::default(),
                    None,
                )
                .await;
            info!("Transmitted software ACK.");
        }
    }
}
