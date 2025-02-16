//! This tests, whether the timeout parameter actually configures the ACK timeout, by transmitting
//! an ACK in software well after one SIFS, so the default value would cause an ACK timeout.
//! SPOILER: It does...
#![no_std]
#![no_main]
use core::marker::PhantomData;

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;
use esp_wifi_hal::{DMAResources, RxFilterBank, TxParameters, WiFi};
use ieee80211::{
    common::{DataFrameSubtype, FCFFlags, SequenceControl},
    data_frame::{header::DataFrameHeader, DataFrame},
    mac_parser::MACAddress,
    scroll::Pwrite,
};
use log::info;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}
const OTHER_MAC_ADDRESS: MACAddress = MACAddress::new([0x00, 0x80, 0x41, 0x13, 0x37, 0x41]);
const OWN_MAC_ADDRESS: MACAddress = MACAddress::new([0x00, 0x80, 0x41, 0x13, 0x37, 0x42]);
#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_println::logger::init_logger_from_env();

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let dma_resources = mk_static!(DMAResources<1500, 10>, DMAResources::new());

    let wifi = WiFi::new(
        peripherals.WIFI,
        peripherals.RADIO_CLK,
        peripherals.ADC2,
        dma_resources,
    );
    let _ = wifi.set_filter(
        RxFilterBank::ReceiverAddress,
        0,
        *OWN_MAC_ADDRESS,
        [0xff; 6],
    );
    let _ = wifi.set_filter_status(RxFilterBank::ReceiverAddress, 0, true);
    let _ = wifi.set_filter(RxFilterBank::BSSID, 0, *OWN_MAC_ADDRESS, [0xff; 6]);
    let _ = wifi.set_filter_status(RxFilterBank::BSSID, 0, true);
    let mut buf = [0x00u8; 300];
    let written = buf
        .pwrite(
            DataFrame {
                header: DataFrameHeader {
                    subtype: DataFrameSubtype::Data,
                    fcf_flags: FCFFlags::new().with_to_ds(true),
                    address_1: OWN_MAC_ADDRESS,
                    address_2: OWN_MAC_ADDRESS,
                    address_3: OTHER_MAC_ADDRESS,
                    sequence_control: SequenceControl::new(),
                    ..Default::default()
                },
                payload: Some([0x69u8].as_slice()),
                _phantom: PhantomData,
            },
            0,
        )
        .unwrap();
    loop {
        let res = wifi
            .transmit(
                &mut buf[..written],
                &TxParameters {
                    tx_timeout: 30,
                    ..Default::default()
                },
                Some(0),
            )
            .await;
        if res.is_ok() {
            info!("Ack arrived in time.");
        } else {
            info!("Ack timeout.");
        }
        Timer::after_millis(500).await;
    }
}
