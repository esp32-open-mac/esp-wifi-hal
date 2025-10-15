//! This example demonstrates, that concurrent tranmissions are possible.

#![no_main]
#![no_std]
use core::marker::PhantomData;

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_rtos::main;
use esp_wifi_hal::{TxParameters, WiFiRate};
use examples::{common_init, embassy_init, wifi_init};
use ieee80211::{
    common::{CapabilitiesInformation, SequenceControl},
    element_chain,
    elements::{
        tim::{StaticBitmap, TIMBitmap, TIMElement},
        DSSSParameterSetElement, SSIDElement,
    },
    mac_parser::{MACAddress, BROADCAST},
    mgmt_frame::{body::BeaconBody, BeaconFrame, ManagementFrameHeader},
    scroll::Pwrite,
    supported_rates,
};

static SSIDS: [&str; 6] = [
    "Never gonna give you up",
    "Never gonna let you down",
    "Never gonna run around",
    "Never gonna make you cry",
    "Never gonna say goodbye",
    "Never gonna tell a lie",
];
#[main]
async fn main(_spawner: Spawner) {
    let peripherals = common_init();
    embassy_init(peripherals.TIMG0);
    let wifi = wifi_init(peripherals.WIFI, peripherals.ADC2);

    loop {
        for (id, ssid) in SSIDS.iter().enumerate() {
            let mac_address = MACAddress::new([0x00, 0x80, 0x41, 0x13, 0x37, id as u8]);
            let mut buffer = [0u8; 300];
            let frame = BeaconFrame {
                header: ManagementFrameHeader {
                    receiver_address: BROADCAST,
                    transmitter_address: mac_address,
                    bssid: mac_address,
                    sequence_control: SequenceControl::new(),
                    ..Default::default()
                },
                body: BeaconBody {
                    beacon_interval: 100,
                    timestamp: 0,
                    capabilities_info: CapabilitiesInformation::new().with_is_ess(true),
                    elements: element_chain! {
                        SSIDElement::new(*ssid).unwrap(),
                        supported_rates![
                                1 B,
                                2 B,
                                5.5 B,
                                11 B,
                                6,
                                9,
                                12,
                                18
                            ],
                        DSSSParameterSetElement {
                            current_channel: 1
                        },
                        TIMElement {
                            dtim_count: 1,
                            dtim_period: 2,
                            bitmap: None::<TIMBitmap<StaticBitmap>>,
                            _phantom: PhantomData
                        }
                    },
                    ..Default::default()
                },
            };
            let written = buffer.pwrite(frame, 0).unwrap();
            let _ = wifi
                .transmit(
                    &mut buffer[..written],
                    &TxParameters {
                        rate: WiFiRate::PhyRate6M,
                        ..Default::default()
                    },
                    None,
                )
                .await;
        }
        Timer::after_millis(100).await;
    }
}
