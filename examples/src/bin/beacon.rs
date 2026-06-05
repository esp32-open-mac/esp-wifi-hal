#![no_main]
#![no_std]
use core::marker::PhantomData;

use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use esp_backtrace as _;
use esp_hal::efuse::base_mac_address;
use esp_rtos::main;
use esp_wifi_hal::prelude::*;
use examples::{common_init, embassy_init, get_test_channel, wifi_init};
use ieee80211::{
    common::{CapabilitiesInformation, SequenceControl, TU},
    element_chain,
    elements::{
        DSSSParameterSetElement,
        tim::{StaticBitmap, TIMBitmap, TIMElement},
    },
    mac_parser::{BROADCAST, MACAddress},
    mgmt_frame::{BeaconFrame, ManagementFrameHeader, body::BeaconBody},
    scroll::Pwrite,
    ssid, supported_rates,
};

const SSID: &str = "The cake is a lie.";

#[main]
async fn main(_spawner: Spawner) {
    let peripherals = common_init();
    embassy_init(peripherals.TIMG0, peripherals.SW_INTERRUPT);
    let mut wifi = wifi_init(peripherals.WIFI);

    let _ = wifi.set_channel(get_test_channel());
    let module_mac_address: [u8; 6] = base_mac_address().as_bytes().try_into().unwrap();
    let module_mac_address = MACAddress::new(module_mac_address);
    let mut beacon_ticker = Ticker::every(Duration::from_micros(TU.as_micros() as u64 * 100));
    let mut buffer = [0u8; 300];
    loop {
        let frame = BeaconFrame {
            header: ManagementFrameHeader {
                receiver_address: BROADCAST,
                transmitter_address: module_mac_address,
                bssid: module_mac_address,
                sequence_control: SequenceControl::new(),
                ..Default::default()
            },
            body: BeaconBody {
                beacon_interval: 100,
                timestamp: 0, // Timestamp is overwritten by hardware.
                capabilities_info: CapabilitiesInformation::new().with_is_ess(true),
                elements: element_chain! {
                    ssid!(SSID),
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
            .transmit_oneshot(
                0,
                &TxPlcpParameters {
                    rate: OfdmRate::Mbits6.into(),
                    ..Default::default()
                },
                &TxMacParameters {
                    override_seq_num: true,
                    ..Default::default()
                },
                HardwareTxQueue::Beacon,
                &mut buffer[..written],
            )
            .await
            .unwrap();
        beacon_ticker.next().await;
    }
}
