#![no_main]
#![no_std]
use core::marker::PhantomData;

use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use esp_backtrace as _;
use esp_rtos::main;
use esp_wifi_hal::{RxFilterBank, TxParameters, WiFiRate};
use examples::{common_init, embassy_init, setup_filters, wifi_init, AP_ADDRESS};
use ieee80211::{
    common::{CapabilitiesInformation, SequenceControl, TU},
    element_chain,
    elements::VendorSpecificElement,
    mac_parser::BROADCAST,
    mgmt_frame::{body::BeaconBody, BeaconFrame, ManagementFrameHeader},
    scroll::Pwrite,
    ssid,
};

const SSID: &str = "BEACON TSF HIL TEST";

#[main]
async fn main(_spawner: Spawner) {
    let peripherals = common_init();
    embassy_init(peripherals.TIMG0);
    let wifi = wifi_init(peripherals.WIFI, peripherals.ADC2);

    let mut beacon_ticker = Ticker::every(Duration::from_micros(TU.as_micros() as u64 * 100));
    let mut buffer = [0u8; 300];
    unsafe {
        const ADDRESS: *mut u32 = 0x3ff73c20 as _;
        ADDRESS.write_volatile(ADDRESS.read_volatile() | 0xc000_0000);
    }
    loop {
        for interface in 0..4 {
            setup_filters(&wifi, AP_ADDRESS, AP_ADDRESS);

            let vendor_body = [interface as u8];
            let frame = BeaconFrame {
                header: ManagementFrameHeader {
                    receiver_address: BROADCAST,
                    transmitter_address: AP_ADDRESS.into(),
                    bssid: AP_ADDRESS.into(),
                    sequence_control: SequenceControl::new(),
                    ..Default::default()
                },
                body: BeaconBody {
                    beacon_interval: 100,
                    timestamp: 0, // Timestamp is overwritten by hardware.
                    capabilities_info: CapabilitiesInformation::new().with_is_ess(true),
                    elements: element_chain! {
                        ssid!(SSID),
                        VendorSpecificElement::new_prefixed([0x00, 0x80, 0x41].as_slice(), vendor_body.as_slice())
                    },
                    _phantom: PhantomData,
                },
            };
            let written = buffer.pwrite(frame, 0).unwrap();
            wifi.transmit(
                &mut buffer[..written],
                &TxParameters {
                    rate: WiFiRate::PhyRate1ML,
                    override_seq_num: true,
                    ..Default::default()
                },
                None,
            )
            .await
            .unwrap();
            beacon_ticker.next().await;
            let _ = wifi.set_filter_status(RxFilterBank::BSSID, interface, false);
        }
    }
}
