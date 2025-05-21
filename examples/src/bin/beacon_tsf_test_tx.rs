#![no_main]
#![no_std]
use core::marker::PhantomData;

use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use esp_backtrace as _;
use esp_hal::{efuse::Efuse, timer::timg::TimerGroup};
use esp_hal_embassy::main;
use esp_wifi_hal::{WiFiResources, RxFilterBank, TxParameters, WiFi, WiFiRate};
use ieee80211::{
    common::{CapabilitiesInformation, SequenceControl, TU},
    element_chain,
    elements::VendorSpecificElement,
    mac_parser::{MACAddress, BROADCAST},
    mgmt_frame::{body::BeaconBody, BeaconFrame, ManagementFrameHeader},
    scroll::Pwrite,
    ssid,
};

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

const SSID: &str = "BEACON TSF HIL TEST";

#[main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_println::logger::init_logger_from_env();

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let dma_resources = mk_static!(WiFiResources<10>, WiFiResources::new());

    let wifi = WiFi::new(
        peripherals.WIFI,
        peripherals.RADIO_CLK,
        peripherals.ADC2,
        dma_resources,
    );
    let module_mac_address = MACAddress::new(Efuse::read_base_mac_address());
    let mut beacon_ticker = Ticker::every(Duration::from_micros(TU.as_micros() as u64 * 100));
    let mut buffer = [0u8; 300];
    unsafe {
        const ADDRESS: *mut u32 = 0x3ff73c20 as _;
        ADDRESS.write_volatile(ADDRESS.read_volatile() | 0xc000_0000);
    }
    loop {
        for interface in 0..4 {
            let _ = wifi.set_filter(
                esp_wifi_hal::RxFilterBank::BSSID,
                interface,
                *module_mac_address,
                [0xff; 6],
            );
            let _ = wifi.set_filter_status(RxFilterBank::BSSID, interface, true);
            let vendor_body = [interface as u8];
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
