#![no_std]
#![no_main]

extern crate alloc;

use core::iter::repeat;

use alloc::{
    boxed::Box,
    collections::btree_set::BTreeSet,
    string::{String, ToString},
};
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Ticker};
use esp_alloc::{self as _, heap_allocator};
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;
use esp_wifi_hal::{DMAResources, ScanningMode, WiFi};
use ieee80211::{match_frames, mgmt_frame::BeaconFrame, GenericFrame};
use log::info;

async fn scan_on_channel(wifi: &mut WiFi<'_>, known_ssids: &mut BTreeSet<String>) {
    loop {
        let received = wifi.receive().await;
        let buffer = received.mpdu_buffer();
        let res = match_frames! {
            buffer,
            beacon_frame = BeaconFrame => {
                let ssid = beacon_frame.ssid().unwrap_or_default();
                if known_ssids.insert(ssid.to_string()) {
                    info!("Found new AP with SSID: {ssid}");
                }
            }
        };
        if res.is_err() {
            let generic_frame = GenericFrame::new(buffer, false).unwrap();
            info!(
                "Got non beacon frame of type: {:?}",
                generic_frame.frame_control_field().frame_type()
            );
        }
    }
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    heap_allocator!(64 * 1024);
    esp_println::logger::init_logger_from_env();

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let mut dma_resources = Box::new(DMAResources::<10>::new());
    let mut wifi = WiFi::new(
        peripherals.WIFI,
        peripherals.RADIO_CLK,
        peripherals.ADC2,
        dma_resources.as_mut(),
    );
    let _ = wifi.set_scanning_mode(0, ScanningMode::BeaconsOnly);
    let mut known_ssids = BTreeSet::new();
    let mut hop_set = repeat([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]).flatten();
    let mut hop_interval = Ticker::every(Duration::from_secs(1));
    loop {
        if let Either::Second(_) = select(
            scan_on_channel(&mut wifi, &mut known_ssids),
            hop_interval.next(),
        )
        .await
        {
            let _ = wifi.set_channel(hop_set.next().unwrap());
        }
    }
}
