#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, with_timeout};
use esp_backtrace as _;
use esp_wifi_hal::{ll::LowLevelDriver, prelude::*};
use examples::{common_init, embassy_init, wifi_init};
use log::info;

/// Bounded receive/channel/TX test using the Rust MAC driver.
#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let peripherals = common_init();
    info!("stage=boot test=rust_wifi_smoke");
    embassy_init(peripherals.TIMG0, peripherals.SW_INTERRUPT);
    info!("stage=initializing");
    let mut wifi = wifi_init(peripherals.WIFI);
    info!("stage=initialized");
    wifi.set_scanning_mode(0, ScanningMode::BeaconsOnly)
        .unwrap();
    let mut received = 0usize;
    let mut ofdm = 0usize;
    let mut busiest_channel = 1;
    let mut busiest_frames = 0;
    for channel in 1..=11 {
        wifi.set_channel(channel).unwrap();
        let until = Instant::now() + Duration::from_millis(200);
        let mut on_channel = 0usize;
        while Instant::now() < until {
            if let Ok(frame) = with_timeout(Duration::from_millis(50), wifi.receive()).await {
                if frame.mpdu_buffer().len() >= 24 {
                    on_channel += 1;
                    if matches!(frame.phy_rate(), Some(RxPhyRate::Ofdm(_))) {
                        ofdm += 1;
                    }
                }
            }
        }
        received += on_channel;
        if on_channel > busiest_frames {
            busiest_channel = channel;
            busiest_frames = on_channel;
        }
        info!("stage=rx channel={} frames={}", channel, on_channel);
    }
    assert!(received > 0, "No frames received");
    wifi.set_channel(busiest_channel).unwrap();
    // Exhaust the ten descriptors supplied by examples::wifi_init, then return
    // them in reverse order. RX must recover after all buffers were loaned out.
    let mut held: [Option<BorrowedBuffer<'_>>; 10] = core::array::from_fn(|_| None);
    for entry in &mut held {
        *entry = Some(
            with_timeout(Duration::from_secs(2), wifi.receive())
                .await
                .expect("RX drain timed out"),
        );
    }
    info!("stage=rx_exhausted buffers=10");
    for entry in held.iter_mut().rev() {
        drop(entry.take());
    }
    drop(
        with_timeout(Duration::from_secs(2), wifi.receive())
            .await
            .expect("RX did not recover"),
    );
    info!("stage=rx_recovered");
    wifi.set_channel(examples::get_test_channel()).unwrap();
    // Broadcast probe request with wildcard SSID and the four basic DSSS rates.
    let mut probe = [0u8; 32];
    probe[0] = 0x40;
    probe[4..10].fill(0xff);
    probe[10..16].copy_from_slice(&examples::STA_ADDRESS);
    probe[16..22].fill(0xff);
    probe[24..].copy_from_slice(&[0, 0, 1, 4, 0x82, 0x84, 0x8b, 0x96]);
    let result = with_timeout(
        Duration::from_secs(2),
        wifi.transmit_oneshot(
            0,
            &TxPlcpParameters {
                rate: OfdmRate::Mbits6.into(),
                ..Default::default()
            },
            &TxMacParameters {
                override_seq_num: true,
                ..Default::default()
            },
            HardwareTxQueue::Edcaf(EdcaAccessCategory::Voice),
            &mut probe,
        ),
    )
    .await;
    info!("stage=tx result={:?}", result);
    assert!(matches!(result, Ok(Ok(_))), "TX did not complete");
    let before = unsafe { LowLevelDriver::mac_time() };
    embassy_time::Timer::after_millis(100).await;
    let after = unsafe { LowLevelDriver::mac_time() };
    assert!(after > before, "MAC timer stopped");
    info!(
        "stage=complete test=rust_wifi_smoke received={} ofdm={}",
        received, ofdm
    );
}
