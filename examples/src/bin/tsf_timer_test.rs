//! TSF timer and TBTT test.
//!
//! Enables the TSF counter of interface 0, arms TSF timer 0 one second ahead a few times and
//! measures how long the PWR interrupt takes to arrive. Then configures the TBTT generator of
//! interface 0 with a 100 TU beacon interval and prints the spacing and phase of the TBTT events.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_println::println;
use esp_wifi_hal::prelude::*;
use examples::{common_init, embassy_init, get_test_channel, wifi_init};

const TU: u32 = 1024;

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let peripherals = common_init();
    embassy_init(peripherals.TIMG0, peripherals.SW_INTERRUPT);
    let mut wifi = wifi_init(peripherals.WIFI);
    let _ = wifi.set_channel(get_test_channel());

    println!(
        "tsf_timer_test: tsf0 enabled={} tsf0={} mac_time={}",
        unsafe { wifi.ll_driver_ref() }.tsf_enabled(0),
        wifi.tsf_time(0).unwrap(),
        wifi.mac_time().duration_since_epoch().as_micros()
    );
    wifi.set_tsf_enabled(0, true).unwrap();
    wifi.set_tsf_time(0, 0).unwrap();
    Timer::after_millis(10).await;
    println!(
        "after enable: tsf0={} mac_time={}",
        wifi.tsf_time(0).unwrap(),
        wifi.mac_time().duration_since_epoch().as_micros()
    );

    // TSF timer 0: fire one second after now, five times.
    for round in 0..5 {
        let now = wifi.tsf_time(0).unwrap();
        let target = (now as u32).wrapping_add(1_000_000);
        let mac_before = wifi.mac_time();
        wifi.set_tsf_timer(0, target).unwrap();
        let waiter = wifi.wait_for_tsf_timer(0).unwrap();
        match select(waiter, Timer::after(Duration::from_secs(3))).await {
            Either::First(()) => {
                let fired_at = wifi.tsf_time(0).unwrap();
                println!(
                    "timer #{round}: target={target} fired at tsf={fired_at} (+{} us tsf, +{} us mac)",
                    fired_at.wrapping_sub(now),
                    (wifi.mac_time() - mac_before).as_micros()
                );
            }
            Either::Second(()) => {
                println!(
                    "timer #{round}: TIMEOUT, tsf now={} target={target} cfg_target={}",
                    wifi.tsf_time(0).unwrap(),
                    unsafe { wifi.ll_driver_ref() }.tsf_timer_target(0)
                );
            }
        }
    }
    wifi.cancel_tsf_timer(0).unwrap();

    // TBTT of interface 0 with a 100 TU beacon interval. TBTTs happen when the TSF is a multiple
    // of the interval, so load the TSF to a known phase first.
    let interval: u16 = 100;
    let interval_us = interval as u64 * TU as u64;
    wifi.set_tsf_time(0, 20 * interval_us + 30_000).unwrap();
    wifi.set_tbtt(0, interval, 0).unwrap();
    wifi.set_tbtt_enabled(0, true).unwrap();
    let mut last: Option<u64> = None;
    for round in 0..6 {
        let waiter = wifi.wait_for_tbtt(0).unwrap();
        match select(waiter, Timer::after(Duration::from_secs(2))).await {
            Either::First(()) => {
                let t = wifi.tsf_time(0).unwrap();
                let spacing = last.map(|l| t.wrapping_sub(l)).unwrap_or(0);
                println!(
                    "tbtt #{round}: tsf={t} spacing={spacing} us phase={} us",
                    t % interval_us
                );
                last = Some(t);
            }
            Either::Second(()) => {
                println!(
                    "tbtt #{round}: TIMEOUT, tsf now={}",
                    wifi.tsf_time(0).unwrap()
                );
                break;
            }
        }
    }
    wifi.set_tbtt_enabled(0, false).unwrap();
    println!("done");
    loop {
        Timer::after_secs(10).await;
    }
}
