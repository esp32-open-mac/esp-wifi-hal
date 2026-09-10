#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_net::icmp::{
    ChecksumCapabilities, IcmpSocket, Icmpv4Packet, Icmpv4Repr, PacketMetadata,
};
use embassy_net::{Runner as NetRunner, StackResources};
use embassy_time::{Duration, Timer, with_timeout};
use esp_backtrace as _;
use esp_hal::rng::Rng;
use examples::{common_init, embassy_init, mk_static};
use foa::{FoAResources, FoARunner, VirtualInterface};
use foa_sta::{ConnectionConfig, Credentials, StaNetDevice, StaResources, StaRunner};
use log::info;

#[embassy_executor::task]
async fn foa_task(mut runner: FoARunner<'static>) {
    runner.run().await
}

#[embassy_executor::task]
async fn sta_task(mut runner: StaRunner<'static, 'static>) {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: NetRunner<'static, StaNetDevice<'static>>) -> ! {
    runner.run().await
}

#[cfg(all(feature = "esp32s3", feature = "network-trace"))]
fn log_rx_state() {
    let regs = unsafe { esp_wifi_hal::ll::LowLevelDriver::regs() };
    let base = regs.rx_dma_list().rx_descr_base().read().bits();
    info!(
        "stage=rx_state base={:x} next={:x} last={:x} control={:x} interrupt={:x}",
        base,
        regs.rx_dma_list().rx_descr_next().read().bits(),
        regs.rx_dma_list().rx_descr_last().read().bits(),
        regs.rx_ctrl().read().bits(),
        regs.mac_interrupt().wifi_int_status().read().bits()
    );
    // S3 descriptor registers contain a 20-bit DRAM offset, plus status bits.
    let base = (base & 0xfffff) | 0x3fc00000;
    if (0x3fc80000..0x3fd00000).contains(&base) && base & 3 == 0 {
        let descriptor = unsafe { (base as *const esp_hal::dma::DmaDescriptor).read_volatile() };
        info!(
            "stage=rx_head length={} complete={} flags={:x}",
            descriptor.len(),
            descriptor.flags.suc_eof(),
            descriptor.flags.0
        );
    }
}

/// Match the C reference test: twenty 512-byte echo requests to the DHCP gateway.
async fn ping_gateway(stack: embassy_net::Stack<'_>, cycle: u32) -> usize {
    let gateway = stack.config_v4().unwrap().gateway.expect("No DHCP gateway");
    let mut rx_meta = [PacketMetadata::EMPTY; 2];
    let mut tx_meta = [PacketMetadata::EMPTY; 1];
    let mut rx_bytes = [0u8; 1200];
    let mut tx_bytes = [0u8; 600];
    let mut socket = IcmpSocket::new(
        stack,
        &mut rx_meta,
        &mut rx_bytes,
        &mut tx_meta,
        &mut tx_bytes,
    );
    let identifier = 0x5353u16;
    socket
        .bind(embassy_net::icmp::IcmpEndpoint::Ident(identifier))
        .unwrap();
    let payload = [0x5au8; 512];
    let mut request = [0u8; 520];
    let mut response = [0u8; 600];
    let checksum = ChecksumCapabilities::default();
    let mut received = 0;
    for sequence in 1..=20 {
        Icmpv4Repr::EchoRequest {
            ident: identifier,
            seq_no: sequence,
            data: &payload,
        }
        .emit(
            &mut Icmpv4Packet::new_unchecked(&mut request[..]),
            &checksum,
        );
        let reply = with_timeout(Duration::from_secs(1), async {
            socket.send_to(&request, gateway).await.unwrap();
            loop {
                let (length, source) = socket.recv_from(&mut response).await.unwrap();
                let packet = Icmpv4Packet::new_checked(&response[..length]).unwrap();
                if let Ok(Icmpv4Repr::EchoReply {
                    ident,
                    seq_no,
                    data,
                }) = Icmpv4Repr::parse(&packet, &checksum)
                {
                    if source == gateway.into()
                        && ident == identifier
                        && seq_no == sequence
                        && data == payload
                    {
                        break;
                    }
                }
            }
        })
        .await;
        if reply.is_ok() {
            received += 1;
        } else {
            log::warn!(
                "stage=gateway_timeout cycle={} sequence={}",
                cycle,
                sequence
            );
            #[cfg(all(feature = "esp32s3", feature = "network-trace"))]
            log_rx_state();
        }
        Timer::after_millis(100).await;
    }
    info!(
        "stage=gateway_ping cycle={} sent=20 received={}",
        cycle, received
    );
    received
}

/// Exercise WPA2, DHCP and reconnection using FoA and the Rust MAC driver.
/// Credentials are build-time environment variables and are never logged.
#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let peripherals = common_init();
    embassy_init(peripherals.TIMG0, peripherals.SW_INTERRUPT);
    esp_alloc::heap_allocator!(size: 40 * 1024);
    let cycles = option_env!("S3_SMOKE_CYCLES")
        .map(|s| s.parse::<u32>().unwrap())
        .unwrap_or(3);
    assert!((1..=100).contains(&cycles));
    info!("stage=boot test=rust_foa_wpa2 cycles={}", cycles);
    let resources = mk_static!(FoAResources, FoAResources::new());
    let ([vif, ..], runner) = foa::init(resources, peripherals.WIFI);
    spawner.spawn(foa_task(runner).unwrap());
    let (mut control, runner, device) = foa_sta::new_sta_interface(
        mk_static!(VirtualInterface<'static>, vif),
        mk_static!(StaResources<'static>, StaResources::default()),
    );
    spawner.spawn(sta_task(runner).unwrap());
    control.randomize_mac_address().unwrap();
    let (stack, runner) = embassy_net::new(
        device,
        embassy_net::Config::dhcpv4(Default::default()),
        mk_static!(StackResources<3>, StackResources::new()),
        Rng::new().random() as u64,
    );
    spawner.spawn(net_task(runner).unwrap());
    let mut gateway_received = 0;
    for cycle in 1..=cycles {
        with_timeout(
            Duration::from_secs(25),
            control.connect_by_ssid(
                env!("SSID"),
                Some(ConnectionConfig {
                    beacon_timeout: None,
                    ..Default::default()
                }),
                Some(Credentials::Passphrase(env!("PASSWORD"))),
            ),
        )
        .await
        .expect("Connection timed out")
        .expect("Connection failed");
        info!("stage=connected cycle={}", cycle);
        with_timeout(Duration::from_secs(15), stack.wait_config_up())
            .await
            .expect("DHCP timed out");
        info!(
            "stage=dhcp cycle={} address={:?}",
            cycle,
            stack.config_v4().unwrap().address
        );
        // Report immediate configuration separately: the initial test lost the
        // first ping on two reconnects. This window measures settled traffic.
        Timer::after_secs(1).await;
        info!(
            "stage=traffic_ready cycle={} address={:?}",
            cycle,
            stack.config_v4().unwrap().address
        );
        // A host can send 20 pings during each window; auto-ICMP replies exercise
        // encrypted data in both directions without an external service.
        Timer::after_secs(10).await;
        // Run after the host window so gateway ARP cannot prime that test.
        gateway_received += ping_gateway(stack, cycle).await;
        control.disconnect().await.expect("Disconnect failed");
        Timer::after_millis(500).await;
        info!("stage=cycle_complete cycle={}", cycle);
    }
    assert_eq!(gateway_received, cycles as usize * 20, "Gateway echo loss");
    info!(
        "stage=complete test=rust_foa_wpa2 cycles={} gateway_replies={}",
        cycles, gateway_received
    );
    loop {
        Timer::after_secs(1).await;
    }
}
