#![no_std]
#![no_main]
use core::{
    fmt::Write,
    iter::repeat,
    marker::PhantomData,
    str::{self, FromStr},
};

use alloc::{
    collections::btree_set::BTreeSet,
    string::{String, ToString},
};
use embassy_executor::Spawner;
use embassy_futures::{
    select::{select, Either},
    yield_now,
};
use embassy_time::{Duration, Instant, Ticker};
use esp_backtrace as _;
use esp_hal::{
    efuse::Efuse,
    uart::{Config, RxConfig, Uart, UartRx},
    Async,
};
use esp_wifi_hal::{RxFilterBank, ScanningMode, TxParameters, WiFi, WiFiRate, INTERFACE_COUNT};
use examples::{common_init, embassy_init, wifi_init};
use ieee80211::{
    common::{CapabilitiesInformation, FrameType, ManagementFrameSubtype, TU},
    element_chain,
    elements::{
        tim::{StaticBitmap, TIMBitmap, TIMElement},
        DSSSParameterSetElement, SSIDElement,
    },
    mac_parser::{MACAddress, BROADCAST},
    match_frames,
    mgmt_frame::{body::BeaconBody, BeaconFrame, ManagementFrameHeader},
    scroll::Pwrite,
    supported_rates, GenericFrame,
};
extern crate alloc;
use esp_alloc::{self as _, heap_allocator};

const QUIT_SIGNAL: u8 = b'q';

async fn wait_for_quit(uart_rx: &mut UartRx<'_, Async>) {
    loop {
        let mut buf = [0x0u8];
        let _ = uart_rx.read_async(buf.as_mut_slice()).await;
        if buf[0] == QUIT_SIGNAL {
            break;
        }
    }
}
fn channel_cmd<'a>(
    wifi: &WiFi,
    uart0_tx: &mut impl embedded_io::Write,
    mut args: impl Iterator<Item = &'a str> + 'a,
) {
    match args.next() {
        Some("set") => {
            let Some(channel_number) = args.next() else {
                let _ = writeln!(uart0_tx, "Missing argument channel number.");
                return;
            };
            let Ok(channel_number) = channel_number.trim().parse() else {
                let _ = writeln!(uart0_tx, "Argument channel number wasn't a number.");
                return;
            };
            if wifi.set_channel(channel_number).is_ok() {
                let _ = writeln!(
                    uart0_tx,
                    "Successfully switched to channel {channel_number}"
                );
            } else {
                let _ = writeln!(uart0_tx, "Invalid channel {channel_number}.");
            }
        }
        Some("get") => {
            let _ = writeln!(uart0_tx, "Current channel: {}", wifi.get_channel());
        }
        None => {
            let _ = writeln!(uart0_tx, "Missing operand. Valid operands are: set,get");
        }
        Some(operand) => {
            let _ = writeln!(
                uart0_tx,
                "Unknown operand: {operand}, valid operands are: set,get"
            );
        }
    }
}
async fn scan_on_channel(
    wifi: &WiFi<'_>,
    uart0_tx: &mut impl embedded_io::Write,
    known_aps: &mut BTreeSet<String>,
) {
    loop {
        let received = wifi.receive().await;
        let mpdu_buffer = received.mpdu_buffer();
        let _ = match_frames! {
            mpdu_buffer,
            beacon_frame = BeaconFrame => {
                let Some(ssid) = beacon_frame.ssid() else {
                    continue;
                };
                if ssid.trim().is_empty() {
                    continue;
                }
                if !known_aps.contains(ssid) {
                    known_aps.insert(ssid.to_string());
                    let _ = writeln!(uart0_tx, "Found AP with SSID: {ssid} on channel: {}", wifi.get_channel());
                }
            }
        };
    }
}
async fn scan_command<'a>(
    wifi: &WiFi<'_>,
    uart0_tx: &mut impl embedded_io::Write,
    mut args: impl Iterator<Item = &'a str> + 'a,
) {
    let scanning_mode = match args.next() {
        Some("beacons") => ScanningMode::BeaconsOnly,
        Some("all") => ScanningMode::ManagementAndData,
        _ => {
            let _ = writeln!(uart0_tx, "Invalid scan mode, valid modes are beacons, all.");
            return;
        }
    };
    let _ = wifi.set_scanning_mode(0, scanning_mode);
    let mut known_aps = BTreeSet::new();
    let mode = args.next();
    match mode {
        Some("hop") => {
            let mut hop_interval = Ticker::every(Duration::from_millis(200));
            let mut hop_sequence = repeat(1..13).flatten();
            loop {
                if let Either::First(_) = select(
                    hop_interval.next(),
                    scan_on_channel(wifi, uart0_tx, &mut known_aps),
                )
                .await
                {
                    let _ = wifi.set_channel(hop_sequence.next().unwrap());
                }
            }
        }
        None => scan_on_channel(wifi, uart0_tx, &mut known_aps).await,
        Some(operand) => {
            let _ = writeln!(
                uart0_tx,
                "Unknown operand: {operand}, valid operands are: hop"
            );
        }
    }
}
async fn beacon_command<'a>(
    wifi: &WiFi<'_>,
    uart0_tx: &mut impl embedded_io::Write,
    mut args: impl Iterator<Item = &'a str> + 'a,
) {
    let Some(ssid) = args.next() else {
        let _ = writeln!(uart0_tx, "Missing SSID.");
        return;
    };
    let channel_number = if let Some(channel_number) = args.next() {
        let Ok(channel_number) = channel_number.parse() else {
            let _ = writeln!(uart0_tx, "Channel number wasn't a number.");
            return;
        };
        if wifi.set_channel(channel_number).is_err() {
            let _ = writeln!(uart0_tx, "Invalid channel number {channel_number}");
            return;
        }
        channel_number
    } else {
        wifi.get_channel()
    };
    let _ = writeln!(
        uart0_tx,
        "Transmitting beacons with SSID: {ssid} on channel {channel_number}."
    );
    let mac_address = MACAddress::new(Efuse::read_base_mac_address());
    let mut beacon_template = BeaconFrame {
        header: ManagementFrameHeader {
            bssid: mac_address,
            transmitter_address: mac_address,
            receiver_address: BROADCAST,
            ..Default::default()
        },
        body: BeaconBody {
            capabilities_info: CapabilitiesInformation::new().with_is_ess(true),
            timestamp: 0,
            beacon_interval: 100,
            elements: element_chain! {
                SSIDElement::new(ssid).unwrap(),
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
                    current_channel: channel_number,
                },
                TIMElement {
                    dtim_count: 1,
                    dtim_period: 2,
                    bitmap: None::<TIMBitmap<StaticBitmap>>,
                    _phantom: PhantomData
                }
            },
            _phantom: PhantomData,
        },
    };
    let start_timestamp = Instant::now();
    let mut beacon_interval = Ticker::every(Duration::from_micros(100 * TU.as_micros() as u64));
    for i in 0.. {
        let mut buf = [0x00; 200];
        beacon_template.body.timestamp = start_timestamp.elapsed().as_micros();
        beacon_template.elements.next.next.next.inner.dtim_count = i % 2;
        let written = buf.pwrite(beacon_template, 0).unwrap();
        let _ = wifi
            .transmit(
                &mut buf[..written],
                &TxParameters {
                    rate: WiFiRate::PhyRate1ML,
                    override_seq_num: true,
                    ..Default::default()
                },
                None,
            )
            .await
            .unwrap();
        beacon_interval.next().await;
    }
}
fn dump_command(wifi: &WiFi, uart0_tx: &mut impl embedded_io::Write) {
    let _ = writeln!(uart0_tx, "Current channel: {}", wifi.get_channel());
    #[cfg(feature = "esp32")]
{
    extern "C" {
        fn ram_phy_get_noisefloor() -> i32;
    }
    let nf_qdbm = unsafe {
        ram_phy_get_noisefloor()
    };
    let decimal_nf = (nf_qdbm & 0b11) * 25;
    let integer_nf = (nf_qdbm & !0b11) / 4;
    let _ = writeln!(uart0_tx, "Noise floor: {integer_nf}.{decimal_nf} dBm");
    }
    wifi.log_dma_list_stats();
}
fn parse_mac(mac_str: &str) -> Option<MACAddress> {
    let mut mac = [0x00u8; 6];
    let mut octet_iter = mac_str
        .split(':')
        .map(|octet| u8::from_str_radix(octet, 0x10).ok());
    for octet in mac.iter_mut() {
        *octet = octet_iter.next()??;
    }
    Some(MACAddress::new(mac))
}
fn parse_interface(
    interface: Option<&str>,
    uart0_tx: &mut impl embedded_io::Write,
) -> Option<usize> {
    match interface.map(str::parse::<usize>) {
        Some(Ok(interface)) if WiFi::validate_interface(interface).is_ok() => Some(interface),
        _ => {
            let _ = writeln!(
                uart0_tx,
                "Expected argument [interface], valid banks are 0-{}",
                INTERFACE_COUNT
            );
            None
        }
    }
}
fn filter_command<'a>(
    wifi: &WiFi,
    uart0_tx: &mut impl embedded_io::Write,
    mut args: impl Iterator<Item = &'a str> + 'a,
) {
    let bank = match args.next() {
        Some("BSSID") => RxFilterBank::Bssid,
        Some("RA") => RxFilterBank::ReceiverAddress,
        _ => {
            let _ = writeln!(
                uart0_tx,
                "Expected argument [bank], valid banks are BSSID, RA"
            );
            return;
        }
    };
    let Some(interface) = parse_interface(args.next(), uart0_tx) else {
        return;
    };
    match args.next() {
        Some("set") => {
            let Some(mac_address) = args.next() else {
                let _ = writeln!(uart0_tx, "A MAC address is required.");
                return;
            };
            let Some(mac_address) = parse_mac(mac_address) else {
                let _ = writeln!(uart0_tx, "The provided MAC address was invalid.");
                return;
            };
            let _ = wifi.set_filter(bank, interface, *mac_address);
        }
        None => {
            let _ = writeln!(
                uart0_tx,
                "Missing operand, valid operands are: set,enable,disable"
            );
        }
        Some(operand) => {
            let _ = writeln!(
                uart0_tx,
                "Invalid operand {operand}, valid operands are: set,enable,disable"
            );
        }
    }
}
async fn sniff_command(wifi: &WiFi<'_>, uart0_tx: &mut impl embedded_io::Write) {
    wifi.clear_rx_queue();
    loop {
        let received = wifi.receive().await;
        let mut interfaces = [0; 4];
        let if_count = received
            .interface_iterator()
            .enumerate()
            .map(|(i, interface)| interfaces[i] = interface)
            .count();

        let buffer = received.mpdu_buffer();

        let Ok(generic_frame) = GenericFrame::new(buffer, false) else {
            continue;
        };
        if generic_frame.frame_control_field().frame_type()
            == FrameType::Management(ManagementFrameSubtype::Beacon)
        {
            // continue;
        }
        let _ = write!(
            uart0_tx,
            "Type: {:?} RSSI: {}dBm RX state: {:02x} Interfaces: {:?} PHY Rate: {:?} Address 1: {}",
            generic_frame.frame_control_field().frame_type(),
            received.rssi(),
            received.rx_state(),
            &interfaces[..if_count],
            received.phy_rate(),
            generic_frame.address_1()
        );
        if let Some(address_2) = generic_frame.address_2() {
            let _ = write!(uart0_tx, " Address 2: {address_2}");
        } else {
            let _ = writeln!(uart0_tx,);
        }
        if let Some(address_3) = generic_frame.address_3() {
            let _ = writeln!(uart0_tx, " Address 3: {address_3}");
        } else {
            let _ = writeln!(uart0_tx,);
        }
        yield_now().await;
    }
}
fn scanning_mode_command<'a>(
    wifi: &WiFi,
    uart0_tx: &mut impl embedded_io::Write,
    mut args: impl Iterator<Item = &'a str> + 'a,
) {
    let Some(interface) = parse_interface(args.next(), uart0_tx) else {
        let _ = writeln!(uart0_tx, "Expected interface 0-3");
        return;
    };
    let scanning_mode = match args.next() {
        Some("management_and_data") => ScanningMode::ManagementAndData,
        Some("beacons_only") => ScanningMode::BeaconsOnly,
        Some("disabled") => ScanningMode::Disabled,
        _ => {
            let _ = writeln!(
                uart0_tx,
                "Expected argument [management_and_data|beacons_only|disabled]."
            );
            return;
        }
    };
    let _ = wifi.set_scanning_mode(interface, scanning_mode);
}
async fn s_pol_command<'a>(
    wifi: &WiFi<'_>,
    uart0_tx: &mut impl embedded_io::Write,
    mut args: impl Iterator<Item = &'a str> + 'a,
) {
    let Some(interface) = args.next() else {
        let _ = writeln!(uart0_tx, "Missing interface parameter.");
        return;
    };
    let Ok(interface) = interface.parse::<usize>() else {
        let _ = writeln!(uart0_tx, "Couldn't parse interface.");
        return;
    };
    if WiFi::validate_interface(interface).is_err() {
        let _ = writeln!(uart0_tx, "Invalid interface.");
        return;
    }
    let Some(rx_policy) = args.next() else {
        let _ = writeln!(uart0_tx, "Missing RX policy parameter.");
        return;
    };
    let Ok(rx_policy) = u32::from_str_radix(rx_policy, 16) else {
        let _ = writeln!(uart0_tx, "Couldn't parse RX policy.");
        return;
    };
    let _ = writeln!(
        uart0_tx,
        "Previous RX policy: {:#08x}",
        wifi.read_rx_policy_raw(interface).unwrap()
    );
    let _ = wifi.write_rx_policy_raw(interface, rx_policy);
    let _ = writeln!(uart0_tx, "New RX policy: {rx_policy:#08x}");
}
#[derive(PartialEq, Eq)]
enum MemoryOperation {
    Read,
    Write,
    And,
    Or,
    Xor,
}
impl FromStr for MemoryOperation {
    type Err = ();
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Ok(match s {
            "r" => Self::Read,
            "w" => Self::Write,
            "&" => Self::And,
            "|" => Self::Or,
            // I hate typing ^
            "x" => Self::Xor,
            _ => return Err(()),
        })
    }
}
fn m_mod<'a>(uart0_tx: &mut impl embedded_io::Write, mut args: impl Iterator<Item = &'a str> + 'a) {
    let Some(operation) = args.next() else {
        let _ = writeln!(uart0_tx, "Missing operation.");
        return;
    };
    let Ok(operation) = MemoryOperation::from_str(operation) else {
        let _ = writeln!(uart0_tx, "Invalid operation.");
        return;
    };
    let Some(offset_or_address) = args.next() else {
        let _ = writeln!(uart0_tx, "Missing offset from peripheral.");
        return;
    };
    let address = if let Some(address) = offset_or_address.strip_prefix('r') {
        let Ok(offset) = usize::from_str_radix(address, 16) else {
            let _ = writeln!(uart0_tx, "Invalid offset.");
            return;
        };
        if offset & 0b11 != 0 {
            let _ = writeln!(uart0_tx, "Misaligned offset.");
            return;
        }
        unsafe { esp_hal::peripherals::WIFI::PTR.byte_add(offset) as *mut u32 }
    } else {
        let Ok(address) = usize::from_str_radix(offset_or_address, 16) else {
            let _ = writeln!(uart0_tx, "Invalid address.");
            return;
        };
        address as *mut u32
    };
    let previous = unsafe { address.read_volatile() };
    if operation == MemoryOperation::Read {
        let _ = writeln!(uart0_tx, "Read {address:08x?}:{previous:08x}");
        return;
    }

    let Some(value) = args.next() else {
        let _ = writeln!(uart0_tx, "Missing value.");
        return;
    };
    let maybe_stripped_value = value.strip_prefix("!");
    let invert_value = maybe_stripped_value.is_some();
    let Ok(mut value) = u32::from_str_radix(maybe_stripped_value.unwrap_or(value), 16) else {
        let _ = writeln!(uart0_tx, "Invalid value.");
        return;
    };
    if invert_value {
        value = !value;
    }
    let new_value = match operation {
        MemoryOperation::Read => unreachable!(),
        MemoryOperation::Write => value,
        MemoryOperation::And => previous & value,
        MemoryOperation::Or => previous | value,
        MemoryOperation::Xor => previous ^ value,
    };
    unsafe {
        address.write_volatile(new_value);
    }
    let _ = writeln!(uart0_tx, "{address:08x?}:{new_value:08x}");
}
async fn run_command<'a>(
    wifi: &WiFi<'_>,
    uart0_tx: &mut impl embedded_io::Write,
    command: &str,
    args: impl Iterator<Item = &'a str> + 'a,
) {
    match command.trim() {
        "help" => {
            let _ = writeln!(
                uart0_tx,
                "\
                ESP32-OPEN-MAC TEST CLI\n\n\
                Enter the command you wish to execute and hit enter.\n\
                If you want to terminate the currently executing command type 'q'.\n\n\
                Available commands:\n\n\
                channel [get|set] <CHANNEL_NUMBER> - Get or set the current channel.\n\
                scan [all|beacons] [hop] - Look for available access points.\n\
                beacon [SSID] <CHANNEL_NUMBER> - Transmit a beacon every 100 TUs.\n\
                dump - Dump status information.\n\
                filter [BSSID|RA] [0|1] [set|enable|disable] <FILTER_ADDRESS> - Set filter status.\n\
                sniff - Logs frames with rough info.\n\
                scanning_mode INTERFACE [management_and_data|beacons_only|disabled] - Set the scanning mode.\n\
                s_pol INTERFACE REGISTER_VALUE - Set the RX policy register for the given interface. \n\
                m_mod [r|w|&|||x] ADDRESS_OR_OFFSET <VALUE> - Modify raw memory. If you prefix the address with an `r`, it will be treated as an offset relative to the Wi-Fi peripheral base.
            "
            );
        }
        "channel" => {
            channel_cmd(wifi, uart0_tx, args);
        }
        "scan" => {
            scan_command(wifi, uart0_tx, args).await;
        }
        "beacon" => {
            beacon_command(wifi, uart0_tx, args).await;
        }
        "dump" => {
            dump_command(wifi, uart0_tx);
        }
        "filter" => {
            filter_command(wifi, uart0_tx, args);
        }
        "sniff" => sniff_command(wifi, uart0_tx).await,
        "scanning_mode" => scanning_mode_command(wifi, uart0_tx, args),
        "s_pol" => s_pol_command(wifi, uart0_tx, args).await,
        "m_mod" => m_mod(uart0_tx, args),
        _ => {
            let _ = writeln!(uart0_tx, "Unknown command {command}");
        }
    }
}
/*
fn find_last_codepoint(bytes: &[u8]) -> usize {
    for (i, byte) in bytes.iter().enumerate().rev() {
        if !check_bit!(*byte, bit!(7)) {
            return i;
        }
    }
    0
}
*/
#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    heap_allocator!(size: 32 * 1024);

    let peripherals = common_init();
    embassy_init(peripherals.TIMG0);
    let wifi = wifi_init(peripherals.WIFI, peripherals.ADC2);

    #[cfg(feature = "esp32")]
    let (rx_pin, tx_pin) = (peripherals.GPIO3, peripherals.GPIO1);
    #[cfg(feature = "esp32s2")]
    let (rx_pin, tx_pin) = (peripherals.GPIO44, peripherals.GPIO43);

    let (mut uart0_rx, mut uart0_tx) = Uart::new(
        peripherals.UART0,
        Config::default().with_rx(RxConfig::default().with_fifo_full_threshold(64)),
    )
    .unwrap()
    .with_rx(rx_pin)
    .with_tx(tx_pin)
    .into_async()
    .split();

    let _ = uart0_tx.flush_async().await;
    let _ = writeln!(&mut uart0_tx, "READY");
    loop {
        let _ = write!(&mut uart0_tx, "> ");
        let mut buf = [0x00u8; 0xff];
        let mut current_offset = 0;
        loop {
            let Ok(received) = uart0_rx.read_async(&mut buf[current_offset..]).await else {
                break;
            };
            match &buf[current_offset..][..received] {
                // Prevent the user from going up or down.
                b"\x1b[A" | b"\x1b[B" => continue,
                // Handle the user hitting enter.
                b"\x0d" => {
                    let _ = writeln!(uart0_tx,);
                    break;
                }
                _ => {}
            }
            // Increase the offset.
            current_offset += received;
            if current_offset > buf.len() {
                let _ = writeln!(&mut uart0_tx, "Command length exceed buffer limit.");
                break;
            }
            // Handle backspaces.
            if buf[current_offset - 1] == 0x08 {
                if current_offset == 1 {
                    current_offset = 0;
                } else {
                    current_offset -= received + 1;
                    let _ = uart0_tx.write_async(b"\x08\x1bd \x08").await;
                }
                continue;
            }
            // Return the new data to the console.
            let _ = uart0_tx
                .write_async(&buf[(current_offset - received)..current_offset])
                .await;
        }
        let mut cmd_with_args = str::from_utf8(&buf[..current_offset])
            .unwrap()
            .split_whitespace();
        let Some(cmd) = cmd_with_args.next() else {
            continue;
        };
        select(
            wait_for_quit(&mut uart0_rx),
            run_command(&wifi, &mut uart0_tx, cmd, cmd_with_args),
        )
        .await;
    }
}
