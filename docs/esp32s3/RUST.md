# Experimental ESP32-S3 Rust port

The `esp32s3` feature runs the existing Rust driver on S3. MAC initialization is
implemented in `esp-wifi-hal/src/s3_mac.rs`, translated from the separately
[reviewed C reference](REVIEW.md). It initializes the TX/RX block, filter tables,
interrupt defaults and low-rate tables using ordered 32-bit register accesses.
The Rust driver owns the DMA list and installs it after initialization.

This is a partial Wi-Fi deblob. PHY calibration/channel control, auto-ACK rates,
crypto initialization, antenna, timer and coexistence-priority helpers still
come from `esp-phy`, `esp-wifi-sys-esp32s3` or ROM. The S3 ROM OS-adapter pointer
is installed before those helpers run. Bluetooth coexistence is not implemented;
the priority callbacks leave their initialized zero values unchanged.

The final station ELF has no `hal_init`, `mac_txrx_init`, `mac_rxbuf_init` or
`mac_last_rxbuf_init` symbol. Its map discards the blob MAC initialization code.
WPA2 authentication and network traffic in this example use the published FoA
and embassy-net Rust crates. This result is distinct from the earlier ESP-IDF
C reference test, which retained IDF's station stack.

## S3 data-path differences

- RX has a 48-byte control header with SIG_LEN in its final word. The S2 length
  fields at byte 24 cannot validate S3 OFDM frames. Descriptor bounds are checked
  before constructing slices, and SIG_LEN must cover the DMA payload. Header-only descriptors are rejected
  before trailer/padding arithmetic.
- S3 RX descriptor registers return 20-bit DRAM offsets with upper status
  bits. The driver reconstructs addresses under `0x3fc00000`, handles the empty
  queue indication, and recycles completed short descriptors rather than leaving
  them at the queue head. Invalid frames are drained before waiting for another
  interrupt. Borrowed/recycled descriptors are detached from their old successor
  before rejoining the queue, keeping a null tail and avoiding stale links into
  borrowed buffers. Host regressions exercise the production queue code with MMIO stubs.
- TX slot registers have a 0x4c stride. Expected response rate and interface ID
  are in PLCP2 at bits 6..13 and 28..29; writes preserve unrelated slot state.
- HT20 rate codes are 16..23 for long GI and 24..31 for short GI. S3's short-GI
  HT-SIG bit is 31, not 7. The S3 packing code avoids the overlapping short-GI
  rate codes produced by `esp-wifi-rates` 0.1.0.
- S3 needs its own modem-clock enable mask and ROM AGC symbol names.

The ROM adapter and TX port structure build on okhsunrog's
[C3 driver draft](https://github.com/esp32-open-mac/esp-wifi-hal/pull/22).
The PAC mapping likewise credits the C3 work. S3-specific offsets and HT fields
were checked against the S3 blob; C3 TSF latch definitions were not copied.

## Build

Install the esp Xtensa Rust toolchain with espup and source its export script.
The tested toolchain is esp Rust 1.97.0.0 with Xtensa GCC 15.2.0_20250920.
The example lockfile selects esp-hal 1.1.2 and FoA/foa_sta 0.2.0.

The S3 PAC mapping is pending in [esp-pacs #511](https://github.com/esp-rs/esp-pacs/pull/511).
The manifests pin the same Wi-Fi mapping on a PAC 0.35.2 compatibility branch,
because the current driver/PHY dependencies use that release. The PAC PR itself
targets upstream main (0.36.0). An external application's root manifest needs:

```toml
[patch.crates-io]
esp32s3 = { git = "https://github.com/matteius/esp-pacs", rev = "37b54bd9ad62de17b17a8d7984a2ace5e73d63dd" }
# When testing published FoA against this checkout:
esp-wifi-hal = { path = "/path/to/esp-wifi-hal/esp-wifi-hal" }
```

From this repository:

```sh
sh docs/esp32s3/tests/run-rust-tests.sh
cd examples
ESP_LOG=info cargo +esp build --locked --release \
  --target xtensa-esp32s3-none-elf --features esp32s3 --bin wifi_smoke

# Set SSID and PASSWORD in the build environment for your own test network.
ESP_LOG=info cargo +esp build --locked --release \
  --target xtensa-esp32s3-none-elf --features esp32s3,foa-smoke --bin sta_smoke
```

The station example embeds those credentials in its binary. Keep build artifacts
private and use info logging; FoA debug logs can include security material.

## Hardware tests

`wifi_smoke` scans channels 1..11, requires a received frame, submits an OFDM
6 Mbps broadcast probe, checks TX completion, and checks that the MAC timer
advances. It also loans out all ten RX buffers, returns them in reverse order,
and requires reception to resume. TX completion does not independently establish over-the-air FCS or
interoperability at every rate.

`sta_smoke` starts FoA and embassy-net once, then performs three WPA2 connection,
IPv4 configuration and disconnect cycles. Each cycle logs `stage=dhcp`, waits
one second, then logs `stage=traffic_ready` and leaves a ten-second traffic
window. Set the build environment variable `S3_SMOKE_CYCLES=10` for ten cycles.
Send 20 host pings in each window to the reported address:

```sh
ping -c 20 -i .2 -s 512 -W 1 DEVICE_IP
```

The original immediate and one-second-delay tests both received 58/60 host
ping replies: 20/20 in cycle 1, then 19/20 in cycles 2 and 3. The missing reply
was the first ping after each reconnect. The cause is smoltcp's automatic ICMP
responder, described below. IPv4 configuration can reuse the DHCP lease.
Three completed cycles do not test full PHY/MAC teardown:
the final application remains initialized with the station disconnected.

The tested S3 rev 0.1 has secure boot and Secure Download Mode enabled, with
flash encryption disabled. Images were signed with its existing accepted key
and written only into its factory app slot at `0x20000` (1 MiB). The existing
bootloader and partition table were preserved. For this specific layout, the
application-only image flow was:

```sh
espflash save-image --chip esp32s3 --flash-size 2mb --flash-mode dio \
  --flash-freq 40mhz --partition-table "$S3_PROJECT/partitions.csv" \
  --partition-table-offset 0x10000 --target-app-partition factory \
  --bootloader "$S3_PROJECT/build/bootloader/bootloader.bin" \
  target/xtensa-esp32s3-none-elf/release/sta_smoke sta-smoke.bin
python -m espsecure sign_data --version 2 --keyfile "$S3_SIGNING_KEY" \
  --output sta-smoke.signed.bin sta-smoke.bin
python -m espsecure verify_signature --version 2 --keyfile "$S3_SIGNING_KEY" \
  sta-smoke.signed.bin
python -m esptool --chip esp32s3 --port "$S3_PORT" --no-stub \
  --baud 460800 --after hard_reset write_flash 0x20000 sta-smoke.signed.bin
```

This uses espflash 4.6.0 for image generation and esptool/espsecure 4.11.0 for
signing and app-only deployment. Match the app slot, size, signing key and flash
encryption state to the target before using these layout-specific commands.
The generic `cargo run` flash runner is not the secured-board deployment path.

## Ping loss and RX queue diagnosis

The board trace for reconnects 2 and 3 shows:

```text
stage=traffic_ready cycle=2
address HOST_IP not in neighbor cache, sending ARP request
Failed to send response: NeighborPending
filled HOST_IP => HOST_MAC (was empty)
```

The ping reached the IP stack through the Rust MAC and decrypted data path.
smoltcp 0.13.1 generated an automatic echo reply, but its neighbor cache had
been cleared during reconfiguration. It sent ARP and returned `NeighborPending`.
That immediate response is not queued for retry when ARP completes, so the first
reply is discarded. A delay alone does not populate the neighbor cache.

The exact behavior is reproduced without ESP code, hardware or a real network:

```sh
# Any host Rust toolchain >=1.91; +esp was used here for its host compiler.
cargo +esp test --locked --target x86_64-unknown-linux-gnu \
  --manifest-path docs/esp32s3/tests/neighbor-repro/Cargo.toml
```

The reproducer injects echo 1 into a virtual Ethernet device with an empty
neighbor cache, observes ARP, supplies the ARP reply, confirms echo 1 was not
retried, then verifies echo 2 succeeds. This passing reproduction test documents
an existing stack limitation; it does not fix smoltcp.

The relevant source is
[`socket_ingress` and `lookup_hardware_addr`](https://github.com/smoltcp-rs/smoltcp/blob/e347a1e2d3ac33c5ce2c0c114e24b85ae23c4897/src/iface/interface/mod.rs).
The C test used a different path: the application sent pings **to the gateway**.
`sta_smoke` now also sends twenty 512-byte gateway requests per cycle using a
queued ICMP socket, verifies source/identifier/sequence/payload/checksum, and
requires 60 replies across all three cycles. This runs **after** each host-ping
window so gateway traffic cannot prime that window's ARP state.

To reproduce the board-side diagnosis, enable `network-trace` instead of
`foa-smoke` and build with `ESP_LOG=info,smoltcp=trace,embassy_net=debug`.
This leaves FoA security debug logging disabled. The port does not change
smoltcp's automatic responder or disguise missing replies with warm-up pings.

An extended test also exposed intermittent loss of all reception. Two exploratory
builds stopped receiving after initial successful traffic; an attempted ten-cycle
run then failed during cycle 5. Register snapshots showed hardware visiting a
subset of descriptors while the software head stayed incomplete. Review found
that recycled descriptors retained their old successor, allowing stale links
and hardware loops when buffers were borrowed and returned. The production-queue
regression reproduced that invariant failure before the fix. Descriptors now
detach on removal and have a null successor before being appended. The report
keeps those failed runs alongside the subsequent validation; the ARP responder
limitation and this queue defect have separate reproductions.

## Results and remaining work

[rust-validation-summary.json](rust-validation-summary.json) records the exact
source, image and dependency hashes, host checks and per-cycle hardware results.
The host tests include ten C regression groups, a differential comparison of
the Rust/C ordered initialization trace across four starting register patterns,
three S3 HT/PLCP encoding tests, four RX length/address tests, and four tests
of the production DMA queue with host descriptor/register stubs. The separate
neighbor-cache reproducer isolates the automatic ICMP responder limitation.

The final default station run completed all three reconnect cycles with **60/60
device-to-gateway replies**, matching the C test's direction. Host-to-device
automatic replies remained 58/60, with the first reply lost after reconnects 2
and 3 as explained above. The final `wifi_smoke` run also passed scanning,
OFDM TX completion, MAC timer advancement and exhaustion/recovery of all ten
RX buffers.

The extended run after descriptor detachment completed seven cycles with
140/140 gateway replies and 134/140 host replies (the six first-ping ARP losses),
then its eighth connection attempt exceeded the 25-second timeout. That remains
an unresolved limitation; the ten-cycle run is recorded as incomplete, not a
pass. The three-cycle default and explicit queue exhaustion test are reported
separately.

Hardware coverage is one board and one WPA2 access point. AP mode, multiple VIFs,
HT rate/short-GI sweeps, power saving, Bluetooth coexistence, CSI, FTM exchanges
and long-duration reliability remain unvalidated. The port is ready for review
as experimental support; the PAC/API shape and these limits remain review items.
