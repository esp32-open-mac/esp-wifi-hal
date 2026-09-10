# esp-wifi-hal
This repo contains an experimental port of esp32-open-mac to Rust, with embassy. It is not intended to replace the C version, but to explore, how this can be done in Rust idiomatically. We still rely on the proprietary blobs for initializing the RF frontend and some more minor initialization. 
## DISCLAIMER
This is experimental software. USE AT YOUR OWN RISK! We'll not take any liability for damage to the hardware. We do not condone the use of this for malicious purposes.
## Usage
The actual crate lives in `esp-wifi-hal/` and examples are in `examples/`.
The `critical_section` feature allows using the driver across cores. If it is disabled, no critical sections are used in the driver at all.

For further information see the docs.

ESP32-S3 porting research is in [docs/esp32s3](docs/esp32s3/README.md): a reviewed
MAC reference, register and ABI observations, host regressions, and an ESP-IDF
hardware comparison. This does not yet add S3 support to the Rust crate.
## Building
To set up a development environment follow the guide at https://docs.esp-rs.org/book/installation/index.html. Since this only works on the ESP32 and ESP32-S2 right now, only the Xtensa section is of interest.
To try one of these examples:
1. Clone the repo
2. Connect the ESP32
3. `cd examples`
4. Run `cargo run -r --bin [EXAMPLE_NAME_GOES_HERE]`
## Technical Notes
The ESP32 WiFi peripheral has five TX slots, which we number 0-4. The MMIO addresses, where these are configured are in reverse order. This means, that slot zero starts at the HIGHEST address and slot four at the lowest. This numbering is also suggested by the TX status registers. We could in theory reverse this ordering to ascending addresses, this would however cause headaches with TX slot status handling, so we chose to stick with descending addresses. This is also the way the proprietary stack handles this.

Each TX slot is a hardware transmit queue, four of which directly map to IEEE 802.11 access categories (ACs). The exact mapping is provided in the following table.

Slot | Queue | AC index
-- | -- | --
0 | Beacon | N/A 
1 | Background | 1
2 | Best Effort | 0
3 | Video | 2 
4 | Voice | 3

We have no idea, why they didn't just stick with the AC index order for the slots...
