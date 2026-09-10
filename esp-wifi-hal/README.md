# esp-wifi-hal
An experimental asynchronous driver for the Wi-Fi peripheral of the ESP32-series chips.
## DISCLAIMER
This is experimental software. USE AT YOUR OWN RISK! We'll not take any liability for damage to the hardware. We do not condone the use of this for malicious purposes.
## Usage
Select exactly one chip feature: `esp32`, `esp32s2`, or the experimental `esp32s3`.
S3 currently requires the pinned Wi-Fi PAC patch documented in
[the S3 build and validation notes](../docs/esp32s3/RUST.md). Applications using
this crate must apply that patch in their own workspace manifest too.
## Docs
Since xtensa support isn't mainlined, we can't host the docs on [https://docs.rs/] and right now don't self host them, so you have to generate them with `cargo doc --open --features <YOUR_CHIP> --target xtensa-<YOUR_CHIP>-none-elf`.
## License
This project is licensed under Apache 2.0 or MIT at your option.
