# esp-wifi-hal
An experimental asynchronous driver for the Wi-Fi peripheral of the ESP32-series chips.
## DISCLAIMER
This is experimental software. USE AT YOUR OWN RISK! We'll not take any liability for damage to the hardware. We do not condone the use of this for malicious purposes.
## Usage
Currently the ESP32 and the ESP32-S2 are supported. You need to indicate, which you're using through the `esp32` and `esp32s2` features, only one of which can be enabled at a time.
## Docs
Since xtensa support isn't mainlined, we can't host the docs on [https://docs.rs/] and right now don't self host them, so you have to generate them with `cargo doc --open --features <YOUR_CHIP> --target xtensa-<YOUR_CHIP>-none-elf`.
## License
This project is licensed under Apache 2.0 or MIT at your option.

