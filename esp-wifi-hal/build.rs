/// Indicates, that the chip has the WIFI_PWR interrupt.
const PWR_INTERRUPT_PRESENT: &str = "pwr_interrupt_present";
/// The OS adapter is required by MAC initialization and retained binary helpers.
/// Populate callbacks used by each chip even when the rest of the table is unused.
const OSI_FUNCS_REQUIRED: &str = "osi_funcs_required";
const NOMAC_CHANNEL_SET: &str = "nomac_channel_set";

const ESP32S3_META: &[&str] = &[
    "esp32s3",
    PWR_INTERRUPT_PRESENT,
    OSI_FUNCS_REQUIRED,
    "osi_funcs_in_rom",
];

const ESP32_META: &[&str] = &["esp32", NOMAC_CHANNEL_SET];
const ESP32S2_META: &[&str] = &["esp32s2", PWR_INTERRUPT_PRESENT, OSI_FUNCS_REQUIRED];

fn main() {
    let meta = if cfg!(feature = "esp32") {
        ESP32_META
    } else if cfg!(feature = "esp32s2") {
        ESP32S2_META
    } else if cfg!(feature = "esp32s3") {
        ESP32S3_META
    } else {
        panic!("You must select exactly one chip.");
    };
    for item in meta {
        println!("cargo:rustc-cfg={item}");
    }
}
