/// Indicates, that the chip has the WIFI_PWR interrupt.
const PWR_INTERRUPT_PRESENT: &str = "pwr_interrupt_present";
/// The os adapter is required for `hal_init`. It is fine, if all of the functions are empty
/// though.
const OSI_FUNCS_REQUIRED: &str = "osi_funcs_required";
const NOMAC_CHANNEL_SET: &str = "nomac_channel_set";

const ESP32_META: &[&str] = &[NOMAC_CHANNEL_SET];
const ESP32S2_META: &[&str] = &[PWR_INTERRUPT_PRESENT, OSI_FUNCS_REQUIRED];

fn main() {
    let meta = if cfg!(feature = "esp32") {
        ESP32_META
    } else if cfg!(feature = "esp32s2") {
        ESP32S2_META
    } else {
        panic!("You must select exactly one chip.");
    };
    for item in meta {
        println!("cargo:rustc-cfg={item}");
    }
}
