/// Indicates, that the chip has the WIFI_PWR interrupt.
const PWR_INTERRUPT_PRESENT: &str = "pwr_interrupt_present";
/// The os adapter is required for `hal_init`. It is fine, if all of the functions are empty
/// though.
const OSI_FUNCS_REQUIRED: &str = "osi_funcs_required";
const NOMAC_CHANNEL_SET: &str = "nomac_channel_set";
/// `g_osi_funcs_p` is a variable in the ROM data area (parts of the Wi-Fi stack live in ROM), so it
/// must be written at runtime instead of being defined by us.
const OSI_FUNCS_IN_ROM: &str = "osi_funcs_in_rom";
/// The TSF counters, TSF timers and TBTT generator are known.
const TSF_TIMER_PRESENT: &str = "tsf_timer_present";

const ESP32_META: &[&str] = &["esp32", NOMAC_CHANNEL_SET];
const ESP32S2_META: &[&str] = &["esp32s2", PWR_INTERRUPT_PRESENT, OSI_FUNCS_REQUIRED];
const ESP32C3_META: &[&str] = &[
    "esp32c3",
    PWR_INTERRUPT_PRESENT,
    OSI_FUNCS_REQUIRED,
    OSI_FUNCS_IN_ROM,
    TSF_TIMER_PRESENT,
];

fn main() {
    let meta = if cfg!(feature = "esp32") {
        ESP32_META
    } else if cfg!(feature = "esp32s2") {
        ESP32S2_META
    } else if cfg!(feature = "esp32c3") {
        ESP32C3_META
    } else {
        panic!("You must select exactly one chip.");
    };
    for item in meta {
        println!("cargo:rustc-cfg={item}");
    }
}
