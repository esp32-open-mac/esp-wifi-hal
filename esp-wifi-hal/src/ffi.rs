// These implementations are taken from esp-wifi.

use esp_hal::{ram, rtc_cntl::RtcClock};

#[cfg(osi_funcs_required)]
#[ram]
extern "C" fn empty() {}

#[cfg(osi_funcs_required)]
#[allow(non_upper_case_globals)]
#[no_mangle]
#[ram]
static g_osi_funcs_p: &[extern "C" fn(); 0x6a] = &[empty; 0x6a];

#[ram]
#[no_mangle]
unsafe extern "C" fn esp_dport_access_reg_read(reg: u32) -> u32 {
    (reg as *mut u32).read_volatile()
}

#[cfg(target_arch = "xtensa")]
#[ram]
#[no_mangle]
unsafe extern "C" fn phy_enter_critical() -> u32 {
    core::mem::transmute(critical_section::acquire())
}

/// **************************************************************************
/// Name: phy_exit_critical
///
/// Description:
///   Exit from critical state
///
/// Input Parameters:
///   level - CPU PS value
///
/// Returned Value:
///   None
///
/// *************************************************************************
#[cfg(target_arch = "xtensa")]
#[ram]
#[no_mangle]
unsafe extern "C" fn phy_exit_critical(level: u32) {
    critical_section::release(core::mem::transmute::<u32, critical_section::RestoreState>(
        level,
    ));
}

#[ram]
#[no_mangle]
unsafe extern "C" fn rtc_get_xtal() -> u32 {
    use esp_hal::clock::Clock;

    let xtal = RtcClock::xtal_freq();
    xtal.mhz()
}

extern "C" {
    #[cfg(nomac_channel_set)]
    pub fn chip_v7_set_chan_nomac(channel: u8, bandwidth: u8);
    #[cfg(not(nomac_channel_set))]
    pub fn chip_v7_set_chan(channel: u8, bandwidth: u8);
    pub fn hal_init();
    pub fn tx_pwctrl_background(_: u8, _: u8);
    pub fn enable_wifi_agc();
    pub fn disable_wifi_agc();
}
