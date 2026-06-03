// These implementations are taken from esp-wifi.

use esp_hal::{clock::xtal_clock, ram};

#[cfg(osi_funcs_required)]
#[allow(non_upper_case_globals)]
#[unsafe(no_mangle)]
#[ram]
static g_osi_funcs_p: &crate::esp_wifi_sys::include::wifi_osi_funcs_t =
    &crate::esp_wifi_sys::include::wifi_osi_funcs_t {
        _version: crate::esp_wifi_sys::include::ESP_WIFI_OS_ADAPTER_VERSION as i32,
        _env_is_chip: None,
        _set_intr: None,
        _clear_intr: None,
        _set_isr: None,
        _ints_on: None,
        _ints_off: None,
        _is_from_isr: None,
        _spin_lock_create: None,
        _spin_lock_delete: None,
        _wifi_int_disable: None,
        _wifi_int_restore: None,
        _task_yield_from_isr: None,
        _semphr_create: None,
        _semphr_delete: None,
        _semphr_take: None,
        _semphr_give: None,
        _wifi_thread_semphr_get: None,
        _mutex_create: None,
        _recursive_mutex_create: None,
        _mutex_delete: None,
        _mutex_lock: None,
        _mutex_unlock: None,
        _queue_create: None,
        _queue_delete: None,
        _queue_send: None,
        _queue_send_from_isr: None,
        _queue_send_to_back: None,
        _queue_send_to_front: None,
        _queue_recv: None,
        _queue_msg_waiting: None,
        _event_group_create: None,
        _event_group_delete: None,
        _event_group_set_bits: None,
        _event_group_clear_bits: None,
        _event_group_wait_bits: None,
        _task_create_pinned_to_core: None,
        _task_create: None,
        _task_delete: None,
        _task_delay: None,
        _task_ms_to_tick: None,
        _task_get_current_task: None,
        _task_get_max_priority: None,
        _malloc: None,
        _free: None,
        _event_post: None,
        _get_free_heap_size: None,
        _rand: None,
        _dport_access_stall_other_cpu_start_wrap: None,
        _dport_access_stall_other_cpu_end_wrap: None,
        _wifi_apb80m_request: None,
        _wifi_apb80m_release: None,
        _phy_disable: None,
        _phy_enable: None,
        _phy_update_country_info: None,
        _read_mac: None,
        _timer_arm: None,
        _timer_disarm: None,
        _timer_done: None,
        _timer_setfn: None,
        _timer_arm_us: None,
        _wifi_reset_mac: None,
        _wifi_clock_enable: None,
        _wifi_clock_disable: None,
        _wifi_rtc_enable_iso: None,
        _wifi_rtc_disable_iso: None,
        _esp_timer_get_time: None,
        _nvs_set_i8: None,
        _nvs_get_i8: None,
        _nvs_set_u8: None,
        _nvs_get_u8: None,
        _nvs_set_u16: None,
        _nvs_get_u16: None,
        _nvs_open: None,
        _nvs_close: None,
        _nvs_commit: None,
        _nvs_set_blob: None,
        _nvs_get_blob: None,
        _nvs_erase_key: None,
        _get_random: None,
        _get_time: None,
        _random: None,
        #[cfg(feature = "print-logs-from-driver")]
        _log_write: None,
        #[cfg(not(feature = "print-logs-from-driver"))]
        _log_write: None,
        #[cfg(feature = "print-logs-from-driver")]
        _log_writev: None,
        #[cfg(not(feature = "print-logs-from-driver"))]
        _log_writev: None,
        _log_timestamp: None,
        _malloc_internal: None,
        _realloc_internal: None,
        _calloc_internal: None,
        _zalloc_internal: None,
        _wifi_malloc: None,
        _wifi_realloc: None,
        _wifi_calloc: None,
        _wifi_zalloc: None,
        _wifi_create_queue: None,
        _wifi_delete_queue: None,
        _coex_init: None,
        _coex_deinit: None,
        _coex_enable: None,
        _coex_disable: None,
        _coex_status_get: None,
        _coex_condition_set: None,
        _coex_wifi_request: None,
        _coex_wifi_release: None,
        _coex_wifi_channel_set: None,
        _coex_event_duration_get: None,
        _coex_pti_get: None,
        _coex_schm_status_bit_clear: None,
        _coex_schm_status_bit_set: None,
        _coex_schm_interval_set: None,
        _coex_schm_interval_get: None,
        _coex_schm_curr_period_get: None,
        _coex_schm_curr_phase_get: None,
        #[cfg(any(
            esp32c3, esp32c2, esp32c5, esp32c6, esp32c61, esp32h2, esp32s3, esp32s2
        ))]
        _slowclk_cal_get: None,
        #[cfg(any(esp32, esp32s2))]
        _phy_common_clock_disable: None,
        #[cfg(any(esp32, esp32s2))]
        _phy_common_clock_enable: None,
        _coex_register_start_cb: None,

        #[cfg(any(esp32c6, esp32c5, esp32c61))]
        _regdma_link_set_write_wait_content: None,
        #[cfg(any(esp32c6, esp32c5, esp32c61))]
        _sleep_retention_find_link_by_id: None,
        _coex_schm_process_restart: None,
        _coex_schm_register_cb: None,

        _coex_schm_flexible_period_set: None,
        _coex_schm_flexible_period_get: None,
        _coex_schm_get_phase_by_idx: None,

        _magic: crate::esp_wifi_sys::include::ESP_WIFI_OS_ADAPTER_MAGIC as i32,
    };

#[ram]
#[unsafe(no_mangle)]
unsafe extern "C" fn esp_dport_access_reg_read(reg: u32) -> u32 {
    unsafe { (reg as *mut u32).read_volatile() }
}

#[cfg(target_arch = "xtensa")]
#[ram]
#[unsafe(no_mangle)]
unsafe extern "C" fn phy_enter_critical() -> u32 {
    unsafe { core::mem::transmute(critical_section::acquire()) }
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
#[unsafe(no_mangle)]
unsafe extern "C" fn phy_exit_critical(level: u32) {
    unsafe {
        critical_section::release(core::mem::transmute::<u32, critical_section::RestoreState>(
            level,
        ))
    };
}

#[ram]
#[unsafe(no_mangle)]
unsafe extern "C" fn rtc_get_xtal() -> u32 {
    xtal_clock().as_mhz()
}

unsafe extern "C" {
    cfg_select! {
        nomac_channel_set => {
            pub fn chip_v7_set_chan_nomac(channel: u8, bandwidth: u8);
        }
        _ => {
            pub fn chip_v7_set_chan(channel: u8, bandwidth: u8);
        }
    }
    pub fn hal_init();
    pub fn tx_pwctrl_background(_: u8, _: u8);
    pub fn enable_wifi_agc();
    pub fn disable_wifi_agc();
}
