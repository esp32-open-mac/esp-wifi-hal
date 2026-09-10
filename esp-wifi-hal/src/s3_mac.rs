//! S3 MAC initialization translated from the reviewed `hal_mac.o` reference.
//!
//! Register transactions keep the order and 32-bit widths of the original.
//! PHY, auto-ACK rate, antenna, timer and coexistence helpers remain external.
//! See `docs/esp32s3/REVIEW.md` for the original input and instruction evidence.

#[inline(always)]
fn read(address: usize) -> u32 {
    #[cfg(test)]
    unsafe {
        return s3_test_read(address);
    }
    #[cfg(not(test))]
    unsafe {
        (address as *const u32).read_volatile()
    }
}

#[inline(always)]
fn write(address: usize, value: u32) {
    #[cfg(test)]
    unsafe {
        s3_test_write(address, value);
    }
    #[cfg(not(test))]
    unsafe {
        (address as *mut u32).write_volatile(value)
    }
}

#[cfg(test)]
unsafe extern "C" {
    fn s3_test_read(address: usize) -> u32;
    fn s3_test_write(address: usize, value: u32);
}

#[inline(always)]
fn update(address: usize, keep: u32, set: u32) {
    write(address, (read(address) & keep) | set);
}

unsafe extern "C" {
    fn hal_mac_rate_autoack_init();
    fn hal_crypto_init();
    fn hal_attenna_init();
    fn phy_disable_low_rate();
    fn hal_timer_update_by_rtc(enable: u32, calibration: u32);
    fn hal_coex_pti_init();
    fn hal_set_rx_active_pti(priority: u32);
    fn hal_set_rx_ack_pti(priority: u32);
    fn hal_set_wifi_default_pti(priority: u32);
}

fn init_tx_rx() {
    update(0x60033c6c, u32::MAX, 0x8080a000);
    update(0x60033c6c, u32::MAX, 0x100);
    for interface in 0..4 {
        update(0x600330d8 + 4 * interface, u32::MAX, 0x40);
        update(0x600330d8 + 4 * interface, 0xffffffdf, 0);
    }
    update(0x60033c74, u32::MAX, 8);
    for interface in 0..4 {
        update(0x60033100 + 4 * interface, 0xffff, 0);
    }
    update(0x60033100, u32::MAX, 0x1000000);
    update(0x60033104, u32::MAX, 0x1000000);
    update(0x60033100, u32::MAX, 0x4000000);
    update(0x60033104, u32::MAX, 0x4000000);
    update(0x60033c6c, u32::MAX, 0x200);
    update(0x60033114, 0xffffff0f, 0);
    update(0x60033118, u32::MAX, 0x80000000);
    update(0x60033118, 0xf00fffff, 0x1b00000);
    update(0x60033c78, u32::MAX, 3);
    update(0x60033c10, 0xfffff000, 0xf0);
    update(0x60033c10, u32::MAX, 0x80000000);
    update(0x60033c10, u32::MAX, 0x40000000);
    update(0x60033c14, 0xfffff000, 0xf0);
    update(0x60033c18, 0xfffff000, 0xf0);
    update(0x60033c94, 0xffffff0f, 0x40);
    update(0x60033c54, u32::MAX, 0x7fff0000);
    update(0x60033c54, u32::MAX, 0x80000000);
    update(0x60033c88, 0xf0ffffff, 0);
    update(0x600332b8, u32::MAX, 2);
    update(0x60033084, 0x7fffffff, 0);
}

fn init_rx_buffers() {
    update(0x60033c5c, 0xfff00000, 0xf8000);
    update(0x60033c60, 0xfff00000, 0x84000);
    update(0x60033c64, 0x000fffff, 0x3fc00000);
    update(0x60033080, 0xffffff00, 0);
    // The Rust driver owns the DMA list and installs it after MAC initialization.
    write(0x60033088, 0);

    let filters = [
        [0x23006, 0x608, 0xffff],
        [0x23006, 0x808, 0xffff],
        [0x23006, 0x8e88, 0xffff],
        [0x2301c, 0x44004300, u32::MAX],
        [0x2301c, 0x43004400, u32::MAX],
        [0x23011, 1, 0xff],
    ];
    for (index, [config, value, mask]) in filters.into_iter().enumerate() {
        write(0x60033120 + index * 4, config);
        write(0x6003313c + index * 4, value);
        write(0x60033158 + index * 4, mask);
    }
    update(0x6003311c, u32::MAX, 0x3f00);
    update(0x6003311c, u32::MAX, 0x7e);
    update(0x6003309c, u32::MAX, 0x8000000);
}

/// Initialize the S3 MAC after power, clocks and PHY have been enabled.
///
/// The caller must own the MAC, hold RX disabled and install the ROM OS adapter.
pub(crate) unsafe fn init() {
    update(0x60033d14, u32::MAX, 2);
    while read(0x60033d14) & 1 == 0 {}
    write(0x60033c34, 0);
    write(0x60033c40, u32::MAX);
    init_tx_rx();
    for interface in 0..4 {
        let policy = 0x600330d8 + 4 * interface;
        update(policy, u32::MAX, 5);
        update(policy, 0xfffff6ff, 0);
        // The blob's hal_mac_rx_set_policy rejects interface 3.
        if interface < 3 {
            update(policy, 0xfffffeef, 0);
            update(0x60033024 + 8 * interface, 0xfffeffff, 0);
            update(0x60033064 + 8 * interface, 0xfffeffff, 0);
        }
    }
    init_rx_buffers();
    unsafe { hal_mac_rate_autoack_init() };
    unsafe { phy_disable_low_rate() };
    write(0x60033410, 0x90a0b);
    write(0x60033414, 0x50100);
    write(0x60033404, 0x90a0b);
    write(0x60033408, 0x50100);
    unsafe {
        hal_crypto_init();
        hal_attenna_init();
    }
    write(0x60033c34, 0x19a879e0);
    update(0x60033c6c, u32::MAX, 0x10000000);
    update(0x6003309c, 0xffffff00, 1);
    update(0x6003309c, 0xffff00ff, 0x200);
    update(0x6003309c, u32::MAX, 0x100000);
    unsafe {
        hal_timer_update_by_rtc(1, crate::ffi::slowclk_cal_get());
        hal_coex_pti_init();
        // Bluetooth coexistence is not active in this driver. The OS adapter's
        // event-3 and event-15 priority lookups leave the initialized zero bytes.
        hal_set_rx_active_pti(0);
        hal_set_rx_ack_pti(0);
        hal_set_wifi_default_pti(0);
    }
}
