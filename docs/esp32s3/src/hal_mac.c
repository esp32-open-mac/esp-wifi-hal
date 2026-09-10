/* ESP32-S3 MAC member reconstruction, manually checked against IDF v5.4
 * esp32-wifi-lib a29b11bf, libpp.a/hal_mac.o. See ../REVIEW.md.
 * PHY, crypto, rate control and the OS adapter remain externally owned.
 */
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#ifdef S3_HAL_HOST_TEST
#include "host_adapter.h"
#else
#include "esp_attr.h"
#include "esp_private/wifi_os_adapter.h"
_Static_assert(offsetof(wifi_osi_funcs_t, _slowclk_cal_get) == 0x148, "SDK ABI");
_Static_assert(offsetof(wifi_osi_funcs_t, _coex_pti_get) == 0x1a8, "SDK ABI");
#endif

/* Preserve the original member's IDF linker-fragment placement. In particular,
 * interrupt entry points must remain executable while flash cache is disabled. */
#ifndef S3_HAL_HOST_TEST
#define PLACED(section_name) __attribute__((section(section_name)))
int mac_tx_set_pti(void *) PLACED(".wifiextrairam.3");
int hal_mac_tx_config_timeout(void *, uint32_t) PLACED(".wifi0iram.4");
int hal_mac_tx_config_edca(void *) PLACED(".wifi0iram.5");
uintptr_t hal_mac_txq_enable(uint8_t) PLACED(".wifi0iram.6");
uintptr_t hal_mac_txq_disable(uint8_t) PLACED(".wifiextrairam.7");
int hal_mac_tx_get_blockack(uint8_t, void *) PLACED(".wifi0iram.8");
uint32_t hal_random(void) PLACED(".wifi0iram.9");
uint32_t hal_mac_is_low_rate_enabled(void) PLACED(".wifi0iram.10");
uint32_t hal_mac_rx_read_rxdscrlast(void) PLACED(".wifislpiram.11");
uint32_t hal_mac_rx_read_rxdscrnext(void) PLACED(".wifislprxiram.12");
void hal_mac_rx_set_base(uint32_t) PLACED(".wifislprxiram.13");
int hal_mac_is_dma_enable(void) PLACED(".iram1.14");
uint32_t hal_mac_interrupt_get_event(void) PLACED(".iram1.15");
void hal_mac_interrupt_clr_event(uint32_t) PLACED(".iram1.16");
void hal_mac_interrupt_clr_watchdog(void) PLACED(".iram1.17");
uint64_t hal_get_tsf_time(uint32_t) PLACED(".wifiextrairam.18");
int hal_mac_init(void) PLACED(".wifislprxiram.19");
int hal_mac_deinit(void) PLACED(".wifi0iram.20");
#endif
#define REG_INLINE static inline __attribute__((always_inline))

extern const wifi_osi_funcs_t *g_osi_funcs_p;
extern uint32_t wDevCtrl[];
extern uint32_t g_wifi_menuconfig[];
uint32_t g_mac_deinit_count;
uint8_t g_mac_deinit_rxing, g_mac_deinit_txing;

extern void phy_disable_low_rate(void), phy_enable_low_rate(void);
extern void hal_mac_rate_autoack_init(void), hal_crypto_init(void), hal_attenna_init(void);
extern void hal_coex_pti_init(void), hal_disable_sta_tsf(void), wDev_reset_bcnSendTick(void);
extern void hal_set_rx_active_pti(uint32_t), hal_set_rx_ack_pti(uint32_t);
extern void hal_set_wifi_default_pti(uint32_t), hal_timer_update_by_rtc(uint32_t, uint32_t);
extern void hal_set_tx_pti(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t);
extern void mac_tx_set_plcp1(void *), mac_tx_set_plcp2(void *), mac_tx_set_txop_q(void *);
extern void mac_tx_set_htsig(void *, uint32_t);
extern void ets_delay_us(uint32_t);

/* GCC supplies MEMW for volatile accesses on this target. Each RMW deliberately
 * remains separate where the original has two distinct hardware transactions. */
#ifndef S3_HAL_HOST_TEST
REG_INLINE uint32_t reg_read(uintptr_t address) { return *(volatile uint32_t *)address; }
REG_INLINE void reg_write(uintptr_t address, uint32_t value) { *(volatile uint32_t *)address = value; }
#endif
REG_INLINE void reg_update(uintptr_t address, uint32_t keep, uint32_t set)
{ reg_write(address, (reg_read(address) & keep) | set); }
REG_INLINE uint32_t load32(const void *p, size_t offset)
{ return *(const uint32_t *)((const uint8_t *)p + offset); }
REG_INLINE uint16_t load16(const void *p, size_t offset)
{ return *(const uint16_t *)((const uint8_t *)p + offset); }
REG_INLINE uint8_t load8(const void *p, size_t offset)
{ return *((const uint8_t *)p + offset); }
REG_INLINE void *pointer32(const void *p, size_t offset)
{ return (void *)(uintptr_t)load32(p, offset); }
REG_INLINE uintptr_t tx_control(uint8_t slot) { return 0x60033d08u - 8u * slot; }
REG_INLINE uintptr_t rx_policy(uint8_t interface) { return 0x600330d8u + 4u * interface; }

int mac_tx_set_plcp0(void *tx)
{
    const void *buffer = pointer32(tx, 0);
    const void *info = pointer32(buffer, 44);
    uint32_t plcp = (load32(buffer, 4) & 0xfffffu) | 0x200000u;
    if ((int16_t)load16(tx, 20) < 1 && (load32(info, 0) & 0xc0u) != 0x80u &&
        (uint8_t)(load8(info, 12) - 16u) > 15u)
        plcp |= 0x400000u;
    uint32_t flags = load32(info, 0);
    if (!(flags & 0x402u) && (flags & 0x480000u) != 0x400000u) {
        uint32_t mode = flags & (1u << 20) ? 3u : flags & (1u << 19) ? 2u : 1u;
        plcp = (plcp & 0xf8ffffffu) | (mode << 24);
    }
    flags = load32(info, 0);
    plcp = (plcp & 0xe7ffffffu) | ((flags & 0x300u) << 19);
    reg_write(tx_control(load8(tx, 4)), plcp);
    return 0;
}

int mac_tx_set_duration(void *tx)
{
    const void *buffer = pointer32(tx, 0);
    const uint8_t *entry = pointer32(pointer32(buffer, 4), 4);
    /* The flag is in the pointee, not in the pointer word at buffer+44. */
    if (load32(pointer32(buffer, 44), 0) & (1u << 18)) entry += 8;
    uint32_t duration = load16(entry, 2);
    reg_write(0x60034318u - 76u * load8(tx, 4), duration | (duration << 16));
    return 0;
}

int mac_tx_set_pti(void *tx)
{
    const void *buffer = pointer32(tx, 0);
    uint8_t before = load8(pointer32(buffer, 44), 32);
    uint8_t requested = before;
    g_osi_funcs_p->_coex_pti_get(1, &requested);
    const void *info = pointer32(buffer, 44); /* reload after the callback */
    uint32_t current = load8(info, 32);
    /* MINU uses the saved pre-callback byte. Argument seven is at [a1+0]. */
    hal_set_tx_pti(load8(tx, 4), before < requested ? before : requested,
                   current, current, current, current, load16(info, 34));
    return 0;
}

int hal_mac_tx_set_ppdu(void *tx, uint32_t mode)
{
    mac_tx_set_plcp0(tx); mac_tx_set_plcp1(tx); mac_tx_set_plcp2(tx);
    mac_tx_set_duration(tx); mac_tx_set_htsig(tx, mode);
    mac_tx_set_txop_q(tx); mac_tx_set_pti(tx);
    return 0;
}
int hal_mac_tx_config_timeout(void *tx, uint32_t timeout)
{ reg_update(0x60033d04u - 8u * load8(tx, 4), 0xfffff000u, timeout & 0xfffu); return 0; }
int hal_mac_tx_config_edca(void *tx)
{
    reg_update(0x60033d04u - 8u * load8(tx, 4), 0xf0ffffffu, (load8(tx, 5) & 15u) << 24);
    reg_update(0x60033d04u - 8u * load8(tx, 4), 0xffc00fffu, (load16(tx, 6) & 0x3ffu) << 12);
    return 0;
}
uintptr_t hal_mac_txq_enable(uint8_t slot)
{ uintptr_t p = tx_control(slot); reg_update(p, UINT32_MAX, 0xc0000000u); return p; }
uintptr_t hal_mac_txq_disable(uint8_t slot)
{ uintptr_t p = tx_control(slot); reg_update(p, 0x3fffffffu, 0); return p; }
uintptr_t hal_mac_set_txq_invalid(uint8_t slot)
{ uintptr_t p = tx_control(slot); reg_update(p, 0xbfffffffu, 0); return p; }
uint32_t hal_mac_is_txq_valid(uint8_t slot) { return (reg_read(tx_control(slot)) >> 30) & 1u; }
uint32_t hal_mac_is_txq_enabled(uint8_t slot) { return reg_read(tx_control(slot)) >> 31; }
int hal_mac_tx_get_blockack(uint8_t slot, void *result)
{
    uintptr_t base = 0x60034300u - 76u * slot;
    *(uint8_t *)result = (reg_read(base + 0x34) >> 12) & 15;
    *(uint16_t *)((uint8_t *)result + 2) = reg_read(base + 0x34) & 0xfff;
    *(uint32_t *)((uint8_t *)result + 4) = reg_read(base + 0x28);
    *(uint32_t *)((uint8_t *)result + 8) = reg_read(base + 0x24);
    return 0;
}
uint32_t hal_random(void) { return reg_read(0x6003507c); }
uint32_t hal_now(void) { return reg_read(0x60035000); }
int hal_mac_tx_set_cca(uint32_t value)
{ reg_update(0x60033c50, 0x3fffffff, value << 30); return 0; }
void hal_mac_disable_low_rate(void)
{
    phy_disable_low_rate();
    reg_write(0x60033410, 0x90a0b); reg_write(0x60033414, 0x50100);
    reg_write(0x60033404, 0x90a0b); reg_write(0x60033408, 0x50100);
}
void hal_mac_enable_low_rate(void)
{
    phy_enable_low_rate();
    reg_write(0x60033410, 0xb0b0b0b); reg_write(0x60033414, 0xb0b0b0b);
    reg_write(0x60033404, 0xb0b0b0b); reg_write(0x60033408, 0xb0b0b0b);
}
uint32_t hal_mac_is_low_rate_enabled(void)
{ return (reg_read(0x6001c860) & 0xc00) == 0xc00 ? (reg_read(0x6001c87c) >> 11) & 1 : 0; }

void mac_rxbuf_init(void)
{
    reg_update(0x60033c5c, 0xfff00000, 0xf8000);
    reg_update(0x60033c60, 0xfff00000, 0x84000);
    reg_update(0x60033c64, 0x000fffff, 0x3fc00000);
    reg_update(0x60033080, 0xffffff00, 0);
    reg_write(0x60033088, wDevCtrl[0]);
}
uint32_t hal_disable_mac(void)
{
    reg_update(0x60033c00, UINT32_MAX, 0xf0);
    uint32_t enabled = reg_read(0x60033c34);
    reg_write(0x60033c34, 0); reg_write(0x60033c38, 0); reg_write(0x60033c24, 0);
    return enabled;
}
void hal_enable_mac(uint32_t interrupts, uint32_t rx_base)
{ reg_update(0x60033c00, 0xffffff0f, 0); reg_write(0x60033c34, interrupts); reg_write(0x60033088, rx_base); }
uint32_t hal_mac_rx_read_rxdscrlast(void) { return reg_read(0x60033090); }
uint32_t hal_mac_rx_read_rxdscrnext(void) { return reg_read(0x6003308c); }
void hal_mac_rx_set_base(uint32_t base) { reg_write(0x60033088, base); }

void mac_txrx_init(void)
{
    reg_update(0x60033c6c, UINT32_MAX, 0x8080a000);
    reg_update(0x60033c6c, UINT32_MAX, 0x100);
    for (unsigned i = 0; i < 4; ++i) {
        reg_update(rx_policy(i), UINT32_MAX, 0x40);
        reg_update(rx_policy(i), 0xffffffdf, 0);
    }
    reg_update(0x60033c74, UINT32_MAX, 8);
    /* These are 32-bit MMIO reads masked to 16 bits, not halfword reads. */
    for (unsigned i = 0; i < 4; ++i) reg_update(0x60033100u + 4u * i, 0xffff, 0);
    reg_update(0x60033100, UINT32_MAX, 0x1000000);
    reg_update(0x60033104, UINT32_MAX, 0x1000000);
    reg_update(0x60033100, UINT32_MAX, 0x4000000);
    reg_update(0x60033104, UINT32_MAX, 0x4000000);
    reg_update(0x60033c6c, UINT32_MAX, 0x200);
    reg_update(0x60033114, 0xffffff0f, 0);
    reg_update(0x60033118, UINT32_MAX, 0x80000000);
    reg_update(0x60033118, 0xf00fffff, 0x1b00000);
    reg_update(0x60033c78, UINT32_MAX, 3);
    reg_update(0x60033c10, 0xfffff000, 0xf0);
    reg_update(0x60033c10, UINT32_MAX, 0x80000000);
    reg_update(0x60033c10, UINT32_MAX, 0x40000000);
    reg_update(0x60033c14, 0xfffff000, 0xf0);
    reg_update(0x60033c18, 0xfffff000, 0xf0);
    reg_update(0x60033c94, 0xffffff0f, 0x40);
    reg_update(0x60033c54, UINT32_MAX, 0x7fff0000);
    reg_update(0x60033c54, UINT32_MAX, 0x80000000);
    reg_update(0x60033c88, 0xf0ffffff, 0);
    reg_update(0x600332b8, UINT32_MAX, 2);
    reg_update(0x60033084, 0x7fffffff, 0);
}
uint32_t hal_mac_set_rxq_policy(uint8_t interface, uint8_t enabled)
{
    uintptr_t p = rx_policy(interface);
    uint32_t value = enabled ? reg_read(p) | 2u : reg_read(p) & ~2u;
    reg_write(p, value);
    return enabled ? value : (uint32_t)p;
}
void mac_last_rxbuf_init(void)
{
    /* Original MMIO ordering is interleaved by filter, although HLIL folds it
     * into two memcpy expressions. Keep 32-bit stores and that ordering. */
    static const uint32_t filters[6][3] = {
        {0x23006, 0x608, 0xffff}, {0x23006, 0x808, 0xffff},
        {0x23006, 0x8e88, 0xffff}, {0x2301c, 0x44004300, UINT32_MAX},
        {0x2301c, 0x43004400, UINT32_MAX}, {0x23011, 1, 0xff}
    };
    for (unsigned i = 0; i < 6; ++i) {
        reg_write(0x60033120u + i * 4, filters[i][0]);
        reg_write(0x6003313cu + i * 4, filters[i][1]);
        reg_write(0x60033158u + i * 4, filters[i][2]);
    }
    reg_update(0x6003311c, UINT32_MAX, 0x3f00);
    reg_update(0x6003311c, UINT32_MAX, 0x7e);
    reg_update(0x6003309c, UINT32_MAX, 0x8000000);
}
void hal_deinit(void)
{
    hal_set_rx_active_pti(0); hal_set_rx_ack_pti(0); hal_set_wifi_default_pti(0);
    reg_update(0x600332b8, UINT32_MAX, 1);
    reg_update(0x60033c34, 0xe657861f, 0);
    reg_write(0x60033c34, 0); reg_write(0x60033c40, UINT32_MAX);
    reg_update(0x60033d14, UINT32_MAX, 2);
    while (!(reg_read(0x60033d14) & 1)) {}
}
int hal_mac_is_dma_enable(void) { (void)reg_read(0x60035128); return 0; }
uint32_t hal_mac_interrupt_get_event(void) { return reg_read(0x60033c3c); }
void hal_mac_interrupt_clr_event(uint32_t value) { reg_write(0x60033c40, value); }
void hal_mac_interrupt_clr_watchdog(void) { reg_update(0x60033c40, UINT32_MAX, 0x800); }
static uint32_t address_low(const uint8_t *a)
{ return (uint32_t)a[0] | (uint32_t)a[1] << 8 | (uint32_t)a[2] << 16 | (uint32_t)a[3] << 24; }
uintptr_t hal_mac_set_addr(uint8_t interface, const uint8_t *address)
{
    uintptr_t base = 0x60033040u + 8u * interface;
    reg_write(base, address_low(address));
    reg_write(base + 4, address[4] | (uint32_t)address[5] << 8);
    reg_write(base + 32, UINT32_MAX); reg_write(base + 36, 0xffff);
    reg_update(base + 36, UINT32_MAX, 0x10000);
    return base;
}
uint32_t hal_mac_set_bssid(uint8_t interface, const uint8_t *address)
{
    uintptr_t base = 0x60033000u + 8u * interface;
    reg_update(base + 36, 0xfffeffff, 0);
    reg_write(base, address_low(address));
    reg_update(base + 4, 0xffff0000, address[4] | (uint32_t)address[5] << 8);
    reg_write(base + 32, UINT32_MAX); reg_write(base + 36, 0xffff);
    uint32_t value = reg_read(base + 36) | 0x10000;
    reg_write(base + 36, value); return value;
}
int hal_mac_rx_set_policy(uint8_t interface, uint32_t policy, uint32_t bssid, uint32_t receiver)
{
    if (interface >= 3) return -1;
    reg_update(rx_policy(interface), policy >= 2 ? UINT32_MAX : 0xfffffeef, policy >= 2 ? 0x110 : 0);
    uintptr_t base = 0x60033000u + 8u * interface;
    if (bssid < 2) reg_update(base + 36, bssid ? UINT32_MAX : 0xfffeffff, bssid ? 0x10000 : 0);
    else {
        static const uint8_t broadcast[6] = {255,255,255,255,255,255};
        hal_mac_set_bssid(interface, broadcast);
    }
    reg_update(base + 0x64, receiver ? UINT32_MAX : 0xfffeffff, receiver ? 0x10000 : 0);
    return 0;
}
void hal_init(void)
{
    reg_update(0x60033d14, UINT32_MAX, 2);
    while (!(reg_read(0x60033d14) & 1)) {}
    reg_write(0x60033c34, 0); reg_write(0x60033c40, UINT32_MAX);
    mac_txrx_init();
    for (unsigned i = 0; i < 4; ++i) {
        reg_update(rx_policy(i), UINT32_MAX, 5);
        reg_update(rx_policy(i), 0xfffff6ff, 0);
        hal_mac_rx_set_policy(i, 0, 0, 0);
    }
    mac_rxbuf_init(); mac_last_rxbuf_init(); hal_mac_rate_autoack_init();
    hal_mac_disable_low_rate(); hal_crypto_init(); hal_attenna_init();
    reg_write(0x60033c34, 0x19a879e0);
    reg_update(0x60033c6c, UINT32_MAX, 0x10000000);
    reg_update(0x6003309c, 0xffffff00, 1);
    reg_update(0x6003309c, 0xffff00ff, 0x200);
    reg_update(0x6003309c, UINT32_MAX, 0x100000);
    hal_timer_update_by_rtc(1, g_osi_funcs_p->_slowclk_cal_get());
    hal_coex_pti_init();
    uint8_t ack = 0, default_pti = 0;
    g_osi_funcs_p->_coex_pti_get(3, &ack);
    g_osi_funcs_p->_coex_pti_get(15, &default_pti);
    /* Active PTI is intentionally zero in the original instruction stream. */
    hal_set_rx_active_pti(0); hal_set_rx_ack_pti(ack); hal_set_wifi_default_pti(default_pti);
}
uintptr_t hal_mac_clr_bssid(uint8_t interface)
{ uintptr_t p = 0x60033024u + 8u * interface; reg_update(p, 0xfffeffff, 0); return p; }
uint64_t hal_mac_tsf_get_time(uint32_t interface)
{
    if (interface > 1) return 0;
    uint32_t mask = interface ? 4 : 2;
    reg_update(0x6003500c, UINT32_MAX, mask);
    uint32_t high = interface ? reg_read(0x6003501c) : 0;
    uint32_t low = reg_read(0x60035018);
    reg_update(0x6003500c, ~mask, 0);
    return ((uint64_t)high << 32) | low;
}
void wDev_Mesh_Disable_Tsf(void) { hal_disable_sta_tsf(); }
uint64_t hal_get_tsf_time(uint32_t interface)
{
    uint32_t mask = interface ? 2 : 1;
    reg_update(0x6003500c, UINT32_MAX, mask);
    uint32_t high = reg_read(0x6003501c), low = reg_read(0x60035018);
    reg_update(0x6003500c, ~mask, 0);
    return ((uint64_t)high << 32) | low;
}
/* The 64-bit argument is aligned to a4/a5; a3 is padding, not an argument. */
void hal_mac_tsf_set_time(uint32_t interface, uint64_t time)
{
    if (interface != 1) return;
    reg_write(0x60035010, (uint32_t)time); reg_write(0x60035014, time >> 32);
    reg_update(0x6003500c, UINT32_MAX, 0x20);
}
void hal_mac_tsf_reset(uint32_t interface)
{
    if (!interface) {
        reg_update(0x60035034, 0x3fffffff, 0); wDev_reset_bcnSendTick();
        reg_write(0x60035010, 0); reg_write(0x60035014, 0);
        reg_update(0x6003500c, UINT32_MAX, 0x20);
        reg_update(0x60035034, UINT32_MAX, 0xc0000000);
    } else if (interface == 1) {
        reg_update(0x60035028, 0x7fffffff, 0);
        reg_write(0x60035010, 0); reg_write(0x60035014, 0);
        reg_update(0x6003500c, UINT32_MAX, 0x10);
        reg_update(0x60035028, UINT32_MAX, 0x80000000);
        reg_update(0x60035034, 0x7fffffff, 0);
        reg_write(0x60035010, 0); reg_write(0x60035014, 0);
        reg_update(0x6003500c, UINT32_MAX, 0x20);
        reg_update(0x60035034, UINT32_MAX, 0x80000000);
    }
}
int hal_mac_set_csi(uint8_t enable)
{
    if (!g_wifi_menuconfig[6]) return -1;
    reg_update(0x6003309c, enable ? UINT32_MAX : 0xff7fffff, enable ? 0x800000 : 0);
    return 0;
}
int hal_mac_init(void) { reg_update(0x60033ca0, 0xff00efff, 0); return 0; }
int hal_mac_deinit(void)
{
    reg_update(0x60033ca0, UINT32_MAX, 0xff1000); ets_delay_us(20);
    while (reg_read(0x60033ca0) & 0x6000) {}
    ets_delay_us(5); return 0;
}
void hal_mac_rx_enable(void) { reg_update(0x60033084, UINT32_MAX, 0x80000000); }
void hal_mac_rx_disable(void) { reg_update(0x60033084, 0x7fffffff, 0); }
void hal_enable_sta_beacon_filter(void)
{
    reg_update(0x60033c34, 0xffff7fff, 0);
    reg_update(0x60033c34, UINT32_MAX, 0x8000);
    reg_update(0x600330a0, UINT32_MAX, 7);
}
void hal_disable_sta_beacon_filter(void)
{ reg_update(0x600330a0, 0xfffffff8, 0); reg_update(0x60033c34, 0xffff7fff, 0); }
uint32_t hal_set_sta_beacon_filter(uint32_t value)
{
    uint32_t result = (reg_read(0x60033004) & 0xf800ffffu) | ((value & 0x7ffu) << 16);
    reg_write(0x60033004, result); return result;
}
void hal_enable_sta_dump_beacon(void)
{ reg_update(0x600330e8, 0xffff00ff, 0x8100); reg_update(0x600330e8, UINT32_MAX, 0xff); }
void hal_beacon_ie_crc_get(uint32_t *value) { *value = reg_read(0x600332b0); }
void hal_beacon_ie_crc_set(uint32_t value) { reg_write(0x600332b4, value); }
uintptr_t hal_rx_disable_bssid_check(uint8_t interface)
{ uintptr_t p = rx_policy(interface); reg_update(p, 0xfffffffd, 0); return p; }
uintptr_t hal_rx_enable_bssid_check(uint8_t interface)
{ uintptr_t p = rx_policy(interface); reg_update(p, UINT32_MAX, 2); return p; }
void mac_rxbuf_disable(void) {}
uint64_t hal_mac_ftm_get_t3(const void *descriptor)
{
    /* MULUH, carry and a3 return are absent from the exported HLIL. */
    return (((uint64_t)load32(descriptor, 36) * 80 + (load32(descriptor, 40) & 127)) << 3) - 0x1400;
}
