#define _GNU_SOURCE
#define S3_HAL_HOST_TEST
#include <assert.h>
#include <stdio.h>
#include <sys/mman.h>
#include "../src/hal_mac.c"

struct access { uintptr_t address; uint32_t value; char kind; } trace[4096];
static size_t ntrace;
static uint32_t regs[0x40000 / 4];
uint32_t wDevCtrl[4], g_wifi_menuconfig[8];
static uint32_t pti_args[7], priorities[3], slow_args[2], delays[4];
static unsigned ndelays, callback_count;
static uint8_t *changed_info;
static uint32_t reg_read(uintptr_t address)
{
    assert(address >= 0x60000000 && address < 0x60040000 && !(address & 3));
    uint32_t value = regs[(address - 0x60000000) / 4];
    assert(ntrace < 4096); trace[ntrace++] = (struct access){address, value, 'r'};
    return value;
}
static void reg_write(uintptr_t address, uint32_t value)
{
    assert(address >= 0x60000000 && address < 0x60040000 && !(address & 3));
    regs[(address - 0x60000000) / 4] = value;
    assert(ntrace < 4096); trace[ntrace++] = (struct access){address, value, 'w'};
}
static void preset(uintptr_t address, uint32_t value) { regs[(address - 0x60000000) / 4] = value; }
static uint32_t peek(uintptr_t address) { return regs[(address - 0x60000000) / 4]; }
static void reset(void) { memset(regs, 0, sizeof(regs)); ntrace = callback_count = 0; changed_info = NULL; }
static uint32_t slowclk(void) { return 0x12345678; }
static int coex(uint32_t event, uint8_t *value)
{
    callback_count++;
    if (event == 1) { assert(*value == 9); *value = 11; changed_info[32] = 2; }
    else if (event == 3) *value = 7;
    else { assert(event == 15); *value = 13; }
    return 0;
}
static const wifi_osi_funcs_t osi = {slowclk, coex};
const wifi_osi_funcs_t *g_osi_funcs_p = &osi;
void hal_set_tx_pti(uint32_t a,uint32_t b,uint32_t c,uint32_t d,uint32_t e,uint32_t f,uint32_t g)
{ uint32_t values[] = {a,b,c,d,e,f,g}; memcpy(pti_args, values, sizeof(values)); }
void hal_timer_update_by_rtc(uint32_t a,uint32_t b) { slow_args[0]=a; slow_args[1]=b; }
void hal_set_rx_active_pti(uint32_t v) { priorities[0]=v; }
void hal_set_rx_ack_pti(uint32_t v) { priorities[1]=v; }
void hal_set_wifi_default_pti(uint32_t v) { priorities[2]=v; }
void ets_delay_us(uint32_t v) { assert(ndelays < 4); delays[ndelays++]=v; }
#define STUB(name) void name(void) {}
STUB(phy_disable_low_rate) STUB(phy_enable_low_rate) STUB(hal_mac_rate_autoack_init)
STUB(hal_crypto_init) STUB(hal_attenna_init) STUB(hal_coex_pti_init)
STUB(hal_disable_sta_tsf) STUB(wDev_reset_bcnSendTick)
void mac_tx_set_plcp1(void *v) { (void)v; } void mac_tx_set_plcp2(void *v) { (void)v; }
void mac_tx_set_txop_q(void *v) { (void)v; } void mac_tx_set_htsig(void *v,uint32_t n) { (void)v; (void)n; }
static void store32(void *p, size_t off, uint32_t v) { memcpy((uint8_t *)p + off, &v, 4); }
static void store16(void *p, size_t off, uint16_t v) { memcpy((uint8_t *)p + off, &v, 2); }

int main(void)
{
    /* Test real 32-bit pointer fields without changing the embedded layout. */
    uint8_t *arena = mmap(NULL, 4096, PROT_READ|PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS|MAP_32BIT, -1, 0);
    assert(arena != MAP_FAILED && (uintptr_t)arena + 4096 <= UINT32_MAX);
    uint8_t *tx=arena, *buffer=arena+128, *info=arena+256, *descriptor=arena+384, *entry=arena+512;
    store32(tx,0,(uintptr_t)buffer); tx[4]=3;
    store32(buffer,4,(uintptr_t)descriptor); store32(buffer,44,(uintptr_t)info);
    store32(descriptor,4,(uintptr_t)entry); store16(entry,2,0x1234); store16(entry,10,0xabcd);
    reset(); store32(info,0,0); mac_tx_set_duration(tx);
    assert(peek(0x60034234)==0x12341234); /* slot 3, base - 3*76 */
    store32(info,0,1u<<18); mac_tx_set_duration(tx); assert(peek(0x60034234)==0xabcdabcd);
    puts("PASS duration: pointer chain, both flag branches, slot stride, 16-bit load");

    reset(); info[32]=9; store16(info,34,0xbeef); changed_info=info;
    mac_tx_set_pti(tx);
    assert(callback_count==1 && pti_args[0]==3 && pti_args[1]==9 && pti_args[6]==0xbeef);
    for(unsigned i=2;i<6;i++) assert(pti_args[i]==2);
    puts("PASS PTI: OS table, outer slot, saved minimum, callback reload, seventh argument");

    reset(); store32(buffer,4,0x12345); store16(tx,20,0); info[12]=0;
    store32(info,0,(1u<<20)|(1u<<8)|(1u<<9)); mac_tx_set_plcp0(tx);
    assert(peek(0x60033cf0)==0x1b612345);
    store32(info,0,0x80); mac_tx_set_plcp0(tx); assert(peek(0x60033cf0)==0x01212345);
    puts("PASS PLCP: header flags, short-rate branch, descending queue address");

    reset(); tx[5]=0xfa; store16(tx,6,0xabc); preset(0x60033cec,UINT32_MAX);
    hal_mac_tx_config_edca(tx); assert(peek(0x60033cec)==0xfaebcfff);
    puts("PASS EDCA: byte/halfword fields and reserved bits");

    reset(); preset(0x60035018,0x89abcdef); preset(0x6003501c,0x12345678);
    preset(0x6003500c,0x8000);
    assert(hal_get_tsf_time(0)==UINT64_C(0x1234567889abcdef));
    assert(hal_mac_tsf_get_time(1)==UINT64_C(0x1234567889abcdef));
    assert(hal_mac_tsf_get_time(0)==UINT64_C(0x89abcdef));
    assert(hal_mac_tsf_get_time(2)==0 && peek(0x6003500c)==0x8000);
    hal_mac_tsf_set_time(1,UINT64_C(0xfeedface01234567));
    assert(peek(0x60035010)==0x01234567 && peek(0x60035014)==0xfeedface);
    puts("PASS TSF: 64-bit return, interface-specific latch, 64-bit set argument");

    store32(descriptor,36,UINT32_MAX); store32(descriptor,40,0xff);
    assert(hal_mac_ftm_get_t3(descriptor)==UINT64_C(2748779064696));
    store32(descriptor,36,0); store32(descriptor,40,0);
    assert(hal_mac_ftm_get_t3(descriptor)==UINT64_MAX-5119);
    puts("PASS FTM: multiply high word, carry, underflow and 64-bit return");

    reset(); mac_last_rxbuf_init();
    const uintptr_t first[] = {0x60033120,0x6003313c,0x60033158,0x60033124,0x60033140,0x6003315c};
    for(unsigned i=0;i<6;i++) assert(trace[i].kind=='w' && trace[i].address==first[i]);
    assert(peek(0x60033148)==0x44004300 && peek(0x6003314c)==0x43004400);
    puts("PASS RX filters: interleaved MMIO writes and protocol constants");

    reset(); preset(0x60033d14,1); wDevCtrl[0]=0x3fc81234; hal_init();
    assert(callback_count==2 && priorities[0]==0 && priorities[1]==7 && priorities[2]==13);
    assert(slow_args[0]==1 && slow_args[1]==0x12345678 && peek(0x60033088)==wDevCtrl[0]);
    assert(peek(0x60033c34)==0x19a879e0 && peek(0x6003309c)==0x8100201);
    assert(peek(0x60033100)==0x5000000 && peek(0x600330e4)==0x45);
    puts("PASS initialization: callbacks, priorities, descriptor base, policy and interrupt masks");

    reset(); uint8_t mac[]={2,3,4,5,6,7}; preset(0x6003300c,0xbeef0000);
    hal_mac_set_bssid(1,mac); assert(peek(0x60033008)==0x05040302 && peek(0x6003300c)==0xbeef0706);
    assert(hal_mac_rx_set_policy(3,2,2,1)==-1);
    hal_mac_rx_set_policy(2,2,2,1); assert(peek(0x60033010)==UINT32_MAX && peek(0x60033034)==0x1ffff);
    puts("PASS filters: interface bounds, BSSID masking and broadcast branch");
    reset(); ndelays=0; hal_mac_deinit(); assert(ndelays==2 && delays[0]==20 && delays[1]==5);
    munmap(arena,4096); puts("PASS deinit: both ROM delays");
    return 0;
}
