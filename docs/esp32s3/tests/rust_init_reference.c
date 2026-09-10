/* Compare Rust initialization with the separately reviewed C reference. */
#define main reference_regression_main
#include "test_hal.c"
#undef main

static int no_coexistence(uint32_t event, uint8_t *value)
{ (void)event; (void)value; return 0; }
static const wifi_osi_funcs_t rust_osi = {slowclk, no_coexistence};

void s3_test_reset(uint32_t seed)
{
    reset();
    for (size_t i=0; i<sizeof(regs)/sizeof(regs[0]); ++i) regs[i]=seed;
    preset(0x60033d14, seed | 1);
    wDevCtrl[0]=0; /* Rust installs its own DMA list after initialization. */
    g_osi_funcs_p=&rust_osi;
    memset(priorities, 0xff, sizeof(priorities));
    memset(slow_args, 0, sizeof(slow_args));
}
void s3_test_reference_init(void) { hal_init(); }
uint32_t s3_test_read(uintptr_t address) { return reg_read(address); }
void s3_test_write(uintptr_t address, uint32_t value) { reg_write(address,value); }
size_t s3_test_trace_length(void) { return ntrace; }
void s3_test_trace_entry(size_t i, uintptr_t *address, uint32_t *value, unsigned char *kind)
{
    assert(i<ntrace);
    *address=trace[i].address; *value=trace[i].value; *kind=trace[i].kind;
}
void s3_test_callbacks(void)
{
    assert(priorities[0]==0 && priorities[1]==0 && priorities[2]==0);
    assert(slow_args[0]==1 && slow_args[1]==0x12345678);
}
