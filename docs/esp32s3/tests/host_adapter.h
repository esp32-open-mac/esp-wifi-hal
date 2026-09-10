#ifndef S3_HAL_HOST_ADAPTER_H
#define S3_HAL_HOST_ADAPTER_H
#include <stdint.h>
typedef struct {
    uint32_t (*_slowclk_cal_get)(void);
    int (*_coex_pti_get)(uint32_t, uint8_t *);
} wifi_osi_funcs_t;
static uint32_t reg_read(uintptr_t address);
static void reg_write(uintptr_t address, uint32_t value);
#endif
