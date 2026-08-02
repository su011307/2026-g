#ifndef DRIVER_ST7789_RP2350_H
#define DRIVER_ST7789_RP2350_H

#include "driver_st7789.h"

#ifdef __cplusplus
extern "C" {
#endif

uint8_t st7789_rp2350_init(void);
uint8_t st7789_rp2350_deinit(void);
st7789_handle_t *st7789_rp2350_get_handle(void);

#ifdef __cplusplus
}
#endif

#endif
