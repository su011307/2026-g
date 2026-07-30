#pragma once
#include <stddef.h>
#include <stdint.h>

#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/uart.h>

#include <pico/stdlib.h>
#include <pico/time.h>

#include "definition.hpp"
#include "read.pio.h"

void pio_init(
    PIO pio,
    uint sm,
    uint offset
);

uint dma_init(
    PIO pio,
    uint sm,
    uint16_t *arena_ptr,
    size_t sample_count
);