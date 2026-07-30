#pragma once
#include <stddef.h>
#include <stdint.h>

#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/uart.h>

#include <pico/platform/compiler.h>
#include <pico/stdlib.h>
#include <pico/time.h>

#include "definition.hpp"

float compute_adc_voltage(int32_t raw);
uint8_t encode_float_to_string(float data, uint8_t *str);
void unpack_adc_buffer(uint16_t *buffer);

uint32_t __force_inline reflected_sample_index(int32_t index) {
    if (index < 0) {
        return static_cast<uint32_t>(-index - 1);
    }
    if (index >= static_cast<int32_t>(sig::BUFFER_SIZE)) {
        return static_cast<uint32_t>(
            2 * static_cast<int32_t>(sig::BUFFER_SIZE) - index - 1
        );
    }
    return static_cast<uint32_t>(index);
}

void uart_write_float(uart_inst_t *const uart_port, float value);
