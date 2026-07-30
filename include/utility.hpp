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

float compute_adc_voltage(int32_t raw);
void unpack_adc_buffer(uint16_t *buffer);
uint32_t reflected_sample_index(int32_t index);
void uart_write_float(uart_inst_t *const uart_port, float value);