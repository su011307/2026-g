#pragma once
#include <stdint.h>

float compute_adc_voltage(int32_t raw);
void unpack_adc_buffer(uint16_t *buffer);
