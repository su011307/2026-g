#include "utility.hpp"

#include "definition.hpp"

// 提取出原始 12b ADC 数据。其中 OTR、CLK 被 discard
static uint16_t unpack_adc_code(uint16_t raw) {
    const uint16_t d11_d6 =
        static_cast<uint16_t>((raw & 0x1f80u) >> 1);
    const uint16_t d5_d4 =
        static_cast<uint16_t>(
            ((raw & 0x0001u) << 5) |
            ((raw & 0x0002u) << 3)
        );
    const uint16_t d3_d0 =
        static_cast<uint16_t>((raw & 0x0078u) >> 3);
    return static_cast<uint16_t>(d11_d6 | d5_d4 | d3_d0);
}

float compute_adc_voltage(int32_t raw) {
    return (5.0f - adc::VOLT_CONVERT_CONSTANT * raw) * 1000 + adc::ZERO_DRIFT_COMPENSATE; // metric: mV
}

void unpack_adc_buffer(uint16_t *buffer) {
    for (uint32_t i = 0; i < sig::BUFFER_SIZE; i++) {
        buffer[i] = unpack_adc_code(buffer[i]);
    }
}
