#include "utility.hpp"

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

uint8_t encode_float_to_string(float data, uint8_t *str) {
    int index = 0;

    if (data < 0.0f) {
        str[index++] = '-';
        data = -data;
    }

    uint32_t integer = static_cast<uint32_t>(data);
    uint32_t fraction = static_cast<uint32_t>((data - integer) * 1000.0f);

    char tmp[12];
    int t = 0;

    if (integer == 0) {
        tmp[t++] = '0';
    } else {
        while (integer > 0) {
            tmp[t++] = '0' + integer % 10;
            integer /= 10;
        }
        for (int i = 0; i < t / 2; i++) {
            char c = tmp[i];
            tmp[i] = tmp[t - 1 - i];
            tmp[t - 1 - i] = c;
        }
    }

    for (int i = 0; i < t; i++) {
        str[index++] = tmp[i];
    }

    str[index++] = '.';
    str[index++] = '0' + (fraction / 100);
    str[index++] = '0' + ((fraction / 10) % 10);
    str[index++] = '0' + (fraction % 10);

    return index;
}

void unpack_adc_buffer(uint16_t *buffer) {
    for (uint32_t i = 0; i < adc::BUFFER_SIZE; i++) {
        buffer[i] = unpack_adc_code(buffer[i]);
    }
}

uint32_t reflected_sample_index(int32_t index) {
    if (index < 0) {
        return static_cast<uint32_t>(-index - 1);
    }
    if (index >= static_cast<int32_t>(adc::BUFFER_SIZE)) {
        return static_cast<uint32_t>(
            2 * static_cast<int32_t>(adc::BUFFER_SIZE) - index - 1
        );
    }
    return static_cast<uint32_t>(index);
}