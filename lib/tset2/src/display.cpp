#include <cstdint>
#include <hardware/uart.h>
#include <pico/time.h>

#include "display.hpp"

Screen::Screen(uart_inst_t *uart_port, uart_inst_t *debug_port) : uart_port_(uart_port), debug_port_(debug_port) {}

static uint8_t embed_uint_to_string(
    uint16_t data,
    uint8_t *output_buffer
) {
    uint8_t idx = 0;
    constexpr char prefix[] = "addt s0.id,0,";

    for (uint8_t i = 0; i < sizeof(prefix) - 1; ++i) {
        output_buffer[idx++] = static_cast<uint8_t>(prefix[i]);
    }

    uint8_t digits[5] = {};
    uint8_t digit_count = 0;
    do {
        digits[digit_count++] =
            static_cast<uint8_t>('0' + data % 10);
        data /= 10;
    } while (data != 0);
    while (digit_count > 0) {
        output_buffer[idx++] = digits[--digit_count];
    }

    output_buffer[idx++] = 0xff;
    output_buffer[idx++] = 0xff;
    output_buffer[idx++] = 0xff;

    return idx;
}

static uint8_t embed_string_to_string(
    const char *target_string,
    uint8_t string_length,
    uint8_t *output_buffer
) {
    uint8_t idx = 0;
    constexpr char prefix[] = "slt0.txt=\"";
    for (uint8_t i = 0; i < sizeof(prefix) - 1; ++i) {
        output_buffer[idx++] = static_cast<uint8_t>(prefix[i]);
    }

    for (uint8_t i = 0; i < string_length; ++i) {
        output_buffer[idx++] =
            static_cast<uint8_t>(target_string[i]);
    }

    output_buffer[idx++] = '"';
    output_buffer[idx++] = 0xff;
    output_buffer[idx++] = 0xff;
    output_buffer[idx++] = 0xff;

    return idx;
}

void Screen::send_graph(const float data[], uint16_t length) {
    uint8_t string_buffer[24] = {};
    auto size = embed_uint_to_string(length, string_buffer);
    uart_write_blocking(uart_port_, string_buffer, size);
    sleep_ms(100);
    for (uint16_t i = 0; i < length; ++i) {
        float display_value = data[i] * stretch_ + offset_;
        if (display_value < 0.0f) {
            display_value = 0.0f;
        } else if (display_value > 255.0f) {
            display_value = 255.0f;
        }
        uart_putc(
            uart_port_,
            static_cast<uint8_t>(display_value + 0.5f)
        );
    }

    constexpr uint8_t graph_end[] = {0x01, 0xff, 0xff, 0xff};
    uart_write_blocking(uart_port_, graph_end, sizeof(graph_end));
}

void Screen::send_text(const char *text, uint8_t length) {
    uint8_t string_buffer[96] = {};
    constexpr uint8_t command_overhead = 14;
    const uint8_t safe_length =
        length <= sizeof(string_buffer) - command_overhead
            ? length
            : sizeof(string_buffer) - command_overhead;
    auto size = embed_string_to_string(
        text,
        safe_length,
        string_buffer
    );
    uart_write_blocking(uart_port_, string_buffer, size);
    uart_write_blocking(debug_port_, string_buffer, size);
}
