#include <cstdint>
#include <hardware/uart.h>
#include "display.hpp"

Screen::Screen(uart_inst_t *const uart_port) : uart_port_(uart_port) {}

uint8_t embed_uint_to_string(
    uint16_t data, 
    uint8_t *output_buffer
) {
    uint8_t idx = 0;
    uint32_t heading = 10000;

    for (auto i : "addt s0.id,0,") {
        output_buffer[idx++] = i;
    }
    
    while (data % heading != 0) {
        auto digit = data % heading;
        output_buffer[idx++] = '0' + digit;
    }

    for (auto i: "\xff\xff\xff") {
        output_buffer[idx++] = i;
    }

    return idx;
}

uint8_t embed_string_to_string(
    char *target_string,
    uint8_t string_length,
    uint8_t *output_buffer
) {
    auto idx = 0;
    for (auto i : "slt0.txt=\"") {
        output_buffer[idx++] = i;
    }

    for (auto i = 0; i < string_length; i++) {
        output_buffer[idx++] = target_string[i];
    }

    for (auto i : "\"\xff\xff\xff") {
        output_buffer[idx++] = i;
    }

    return idx;
}

void Screen::send_graph(float data[], uint16_t length)
{
    uint8_t string_buffer[20] = {};
    auto size = embed_uint_to_string(length, string_buffer);
    uart_write_blocking(uart_port_, string_buffer, size);
    sleep_ms(100);
    for(uint16_t i = 0; i < length; i++) {
        uint8_t data_casted = (int)(data[i] * strentch + offset) % 256;
        uart_putc(uart_port_, data_casted);
    }

    uart_puts(uart_port_, "\x01\xff\xff\xff");
}


void Screen::send_text(char *t, uint8_t length) {
    uint8_t string_buffer[64] = {};
    auto size = embed_string_to_string(t, length, string_buffer);
    uart_write_blocking(uart_port_, string_buffer, size);
    // printf("slt0.txt=\"%s\"\xff\xff\xff", t);
}

Screen::~Screen() {}

uint8_t Screen::get_button() {
/*

    static int test_index = 0;
    uint8_t mock_buttons[] = {0x2B, 0x2D, 0x31, 0x33, 0x76}; // +, -, 1, 3, Spec
    Sleep(10);
    uint8_t cmd = mock_buttons[test_index];
    test_index = (test_index + 1) % 5;
    return cmd;
*/
 //Ƕ��ʽ��

/*    while (uart_is_readable(DISP_UART_PORT)) {
        uint8_t byte = uart_getc(DISP_UART_PORT);

        // ״̬������ [CMD, 0xFF, 0xFF, 0xFF]
        switch (rx_state) {
            case 0:
                if (byte == 0x2B || byte == 0x2D || byte == 0x31 || byte == 0x33 || byte == 0x76) {
                    last_cmd = byte;
                    rx_state = 1;
                }
                break;
            case 1:
                rx_state = (byte == 0xFF) ? 2 : 0;
                break;
            case 2:
                rx_state = (byte == 0xFF) ? 3 : 0;
                break;
            case 3:
                if (byte == 0xFF) {
                    rx_state = 0;
                    return last_cmd;
                } else {
                    rx_state = 0;
                }
                break;
        }
    }
    return 0;
*/
}
