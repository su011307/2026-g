#pragma once

#include <cstdint>
#include <hardware/uart.h>

class Screen final {
public:
    explicit Screen(uart_inst_t *uart_port, uart_inst_t *debug_port);
    void send_graph(const float data[], uint16_t length);
    void send_text(const char *text, uint8_t size);

private:
    float stretch_ = 3.0f;
    uart_inst_t *const uart_port_;
    uart_inst_t *const debug_port_;
    uint8_t offset_ = 128;
};

/*
“+”按钮按下串口：0x2B, 0xFF, 0xFF, 0xFF
"-":0x2D, 0xFF, 0xFF, 0xFF
"1周期"：0x31, 0xFF, 0xFF, 0xFF
“3周期”：0x33, 0xFF, 0xFF, 0xFF
“电压频谱显示”：0x76, 0xFF, 0xFF, 0xFF
*/
