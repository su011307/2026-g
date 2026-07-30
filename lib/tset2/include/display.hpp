#pragma once
#include <cstdint>

class Screen final {
public:
    Screen() = default;
    void send_graph(const float data[], uint16_t length);
    void send_text(const char *text);
    void send_graph_with_step(const float data[], uint16_t length, uint16_t step);

private:
    float stretch_ = 2.0f;
    uint16_t offset_ = 250;
};

/*
“+”按钮按下串口：0x2B, 0xFF, 0xFF, 0xFF
"-":0x2D, 0xFF, 0xFF, 0xFF
"1周期"：0x31, 0xFF, 0xFF, 0xFF
“3周期”：0x33, 0xFF, 0xFF, 0xFF
“电压频谱显示”：0x76, 0xFF, 0xFF, 0xFF
*/
