#pragma once
#include <stdint.h>

class Screen final {
public:
    Screen() = default;
    void send_wave_plot(const float data[], uint16_t length);
    void send_spectrum_plot(
        const float data[],
        uint16_t length
    );
    void send_text(const char *text);
    void send_tune_text(const char *text);
    void send_graph_with_step(const float data[], uint16_t length, uint16_t step);

private:
    float stretch_ = 2.0f;
    float stretch1_ = 2.0f;
    uint16_t offset_ = 128;
    uint16_t offset1_ = 110;
};

/*
“+”按钮按下串口：0x2B, 0xFF, 0xFF, 0xFF
"-":0x2D, 0xFF, 0xFF, 0xFF
"1周期"：0x31, 0xFF, 0xFF, 0xFF
“3周期”：0x33, 0xFF, 0xFF, 0xFF
“电压频谱显示”：0x76, 0xFF, 0xFF, 0xFF
*/
