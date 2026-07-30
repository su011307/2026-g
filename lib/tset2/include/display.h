#ifndef DISPLAY_FOR_G_H
#define DISPLAY_FOR_G_H
#include <stdint.h>
#include <stdio.h>

#include <pico/time.h>

// 嵌入式：
// #include <hardware/uart.h>
// #include "definition.h"
/*// 你放definition.h里面
#define DISP_UART_PORT uart1
constexpr uint DISP_UART_TX_PIN = 8;
constexpr uint DISP_UART_RX_PIN = 9;
constexpr uint32_t DISP_UART_BAUDRATE = 115200;
*/
class Screen
{
public:
    Screen();
    ~Screen();
    void send_graph(float data[], uint16_t length);

    void send_text(char t[]);
    uint8_t get_button();

protected:
private:
    uint8_t offset = 128;
    float strentch = 3;

    uint8_t rx_state = 0;
    uint8_t last_cmd = 0;
};

/*
“+”按钮按下串口：0x2B, 0xFF, 0xFF, 0xFF
"-":0x2D, 0xFF, 0xFF, 0xFF
"1周期"：0x31, 0xFF, 0xFF, 0xFF
“3周期”：0x33, 0xFF, 0xFF, 0xFF
“电压频谱显示”：0x76, 0xFF, 0xFF, 0xFF
*/
#endif // DISPLAY_FOR_G_H
