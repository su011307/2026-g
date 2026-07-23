#pragma once
#include <pico/types.h>

// lcd的引脚
constexpr uint SPI_MISO_PIN = 16;
constexpr uint SPI_CS_PIN = 17;
constexpr uint SPI_SCK_PIN = 18;
constexpr uint SPI_MOSI_PIN = 19;
constexpr uint LCD_SCL_PIN = SPI_SCK_PIN;
constexpr uint LCD_SDA_PIN = SPI_MOSI_PIN;
constexpr uint LCD_RST_PIN = 20;
constexpr uint LCD_DC_PIN = 21;
constexpr uint LCD_CS_PIN = 22;
constexpr uint LCD_BL_PIN = 23;

constexpr uint UART_RX_PIN = 45;
constexpr uint UART_TX_PIN = 44;

// ad9744的控制引脚
constexpr uint DAC_CLK_PIN    = 0;
constexpr uint DAC_PD_PIN     = 1; // sleep pin，高电平有效
constexpr uint DAC_DATA_BASE  = 2;
constexpr uint DAC_DATA_COUNT = 14;

constexpr uint DAC_DATA_PINS[] = {
    14, 15, 12, 13,
    10, 11, 8, 9,
    6, 7, 4, 5,
    2, 3
}; // ad9744的数据引脚，最高位为符号位

// ad9226的控制引脚
constexpr uint ADC_CLK_PIN = 28;
constexpr uint ADC_OTR_PIN = 39;
constexpr uint ADC_DATA_PINS[] = {
    38, 37, 36, 35,
    34, 33, 26, 27,
    32, 31, 30, 29
};
