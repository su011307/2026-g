#include <hardware/structs/io_bank0.h>
#include <pico/platform/common.h>
#include <pico/time.h>
#include <stdio.h>
#include <stdint.h>
#include <pico/types.h>
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <hardware/spi.h>

#define spi_port spi0
#define spi_clk 18
#define spi_mosi 19
#define spi_miso 16

#define rst 44
#define cs 45
#define up 46

void write_reg(uint8_t addr, uint32_t data, uint8_t bytes) {
    uint8_t cmd = addr & 0x7F;

    gpio_put(cs, 0);
    spi_write_blocking(spi_port, &cmd, 1);

    for (int i = bytes - 1; i >= 0; i--) {
        uint8_t byte = (data >> (8 * i)) & 0xFF;
        spi_write_blocking(spi_port, &byte, 1);
    }

    gpio_put(cs, 1);
}

int main() {
    stdio_init_all();

    spi_init(spi_port, 1000 * 1000);
    spi_set_format(spi_port, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(spi_clk, GPIO_FUNC_SPI);
    gpio_set_function(spi_mosi, GPIO_FUNC_SPI);
    gpio_set_function(spi_miso, GPIO_FUNC_SPI);

    gpio_init(rst); gpio_init(cs); gpio_init(up);
    gpio_set_dir(rst, GPIO_OUT); gpio_set_dir(cs, GPIO_OUT); gpio_set_dir(up, GPIO_OUT);

    sleep_ms(10);
    gpio_put(rst, 1);
    sleep_ms(10);
    gpio_put(rst, 0);
    sleep_ms(10);

    uint32_t ftw = (uint32_t)(1e6 * (1ull << 32) / (25e6 * 20));  // divided by 20 due to PLL
    write_reg(0x01, (0b11010000 << 16), 3);
    write_reg(0x04, ftw, 4);

    gpio_put(up, 0);
    sleep_ms(10);
    gpio_put(up, 1);
    sleep_ms(10);
    gpio_put(up, 0);

    while (1) {
        tight_loop_contents();
    }
}
