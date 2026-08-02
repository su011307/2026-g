#include "driver_st7789_interface_rp2350.h"

#include <stdarg.h>
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/time.h"

#define ST7789_RP2350_SPI_PORT    spi0

static void a_st7789_gpio_output_init(uint pin, bool value)
{
    gpio_init(pin);
    gpio_put(pin, value);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_disable_pulls(pin);
    gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
}

uint8_t st7789_interface_spi_init(void)
{
    a_st7789_gpio_output_init(ST7789_RP2350_PIN_CS, true);
    spi_init(ST7789_RP2350_SPI_PORT, ST7789_RP2350_SPI_BAUD_HZ);
    spi_set_format(ST7789_RP2350_SPI_PORT, 8U, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(ST7789_RP2350_PIN_SCL, GPIO_FUNC_SPI);
    gpio_set_function(ST7789_RP2350_PIN_SDA, GPIO_FUNC_SPI);
    gpio_disable_pulls(ST7789_RP2350_PIN_SCL);
    gpio_disable_pulls(ST7789_RP2350_PIN_SDA);
    gpio_set_slew_rate(ST7789_RP2350_PIN_SCL, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(ST7789_RP2350_PIN_SDA, GPIO_SLEW_RATE_FAST);

    return 0;
}

uint8_t st7789_interface_spi_deinit(void)
{
    spi_deinit(ST7789_RP2350_SPI_PORT);
    gpio_deinit(ST7789_RP2350_PIN_SCL);
    gpio_deinit(ST7789_RP2350_PIN_SDA);
    a_st7789_gpio_output_init(ST7789_RP2350_PIN_CS, true);

    return 0;
}

uint8_t st7789_interface_spi_write_cmd(uint8_t *buf, uint16_t len)
{
    int written;

    if ((buf == NULL) && (len != 0U))
    {
        return 1;
    }
    if (len == 0U)
    {
        return 0;
    }

    gpio_put(ST7789_RP2350_PIN_CS, false);
    busy_wait_us_32(1U);
    written = spi_write_blocking(ST7789_RP2350_SPI_PORT, buf, len);
    busy_wait_us_32(1U);
    gpio_put(ST7789_RP2350_PIN_CS, true);
    busy_wait_us_32(1U);

    return written == len ? 0U : 1U;
}

void st7789_interface_delay_ms(uint32_t ms)
{
    sleep_ms(ms);
}

void st7789_interface_debug_print(const char *const fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    (void)vprintf(fmt, args);
    va_end(args);
}

uint8_t st7789_interface_cmd_data_gpio_init(void)
{
    a_st7789_gpio_output_init(ST7789_RP2350_PIN_CS, true);
    a_st7789_gpio_output_init(ST7789_RP2350_PIN_DC, false);

    return 0;
}

uint8_t st7789_interface_cmd_data_gpio_deinit(void)
{
    gpio_deinit(ST7789_RP2350_PIN_DC);
    a_st7789_gpio_output_init(ST7789_RP2350_PIN_CS, true);

    return 0;
}

uint8_t st7789_interface_cmd_data_gpio_write(uint8_t value)
{
    gpio_put(ST7789_RP2350_PIN_DC, value != 0U);
    busy_wait_us_32(1U);

    return 0;
}

uint8_t st7789_interface_reset_gpio_init(void)
{
    a_st7789_gpio_output_init(ST7789_RP2350_PIN_RESET, false);

    return 0;
}

uint8_t st7789_interface_reset_gpio_deinit(void)
{
    gpio_put(ST7789_RP2350_PIN_RESET, false);
    gpio_deinit(ST7789_RP2350_PIN_RESET);

    return 0;
}

uint8_t st7789_interface_reset_gpio_write(uint8_t value)
{
    gpio_put(ST7789_RP2350_PIN_RESET, value != 0U);

    return 0;
}
