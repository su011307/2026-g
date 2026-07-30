#include <arm_math.h>
#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <hardware/structs/io_bank0.h>
#include <hardware/uart.h>
#include <pico/stdio.h>

#include "definition.hpp"
#include "display.hpp"
#include "initialize.hpp"
#include "signal.hpp"
#include "utility.hpp"

#include "read.pio.h"

// Centered median filter: radius 6 gives a 13-sample window.
constexpr uint32_t REPEAT_TIME = 3;
static_assert(REPEAT_TIME > 0);

const auto oled_spi = spi0;
const auto serial_uart = uart0;
const auto screen_uart = uart1;

PIO const adc_pio = pio0;
constexpr uint adc_sm = 0;

static uint16_t RAW_BUFFER[sig::BUFFER_SIZE] = {};
constexpr size_t ARENA_SAMPLE_COUNT = sizeof(RAW_BUFFER) / sizeof(RAW_BUFFER[0]);

void init_peripheral(
    spi_inst_t *const oled_spi_port,
    uart_inst_t *const screen_uart_port,
    uart_inst_t *const serial_uart_port
) {
    stdio_init_all();
    spi_init(oled_spi_port, 150'0000);
    uart_init(serial_uart_port, 150'0000);
    uart_init(screen_uart_port, disp::DISP_UART_BAUDRATE);

    gpio_init(dbg::SERIAL_UART_TX_PIN);
    gpio_init(dbg::SERIAL_UART_RX_PIN);
    gpio_init(disp::DISP_UART_TX_PIN);
    gpio_init(disp::DISP_UART_RX_PIN);

    gpio_set_function(dbg::SERIAL_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(dbg::SERIAL_UART_RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(disp::DISP_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(disp::DISP_UART_RX_PIN, GPIO_FUNC_UART);
}

int main() {
    init_peripheral(oled_spi, screen_uart, serial_uart);

    if (pio_set_gpio_base(adc_pio, adc::PIO_GPIO_BASE) != PICO_OK) {
        panic("cannot select ADC PIO GPIO window");
    }
    auto adc_offset = pio_add_program(adc_pio, &read_program);

    pio_init(adc_pio, adc_sm, adc_offset);
    const uint dma_chan = dma_init(adc_pio, adc_sm, RAW_BUFFER, ARENA_SAMPLE_COUNT);

    sleep_ms(1);

    SignalDescription signal_sum = {};
    reset_signal_analysis();

    for (uint32_t repeat = 0; repeat < REPEAT_TIME; ++repeat) {
        pio_sm_clear_fifos(adc_pio, adc_sm);
        dma_channel_set_write_addr(dma_chan, RAW_BUFFER, false);
        dma_channel_set_trans_count(
            dma_chan,
            ARENA_SAMPLE_COUNT,
            false
        );

        dma_channel_start(dma_chan);
        pio_sm_set_enabled(adc_pio, adc_sm, true);
        dma_channel_wait_for_finish_blocking(dma_chan);
        pio_sm_set_enabled(adc_pio, adc_sm, false);

        unpack_adc_buffer(RAW_BUFFER);
        const SignalDescription current = analyze_signal(RAW_BUFFER);
        signal_sum.v_pp += current.v_pp;
        signal_sum.v_rms += current.v_rms;
    }

    constexpr uint8_t separator = ',';
    const float repeat_scale = 1.0f / static_cast<float>(REPEAT_TIME);
    analyze_accumulated_spectrum(
        repeat_scale,
        signal_sum
    );

    uart_puts(serial_uart, "Spectrum Peak Count: ");
    const uint8_t component_count_character = static_cast<uint8_t>(
        '0' + signal_sum.component_count
    );
    uart_write_blocking(
        serial_uart,
        &component_count_character,
        1
    );

    uart_puts(serial_uart, "\r\nPeak(s) Hz: ");
    for (uint32_t i = 0; i < signal_sum.component_count; ++i) {
        if (i != 0) {
            uart_write_blocking(serial_uart, &separator, 1);
        }
        uart_write_float(serial_uart, signal_sum.component_frequency[i]);
    }
    uart_puts(serial_uart, "\r\nAmplitude(mV): ");
    for (uint32_t i = 0; i < signal_sum.component_count; ++i) {
        if (i != 0) {
            uart_write_blocking(serial_uart, &separator, 1);
        }
        uart_write_float(serial_uart, signal_sum.component_amplitude[i]);
    }
    uart_puts(serial_uart, "\r\nVpp(mV): ");
    uart_write_float(serial_uart, signal_sum.v_pp * repeat_scale);
    uart_puts(serial_uart, "\r\nVrms(mV): ");
    uart_write_float(serial_uart, signal_sum.v_rms * repeat_scale);
    uart_puts(serial_uart, "\r\n");
    uart_tx_wait_blocking(serial_uart);
    return 0;
}
