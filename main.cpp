#include <arm_math.h>

#include <math.h>

#include "definition.hpp"
#include "initialize.hpp"
#include "signal.hpp"
#include "utility.hpp"

#include "read.pio.h"

// Centered median filter: radius 6 gives a 13-sample window.
constexpr uint32_t REPEAT_TIME = 3;
static_assert(REPEAT_TIME > 0);

const auto uart_port = uart0;

PIO const adc_pio = pio0;
constexpr uint adc_sm = 0;

static uint16_t RAW_BUFFER[sig::BUFFER_SIZE] = {};
constexpr size_t ARENA_SAMPLE_COUNT = sizeof(RAW_BUFFER) / sizeof(RAW_BUFFER[0]);

int main() {
    stdio_init_all();
    uart_init(uart_port, 150'0000);

    gpio_init(UART_TX_PIN);
    gpio_init(UART_RX_PIN);

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    if (pio_set_gpio_base(adc_pio, ADC_PIO_GPIO_BASE) != PICO_OK) {
        panic("cannot select ADC PIO GPIO window");
    }
    auto adc_offset = pio_add_program(adc_pio, &read_program);

    // uart_puts(uart_port, "p2");

    pio_init(adc_pio, adc_sm, adc_offset);
    const uint dma_chan = dma_init(adc_pio, adc_sm, RAW_BUFFER, ARENA_SAMPLE_COUNT);

    // uart_puts(uart_port, "p1");

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

    uart_puts(uart_port, "Spectrum Peak Count: ");
    const uint8_t component_count_character = static_cast<uint8_t>(
        '0' + signal_sum.component_count
    );
    uart_write_blocking(
        uart_port,
        &component_count_character,
        1
    );

    uart_puts(uart_port, "\r\nPeak(s) Hz: ");
    for (uint32_t i = 0; i < signal_sum.component_count; ++i) {
        if (i != 0) {
            uart_write_blocking(uart_port, &separator, 1);
        }
        uart_write_float(uart_port, signal_sum.component_frequency[i]);
    }
    uart_puts(uart_port, "\r\nAmplitude(mV): ");
    for (uint32_t i = 0; i < signal_sum.component_count; ++i) {
        if (i != 0) {
            uart_write_blocking(uart_port, &separator, 1);
        }
        uart_write_float(uart_port, signal_sum.component_amplitude[i]);
    }
    uart_puts(uart_port, "\r\nVpp(mV): ");
    uart_write_float(uart_port, signal_sum.v_pp * repeat_scale);
    uart_puts(uart_port, "\r\nVrms(mV): ");
    uart_write_float(uart_port, signal_sum.v_rms * repeat_scale);
    uart_puts(uart_port, "\r\n");
    uart_tx_wait_blocking(uart_port);
    return 0;
}
