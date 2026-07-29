#include <cmath>
#include <cstddef>
#include <cstdint>

#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/uart.h>
#include <pico/stdlib.h>

#include "definition.h"
#include "read.pio.h"

const auto uart_port = uart0;

constexpr size_t FILTERED_RESULT_COUNT = 1024;

PIO const adc_pio = pio0;
constexpr uint adc_sm = 0;
constexpr float VOLT_CONVERT = 0.002441406;

typedef struct {
    float vpp;
    float vrms;
} VoltageResultPair;

// 提取出原始 12b ADC 数据。其中 OTR、CLK 被 discard
static __force_inline uint16_t unpack_adc_code(uint16_t raw) {
    const uint16_t d11_d6 =
        static_cast<uint16_t>((raw & 0x1f80u) >> 1);
    const uint16_t d5_d4 =
        static_cast<uint16_t>(
            ((raw & 0x0001u) << 5) |
            ((raw & 0x0002u) << 3)
        );
    const uint16_t d3_d0 =
        static_cast<uint16_t>((raw & 0x0078u) >> 3);
    return static_cast<uint16_t>(d11_d6 | d5_d4 | d3_d0);
}
float compute_adc_voltage(int32_t raw) {
    return (5.0f - VOLT_CONVERT * raw) * 1000; // metric: mV
}

float sample_clock_divider() {
    constexpr float pio_cycle_per_sample = 2.0f;
    return static_cast<float>(clock_get_hz(clk_sys)) / 
        (ADC_TARGET_SAMPLE_RATE * pio_cycle_per_sample);
}

float median(float a, float b, float c) {
    if (a > b) {
        float t=a;
        a=b;
        b=t;
    }

    if (b > c) {
        float t=b;
        b=c;
        c=t;
    }

    if (a > b) {
        float t=a;
        a=b;
        b=t;
    }

    return b;
}

void pio_init(
    PIO pio,
    uint sm,
    uint offset
){
    for (auto i : ADC_DATA_PINS) {
        pio_gpio_init(pio, i);
        gpio_disable_pulls(i);
    }

    pio_gpio_init(pio, ADC_OTR_PIN);
    pio_gpio_init(pio, ADC_CLK_PIN);
    gpio_disable_pulls(ADC_OTR_PIN);
    gpio_disable_pulls(ADC_CLK_PIN);

    auto cfg = read_program_get_default_config(offset);
    sm_config_set_in_pins(&cfg, ADC_CAPTURE_BASE);
    sm_config_set_in_pin_count(&cfg, ADC_CAPTURE_COUNT);
    sm_config_set_sideset_pins(&cfg, ADC_CLK_PIN);
    sm_config_set_in_shift(&cfg, false, true, ADC_CAPTURE_COUNT);
    sm_config_set_clkdiv(&cfg, sample_clock_divider());

    if (pio_sm_init(pio, sm, offset, &cfg) != PICO_OK) {
        panic("cannot initialize ADC PIO state machine");
    }
    pio_sm_set_consecutive_pindirs(
        pio, sm, ADC_CAPTURE_BASE, ADC_CAPTURE_COUNT, false
    );
    pio_sm_set_consecutive_pindirs(
        pio, sm, ADC_CLK_PIN, 1, true
    );
    pio_sm_set_pins_with_mask64(
        pio, sm, 0, 1ull << ADC_CLK_PIN
    );
}

uint dma_init(
    PIO pio,
    uint sm,
    uint16_t *arena_ptr,
    size_t sample_count
){
    auto chan = dma_claim_unused_channel(true);
    auto cfg = dma_channel_get_default_config(chan);

    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, pio_get_dreq(pio, sm, false));
    channel_config_set_high_priority(&cfg, true);

    dma_channel_configure(chan, &cfg,
        arena_ptr,
        &pio -> rxf[sm],
        sample_count,
        false
    );

    return chan;
}

uint8_t encode_float_to_string(float data, uint8_t *str) {
    int index = 0;

    if (data < 0.0f) {
        str[index++] = '-';
        data = -data;
    }

    uint32_t integer = static_cast<uint32_t>(data);
    uint32_t fraction = static_cast<uint32_t>((data - integer) * 1000.0f);

    char tmp[12];
    int t = 0;

    if (integer == 0) {
        tmp[t++] = '0';
    } else {
        while (integer > 0) {
            tmp[t++] = '0' + integer % 10;
            integer /= 10;
        }
        for (int i = 0; i < t / 2; i++) {
            char c = tmp[i];
            tmp[i] = tmp[t - 1 - i];
            tmp[t - 1 - i] = c;
        }
    }

    for (int i = 0; i < t; i++) {
        str[index++] = tmp[i];
    }

    str[index++] = '.';
    str[index++] = '0' + (fraction / 100);
    str[index++] = '0' + ((fraction / 10) % 10);
    str[index++] = '0' + (fraction % 10);

    str[index] = '\0';
    return index;
}

VoltageResultPair compute_vpp_and_vrms(
    const uint16_t *arena,
    size_t sample_count
) {
    float max = -32768.0f, min = 32768.0f;
    float square_sum = 0.0f;

    for (size_t i = 0; i < sample_count; i++) {
        auto adc_raw = static_cast<int32_t>(unpack_adc_code(arena[i]));
        float volt = compute_adc_voltage(adc_raw);
        max = max > volt ? max : volt;
        min = min < volt ? min : volt;
        square_sum += volt * volt;
    }

    auto vpp = max - min;
    auto vrms = std::sqrt(
        square_sum / static_cast<float>(sample_count)
    );

    return VoltageResultPair{.vpp = vpp, .vrms = vrms};
}

static uint16_t ARENA[32768] = {};
constexpr size_t ARENA_SAMPLE_COUNT = sizeof(ARENA) / sizeof(ARENA[0]);

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
    const uint dma_chan =
        dma_init(adc_pio, adc_sm, ARENA, ARENA_SAMPLE_COUNT);

    // uart_puts(uart_port, "p1");

    dma_channel_start(dma_chan);
    pio_sm_set_enabled(adc_pio, adc_sm, true);

    VoltageResultPair history[2] = {};
    size_t history_count = 0;
    size_t filtered_count = 0;
    float vpp_sum = 0.0f;
    float vrms_sum = 0.0f;

    while (filtered_count < FILTERED_RESULT_COUNT) {
        dma_channel_wait_for_finish_blocking(dma_chan);
        // uart_puts(uart_port, "p0");

        const VoltageResultPair current =
            compute_vpp_and_vrms(ARENA, ARENA_SAMPLE_COUNT);

        // Re-arm on every path. The old early continues left DMA completed,
        // causing the same ARENA contents to be processed repeatedly.
        dma_channel_set_write_addr(dma_chan, ARENA, false);
        dma_channel_set_trans_count(
            dma_chan,
            ARENA_SAMPLE_COUNT,
            true
        );

        if (history_count < 2) {
            history[history_count++] = current;
            continue;
        }

        const float filtered_vpp =
            median(history[0].vpp, history[1].vpp, current.vpp);
        const float filtered_vrms =
            median(history[0].vrms, history[1].vrms, current.vrms);

        vpp_sum += filtered_vpp;
        vrms_sum += filtered_vrms;
        ++filtered_count;

        // Slide the window so a startup outlier is not retained forever.
        history[0] = history[1];
        history[1] = current;
    }

    const float mean_vpp =
        vpp_sum / static_cast<float>(filtered_count);
    const float mean_vrms =
        vrms_sum / static_cast<float>(filtered_count);

    uint8_t uart_buf[12] = {};

    uart_puts(uart_port, "Vpp: ");
    auto size = encode_float_to_string(mean_vpp, uart_buf);
    uart_write_blocking(uart_port, uart_buf, size);
    uart_puts(uart_port, ", Vrms: ");
    size = encode_float_to_string(mean_vrms, uart_buf);
    uart_write_blocking(uart_port, uart_buf, size);
    uart_puts(uart_port, "\r\n");
}
