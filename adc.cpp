#include <cmath>
#include <cstdint>

#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/pio.h>
#include <hardware/spi.h>
#include <hardware/structs/io_bank0.h>
#include <hardware/sync.h>
#include <hardware/uart.h>
#include <pico/platform.h>
#include <pico/stdlib.h>

#include "definition.h"
#include "read.pio.h"
#include "transmit.pio.h"

auto UART_PORT = uart0;

PIO const ADC_PIO = pio1;
PIO const DAC_PIO = pio0;
constexpr uint ADC_SM = 0;
constexpr uint DAC_SM = 0;

bool dac_started = false;
volatile uint32_t processed_samples = 0;
volatile uint32_t adc_overrange_samples = 0;
volatile uint32_t dac_deadline_misses = 0;

// Bilinear transform at Fs = 300 kHz:
//
//                  5
// H(s) = ------------------------  ->
//        1e-8*s^2 + 3e-4*s + 1
//
//        5 + 10*z^-1 + 5*z^-2
// H(z) = ---------------------------
//        3781 - 7198*z^-1 + 3421*z^-2
float filter_state_1 = 0.0f;
float filter_state_2 = 0.0f;

static __force_inline uint32_t reverse_bits(uint32_t value) {
    value = ((value >> 1) & 0x55555555u) |
            ((value & 0x55555555u) << 1);
    value = ((value >> 2) & 0x33333333u) |
            ((value & 0x33333333u) << 2);
    value = ((value >> 4) & 0x0f0f0f0fu) |
            ((value & 0x0f0f0f0fu) << 4);
    value = ((value >> 8) & 0x00ff00ffu) |
            ((value & 0x00ff00ffu) << 8);
    return (value >> 16) | (value << 16);
}

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

// reference.pdf:
// AD9744 DB0..DB13 -> GPIO14,15,12,13,10,11,8,9,6,7,4,5,2,3.
// transmit.pio writes raw bit 0 to GPIO2.
constexpr uint16_t pack_dac_code_constexpr(uint16_t code) {
    return static_cast<uint16_t>(
        ((code & 0x0003u) << 12) |
        ((code & 0x000cu) << 8) |
        ((code & 0x0030u) << 4) |
        ((code & 0x00c0u) << 0) |
        ((code & 0x0300u) >> 4) |
        ((code & 0x0c00u) >> 8) |
        ((code & 0x3000u) >> 12)
    );
}

// Cross-check the physical bit permutation against the known-good SINUSOIDS
// LUT in wavegen.cpp. Do not derive these checks from compute_lut.py.
static_assert(pack_dac_code_constexpr(0x0323u) == 0x3230u);
static_assert(pack_dac_code_constexpr(0x1fffu) == 0x3ffdu);
static_assert(pack_dac_code_constexpr(0x3cddu) == 0x1dcfu);
static_assert(pack_dac_code_constexpr(0x2001u) == 0x1002u);

uint encode_uart_frame(int32_t data, uint8_t *frame) {
    uint8_t index = 0;
    frame[index++] = '[';

    uint32_t magnitude;
    if (data < 0) {
        frame[index++] = '-';
        magnitude = static_cast<uint32_t>(-data);
    } else {
        magnitude = static_cast<uint32_t>(data);
    }

    uint32_t divisor = 10'000;
    bool leading_digit_seen = false;
    while (divisor != 0) {
        const uint8_t digit =
            static_cast<uint8_t>(magnitude / divisor);
        if (digit != 0 || leading_digit_seen || divisor == 1) {
            frame[index++] = static_cast<uint8_t>('0' + digit);
            leading_digit_seen = true;
        }
        magnitude %= divisor;
        divisor /= 10;
    }

    frame[index++] = ']';
    frame[index++] = ',';
    return index;
}

static __force_inline uint16_t pack_dac_code(uint16_t code) {
    const uint32_t reversed = reverse_bits(code) >> 18;
    return static_cast<uint16_t>(
        ((reversed & 0x1555u) << 1) |
        ((reversed & 0x2aaau) >> 1)
    );
}

static __force_inline float run_transfer_function(float input) {
    constexpr float b0 = 5.0f / 3781.0f;
    constexpr float b1 = 10.0f / 3781.0f;
    constexpr float b2 = 5.0f / 3781.0f;
    constexpr float a1 = -7198.0f / 3781.0f;
    constexpr float a2 = 3421.0f / 3781.0f;

    const float output = b0 * input + filter_state_1;
    filter_state_1 = b1 * input - a1 * output + filter_state_2;
    filter_state_2 = b2 * input - a2 * output;
    return output;
}

static __force_inline uint16_t normalized_to_dac_code(float value) {
    // The DAC module header accepts 14-bit two's-complement samples, as
    // confirmed by the known-good SINUSOIDS LUT. Code 0x0000 is bipolar zero;
    // 0x1fff and 0x2001 are the symmetric positive/negative full-scale codes.
    const long signed_code = std::lround(value * 8191.0f);
    if (signed_code <= -8191l) {
        return 0x2001u;
    }
    if (signed_code >= 8191l) {
        return 0x1fffu;
    }
    return static_cast<uint16_t>(signed_code) & 0x3fffu;
}

void __isr __not_in_flash_func(adc_pio_irq_handler)() {
    while (!pio_sm_is_rx_fifo_empty(ADC_PIO, ADC_SM)) {
        // const uint16_t raw =
        //     static_cast<uint16_t>(pio_sm_get(ADC_PIO, ADC_SM) & 0x3fffu);

        const uint16_t raw = 16u;
        if (raw & (1u << 13)) {
            ++adc_overrange_samples;
        }

        uint8_t uart_frame[9] = {};

        const int32_t signed_code =
            static_cast<int32_t>(unpack_adc_code(raw)) - 2048;
        const float input = static_cast<float>(signed_code) / 2048.0f;
        // const float output = run_transfer_function(input);
        const float output = input;

        const uint frame_size = encode_uart_frame(output, uart_frame);
        uart_write_blocking(UART_PORT, uart_frame, frame_size);
        const uint16_t packed =
            pack_dac_code(normalized_to_dac_code(output));

        if (pio_sm_get_tx_fifo_level(DAC_PIO, DAC_SM) != 0) {
            ++dac_deadline_misses;
        } else {
            pio_sm_put(DAC_PIO, DAC_SM, packed);
        }

        if (!dac_started) {
            pio_sm_set_enabled(DAC_PIO, DAC_SM, true);
            gpio_put(DAC_PD_PIN, 0);
            dac_started = true;
        }
        ++processed_samples;
    }
}

float sample_clock_divider() {
    static_assert(DAC_TARGET_SAMPLE_RATE == ADC_TARGET_SAMPLE_RATE);
    constexpr float PIO_CYCLES_PER_SAMPLE = 2.0f;
    return static_cast<float>(clock_get_hz(clk_sys)) /
           (ADC_TARGET_SAMPLE_RATE * PIO_CYCLES_PER_SAMPLE);
}

void init_adc_pio(uint offset) {
    for (uint pin : ADC_DATA_PINS) {
        pio_gpio_init(ADC_PIO, pin);
        gpio_disable_pulls(pin);
    }
    pio_gpio_init(ADC_PIO, ADC_OTR_PIN);
    pio_gpio_init(ADC_PIO, ADC_CLK_PIN);
    gpio_disable_pulls(ADC_OTR_PIN);
    gpio_disable_pulls(ADC_CLK_PIN);

    pio_sm_config cfg = read_program_get_default_config(offset);
    sm_config_set_in_pins(&cfg, ADC_CAPTURE_BASE);
    sm_config_set_in_pin_count(&cfg, ADC_CAPTURE_COUNT);
    sm_config_set_sideset_pins(&cfg, ADC_CLK_PIN);
    sm_config_set_in_shift(&cfg, false, true, ADC_CAPTURE_COUNT);
    sm_config_set_clkdiv(&cfg, sample_clock_divider());

    pio_sm_init(ADC_PIO, ADC_SM, offset, &cfg);
    pio_sm_set_consecutive_pindirs(
        ADC_PIO, ADC_SM, ADC_CAPTURE_BASE, ADC_CAPTURE_COUNT, false
    );
    pio_sm_set_consecutive_pindirs(
        ADC_PIO, ADC_SM, ADC_CLK_PIN, 1, true
    );
    pio_sm_set_pins_with_mask64(
        ADC_PIO, ADC_SM, 0, 1ull << ADC_CLK_PIN
    );
}

void init_dac_pio(uint offset) {
    for (uint pin : DAC_DATA_PINS) {
        pio_gpio_init(DAC_PIO, pin);
    }
    pio_gpio_init(DAC_PIO, DAC_CLK_PIN);

    pio_sm_config cfg = transmit_program_get_default_config(offset);
    sm_config_set_out_pins(&cfg, DAC_DATA_BASE, DAC_DATA_COUNT);
    sm_config_set_sideset_pins(&cfg, DAC_CLK_PIN);
    sm_config_set_out_shift(&cfg, true, true, DAC_DATA_COUNT);
    sm_config_set_clkdiv(&cfg, sample_clock_divider());

    pio_sm_init(DAC_PIO, DAC_SM, offset, &cfg);
    pio_sm_set_consecutive_pindirs(
        DAC_PIO, DAC_SM, DAC_DATA_BASE, DAC_DATA_COUNT, true
    );
    pio_sm_set_consecutive_pindirs(
        DAC_PIO, DAC_SM, DAC_CLK_PIN, 1, true
    );

    const uint64_t data_mask =
        ((1ull << DAC_DATA_COUNT) - 1ull) << DAC_DATA_BASE;
    pio_sm_set_pins_with_mask64(
        DAC_PIO, DAC_SM, 0, data_mask | (1ull << DAC_CLK_PIN)
    );
}

int main() {
    stdio_init_all();
    uart_init(UART_PORT, 1500000);

    gpio_init(DAC_PD_PIN);
    gpio_set_dir(DAC_PD_PIN, GPIO_OUT);
    gpio_put(DAC_PD_PIN, 1);

    gpio_init(UART_RX_PIN);
    gpio_init(UART_TX_PIN);
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_PORT, UART_RX_PIN));
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_PORT, UART_TX_PIN));

    uart_set_format(UART_PORT, 8, 1, UART_PARITY_NONE);

    // RP2350B PIO1 addresses GPIO16..GPIO47; PIO0 retains GPIO0..GPIO31.
    if (pio_set_gpio_base(ADC_PIO, ADC_PIO_GPIO_BASE) != PICO_OK) {
        panic("cannot select PIO1 GPIO16..47 window");
    }

    const uint dac_offset = pio_add_program(DAC_PIO, &transmit_program);
    const uint adc_offset = pio_add_program(ADC_PIO, &read_program);
    init_dac_pio(dac_offset);
    init_adc_pio(adc_offset);

    pio_sm_set_enabled(ADC_PIO, ADC_SM, true);
    uart_puts(UART_PORT, "P1");

    // Cover the AD9226 seven-clock pipeline and the optional clock
    // stabilizer's >100-clock startup settling requirement.
    for (uint i = 0; i < 128; ++i) {
        (void)pio_sm_get_blocking(ADC_PIO, ADC_SM);
    }
    uart_puts(UART_PORT, "P2");
    pio_sm_clear_fifos(ADC_PIO, ADC_SM);

    irq_set_exclusive_handler(PIO1_IRQ_0, adc_pio_irq_handler);
    irq_set_priority(PIO1_IRQ_0, PICO_HIGHEST_IRQ_PRIORITY);
    pio_set_irq0_source_enabled(
        ADC_PIO, pis_sm0_rx_fifo_not_empty, true
    );
    irq_set_enabled(PIO1_IRQ_0, true);

    while (true) {
        __wfi();
    }
}
