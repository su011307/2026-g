#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/uart.h>
#include <pico/stdlib.h>
#include <pico/time.h>

#include <arm_math.h>

#include "definition.h"
#include "read.pio.h"

constexpr uint32_t BUFFER_SIZE = 65536;
// Mathematica MedianFilter[data, 7] uses a radius of 7, i.e. 15 samples.
constexpr uint32_t MEDIAN_RADIUS = 7;
constexpr uint32_t MEDIAN_WINDOW = 2 * MEDIAN_RADIUS + 1;
constexpr uint32_t REPEAT_TIME = 16;
constexpr uint32_t FFT_SIZE = 4096;
constexpr uint32_t FIR_ORDER = 64;
constexpr uint32_t NUM_TAPS = FIR_ORDER + 1;
constexpr uint32_t FIR_BLOCK_SIZE = 256;
constexpr uint32_t DECIMATION_FACTOR = BUFFER_SIZE / FFT_SIZE;
constexpr uint32_t SPECTRUM_BIN_COUNT = FFT_SIZE / 2 + 1;
constexpr uint32_t PEAK_MIN_SEPARATION_BINS = 5;

constexpr float PEAK_MIN_FREQUENCY_HZ = 10'000.0f;
constexpr float PEAK_MAX_FREQUENCY_HZ = 500'000.0f;

static_assert(BUFFER_SIZE % FFT_SIZE == 0);
static_assert((FFT_SIZE & (FFT_SIZE - 1)) == 0);
static_assert(BUFFER_SIZE % FIR_BLOCK_SIZE == 0);
static_assert(FIR_BLOCK_SIZE % DECIMATION_FACTOR == 0);
static_assert((MEDIAN_WINDOW & 1u) == 1u);
static_assert(BUFFER_SIZE >= MEDIAN_WINDOW);
static_assert(REPEAT_TIME > 0);

const auto uart_port = uart0;

PIO const adc_pio = pio0;
constexpr uint adc_sm = 0;

constexpr float VOLT_CONVERT = 0.002441406;
constexpr float ZERO_DRIFT_COMPENSATE = -40;
static float32_t FIR_COEFFICIENTS[NUM_TAPS] = {
    1.45347682e-03f, 1.55048002e-03f, 1.73592519e-03f, 2.01724593e-03f,
    2.40052891e-03f, 2.89036140e-03f, 3.48970226e-03f, 4.19977897e-03f,
    5.02001310e-03f, 5.94797592e-03f, 6.97937564e-03f, 8.10807667e-03f,
    9.32615135e-03f, 1.06239634e-02f, 1.19902825e-02f, 1.34124274e-02f,
    1.48764368e-02f, 1.63672641e-02f, 1.78689936e-02f, 1.93650737e-02f,
    2.08385649e-02f, 2.22723963e-02f, 2.36496278e-02f, 2.49537130e-02f,
    2.61687574e-02f, 2.72797695e-02f, 2.82728980e-02f, 2.91356526e-02f,
    2.98571038e-02f, 3.04280583e-02f, 3.08412071e-02f, 3.10912435e-02f,
    3.11749485e-02f, 3.10912435e-02f, 3.08412071e-02f, 3.04280583e-02f,
    2.98571038e-02f, 2.91356526e-02f, 2.82728980e-02f, 2.72797695e-02f,
    2.61687574e-02f, 2.49537130e-02f, 2.36496278e-02f, 2.22723963e-02f,
    2.08385649e-02f, 1.93650737e-02f, 1.78689936e-02f, 1.63672641e-02f,
    1.48764368e-02f, 1.34124274e-02f, 1.19902825e-02f, 1.06239634e-02f,
    9.32615135e-03f, 8.10807667e-03f, 6.97937564e-03f, 5.94797592e-03f,
    5.02001310e-03f, 4.19977897e-03f, 3.48970226e-03f, 2.89036140e-03f,
    2.40052891e-03f, 2.01724593e-03f, 1.73592519e-03f, 1.55048002e-03f,
    1.45347682e-03f
};

typedef struct {
    float top_freq[3];
    float v_pp;
    float v_rms;
} SignalDescription;

typedef struct {
    uint32_t bin;
    float power;
} SpectrumPeak;

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
    return (5.0f - VOLT_CONVERT * raw) * 1000 + ZERO_DRIFT_COMPENSATE; // metric: mV
}

float sample_clock_divider() {
    constexpr float pio_cycle_per_sample = 2.0f;
    return static_cast<float>(clock_get_hz(clk_sys)) / 
        (ADC_TARGET_SAMPLE_RATE * pio_cycle_per_sample);
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

    return index;
}

void unpack_adc_buffer(uint16_t *buffer) {
    for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
        buffer[i] = unpack_adc_code(buffer[i]);
    }
}

static __force_inline uint32_t reflected_sample_index(int32_t index) {
    if (index < 0) {
        return static_cast<uint32_t>(-index - 1);
    }
    if (index >= static_cast<int32_t>(BUFFER_SIZE)) {
        return static_cast<uint32_t>(
            2 * static_cast<int32_t>(BUFFER_SIZE) - index - 1
        );
    }
    return static_cast<uint32_t>(index);
}

static __force_inline uint16_t median_at(
    const uint16_t *samples,
    uint32_t center
) {
    uint16_t sorted[MEDIAN_WINDOW];
    for (uint32_t i = 0; i < MEDIAN_WINDOW; ++i) {
        const int32_t source_index =
            static_cast<int32_t>(center) +
            static_cast<int32_t>(i) -
            static_cast<int32_t>(MEDIAN_RADIUS);
        sorted[i] = samples[reflected_sample_index(source_index)];
    }

    // The window is small and fixed. Only the middle item is used.
    for (uint32_t i = 1; i < MEDIAN_WINDOW; ++i) {
        const uint16_t value = sorted[i];
        uint32_t position = i;
        while (position > 0 && sorted[position - 1] > value) {
            sorted[position] = sorted[position - 1];
            --position;
        }
        sorted[position] = value;
    }
    return sorted[MEDIAN_WINDOW / 2];
}

static void apply_median_filter(
    const uint16_t *input,
    uint16_t *output
) {
    for (uint32_t i = 0; i < BUFFER_SIZE; ++i) {
        output[i] = median_at(input, i);
    }
}

static void spectrum_components(
    const float *fft_output,
    uint32_t bin,
    float &real,
    float &imag
) {
    if (bin == 0) {
        real = fft_output[0];
        imag = 0.0f;
    } else if (bin == FFT_SIZE / 2) {
        real = fft_output[1];
        imag = 0.0f;
    } else {
        real = fft_output[2 * bin];
        imag = fft_output[2 * bin + 1];
    }
}

static float spectrum_magnitude_mv(
    const float *fft_output,
    uint32_t bin,
    float coherent_gain
) {
    float real;
    float imag;
    spectrum_components(fft_output, bin, real, imag);
    const float one_sided_scale =
        (bin == 0 || bin == FFT_SIZE / 2) ? 1.0f : 2.0f;
    return sqrtf(real * real + imag * imag) *
           one_sided_scale /
           (static_cast<float>(FFT_SIZE) * coherent_gain);
}

static void uart_write_float(float value) {
    uint8_t uart_buf[16] = {};
    const uint8_t size = encode_float_to_string(value, uart_buf);
    uart_write_blocking(uart_port, uart_buf, size);
}

static float FFT_INPUT[FFT_SIZE] = {};
static float FFT_OUTPUT[FFT_SIZE] = {};
static float FIR_INPUT[FIR_BLOCK_SIZE] = {};
static float FIR_OUTPUT[FIR_BLOCK_SIZE / DECIMATION_FACTOR] = {};
static float FIR_STATE[NUM_TAPS + FIR_BLOCK_SIZE - 1] = {};
static float SPECTRUM_SUM[SPECTRUM_BIN_COUNT] = {};
static uint16_t MEDIAN_BUFFER[BUFFER_SIZE] = {};

static float accumulated_spectrum_power(
    const float *spectrum,
    uint32_t bin
) {
    return spectrum[bin] * spectrum[bin];
}

static void find_peak_frequencies(
    const float *spectrum,
    float (&frequencies)[3]
) {
    const float bin_width_hz =
        ADC_TARGET_SAMPLE_RATE / static_cast<float>(BUFFER_SIZE);

    // Include bins whose +/- half-bin interval intersects 10--500 kHz.
    // The 10 kHz tone maps to bin 10 (9.918 kHz), not bin 11.
    uint32_t first_peak_bin = static_cast<uint32_t>(
        ceilf(PEAK_MIN_FREQUENCY_HZ / bin_width_hz - 0.5f)
    );
    uint32_t last_peak_bin = static_cast<uint32_t>(
        floorf(PEAK_MAX_FREQUENCY_HZ / bin_width_hz + 0.5f)
    );
    if (first_peak_bin < 1) {
        first_peak_bin = 1;
    }
    if (last_peak_bin > FFT_SIZE / 2 - 1) {
        last_peak_bin = FFT_SIZE / 2 - 1;
    }

    SpectrumPeak peaks[3] = {};
    for (size_t peak_index = 0; peak_index < 3; ++peak_index) {
        SpectrumPeak best = {};
        for (uint32_t bin = first_peak_bin; bin <= last_peak_bin; ++bin) {
            const float left =
                accumulated_spectrum_power(spectrum, bin - 1);
            const float center =
                accumulated_spectrum_power(spectrum, bin);
            const float right =
                accumulated_spectrum_power(spectrum, bin + 1);
            if (!(center > left && center >= right)) {
                continue;
            }

            bool separated = true;
            for (size_t i = 0; i < peak_index; ++i) {
                const uint32_t distance =
                    bin > peaks[i].bin
                        ? bin - peaks[i].bin
                        : peaks[i].bin - bin;
                if (distance < PEAK_MIN_SEPARATION_BINS) {
                    separated = false;
                    break;
                }
            }
            if (separated && center > best.power) {
                best = SpectrumPeak{.bin = bin, .power = center};
            }
        }
        peaks[peak_index] = best;
    }

    for (size_t i = 0; i < 3; ++i) {
        const uint32_t bin = peaks[i].bin;
        if (peaks[i].power == 0.0f || bin == 0) {
            frequencies[i] = 0.0f;
            continue;
        }

        const float left =
            accumulated_spectrum_power(spectrum, bin - 1);
        const float center =
            accumulated_spectrum_power(spectrum, bin);
        const float right =
            accumulated_spectrum_power(spectrum, bin + 1);
        const float denominator = left - 2.0f * center + right;
        float offset = 0.0f;
        if (denominator != 0.0f) {
            offset = 0.5f * (left - right) / denominator;
            if (offset < -0.5f) {
                offset = -0.5f;
            } else if (offset > 0.5f) {
                offset = 0.5f;
            }
        }
        frequencies[i] =
            (static_cast<float>(bin) + offset) * bin_width_hz;
    }
}

SignalDescription analyze_signal(uint16_t *buffer) {
    float min_voltage = 32768;
    float max_voltage = -32768;
    float voltage_sum = 0.0f;
    float voltage_square_sum = 0.0;

    // Match Mathematica's data = MedianFilter[data, 7]. Since voltage
    // conversion is monotonic, filtering ADC codes gives the same median.
    apply_median_filter(buffer, MEDIAN_BUFFER);

    for (uint32_t i = 0; i < BUFFER_SIZE; ++i) {
        const float voltage = compute_adc_voltage(
            static_cast<int32_t>(MEDIAN_BUFFER[i])
        );
        if (voltage < min_voltage) {
            min_voltage = voltage;
        }
        if (voltage > max_voltage) {
            max_voltage = voltage;
        }
        voltage_sum += voltage;
        voltage_square_sum += voltage * voltage;
    }

    SignalDescription result = {};
    result.v_pp = max_voltage - min_voltage;
    const float mean_voltage =
        voltage_sum / static_cast<float>(BUFFER_SIZE);
    float voltage_variance =
        voltage_square_sum / static_cast<float>(BUFFER_SIZE) -
        mean_voltage * mean_voltage;
    if (voltage_variance < 0.0f) {
        voltage_variance = 0.0f;
    }
    // The input is composed of AC fundamental/harmonic components. Removing
    // ADC DC offset matches the notebook's spectrum-derived RMS.
    result.v_rms = sqrtf(voltage_variance);

    // Match the notebook's order: MedianFilter -> LowpassFilter -> FFT.
    arm_fir_decimate_instance_f32 fir = {};
    if (arm_fir_decimate_init_f32(
        &fir,
        NUM_TAPS,
        DECIMATION_FACTOR,
        FIR_COEFFICIENTS,
        FIR_STATE,
        FIR_BLOCK_SIZE
    ) != ARM_MATH_SUCCESS) {
        panic("cannot initialize FIR decimator");
    }

    uint32_t fft_input_index = 0;
    for (
        uint32_t block_start = 0;
        block_start < BUFFER_SIZE;
        block_start += FIR_BLOCK_SIZE
    ) {
        for (uint32_t i = 0; i < FIR_BLOCK_SIZE; ++i) {
            FIR_INPUT[i] = compute_adc_voltage(
                static_cast<int32_t>(
                    MEDIAN_BUFFER[block_start + i]
                )
            ) - mean_voltage;
        }

        arm_fir_decimate_f32(
            &fir,
            FIR_INPUT,
            FIR_OUTPUT,
            FIR_BLOCK_SIZE
        );

        for (uint32_t i = 0;
             i < FIR_BLOCK_SIZE / DECIMATION_FACTOR;
             ++i) {
            FFT_INPUT[fft_input_index++] = FIR_OUTPUT[i];
        }
    }

    // Generate the periodic 92 dB Blackman-Harris window with CMSIS-DSP.
    // FFT_OUTPUT is scratch storage here and is overwritten by the RFFT.
    arm_blackman_harris_92db_f32(FFT_OUTPUT, FFT_SIZE);
    float window_sum = 0.0f;
    for (uint32_t i = 0; i < FFT_SIZE; ++i) {
        const float window = FFT_OUTPUT[i];
        window_sum += window;
        FFT_INPUT[i] *= window;
    }

    arm_rfft_fast_instance_f32 fft = {};
    if (arm_rfft_fast_init_4096_f32(&fft) != ARM_MATH_SUCCESS) {
        panic("cannot initialize 4096-point float RFFT");
    }
    arm_rfft_fast_f32(&fft, FFT_INPUT, FFT_OUTPUT, 0);

    const float coherent_gain =
        window_sum / static_cast<float>(FFT_SIZE);

    for (uint32_t bin = 0; bin < SPECTRUM_BIN_COUNT; ++bin) {
        SPECTRUM_SUM[bin] += spectrum_magnitude_mv(
            FFT_OUTPUT,
            bin,
            coherent_gain
        );
    }

    return result;
}

static uint16_t RAW_BUFFER[BUFFER_SIZE] = {};
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
    for (uint32_t bin = 0; bin < SPECTRUM_BIN_COUNT; ++bin) {
        SPECTRUM_SUM[bin] = 0.0f;
    }

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

    find_peak_frequencies(SPECTRUM_SUM, signal_sum.top_freq);

    constexpr uint8_t separator = ',';
    const float repeat_scale = 1.0f / static_cast<float>(REPEAT_TIME);

    // uart_puts(uart_port, "SPECTRUM_MV,");
    // for (uint32_t bin = 0; bin < SPECTRUM_BIN_COUNT; ++bin) {
    //     if (bin != 0) {
    //         uart_write_blocking(uart_port, &separator, 1);
    //     }
    //     uart_write_float(SPECTRUM_SUM[bin] * repeat_scale);
    // }

    uart_puts(uart_port, "PEAKS_HZ,");
    for (size_t i = 0; i < 3; ++i) {
        if (i != 0) {
            uart_write_blocking(uart_port, &separator, 1);
        }
        uart_write_float(signal_sum.top_freq[i]);
    }
    uart_puts(uart_port, "\r\nVPP_MV,");
    uart_write_float(signal_sum.v_pp * repeat_scale);
    uart_puts(uart_port, "\r\nVRMS_MV,");
    uart_write_float(signal_sum.v_rms * repeat_scale);
    uart_puts(uart_port, "\r\n");
    uart_tx_wait_blocking(uart_port);
    return 0;
}
