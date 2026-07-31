#include "signal.hpp"

#include <arm_math.h>
#include <math.h>
#include <pico/stdlib.h>

#include "definition.hpp"
#include "utility.hpp"

using namespace sig;
using namespace sig::fft;
using namespace sig::flt;

typedef struct {
    uint32_t bin;
    float power;
} SpectrumPeak;

static float FFT_INPUT[FFT_SIZE] = {};
static float FFT_OUTPUT[FFT_SIZE] = {};
static float FIR_INPUT[FIR_BLOCK_SIZE] = {};
static float FIR_OUTPUT[FIR_BLOCK_SIZE / DECIMATION_FACTOR] = {};
static float FIR_STATE[NUM_TAPS + FIR_BLOCK_SIZE - 1] = {};
static float SPECTRUM_SUM[SPECTRUM_BIN_COUNT] = {};
static float SPECTRUM_SORT_SCRATCH[SPECTRUM_BIN_COUNT] = {};
static uint16_t MEDIAN_BUFFER[BUFFER_SIZE] = {};
static float SPECTRUM_WINDOW_ENBW_BINS = 1.0f;
static constexpr float PLOT_VERTICAL_RANGE = 40.0f;

void reset_signal_analysis() {
    for (uint32_t bin = 0; bin < SPECTRUM_BIN_COUNT; ++bin) {
        SPECTRUM_SUM[bin] = 0.0f;
    }
}

static void sort_median_window(uint16_t *window) {
    for (uint32_t i = 1; i < MEDIAN_WINDOW; ++i) {
        const uint16_t value = window[i];
        uint32_t position = i;
        while (position > 0 && window[position - 1] > value) {
            window[position] = window[position - 1];
            --position;
        }
        window[position] = value;
    }
}

static void replace_sorted_window_value(
    uint16_t *window,
    uint16_t outgoing,
    uint16_t incoming
) {
    if (outgoing == incoming) {
        return;
    }

    uint32_t position = 0;
    while (window[position] != outgoing) {
        ++position;
    }

    if (incoming < outgoing) {
        while (position > 0 && window[position - 1] > incoming) {
            window[position] = window[position - 1];
            --position;
        }
    } else {
        while (
            position + 1 < MEDIAN_WINDOW &&
            window[position + 1] < incoming
        ) {
            window[position] = window[position + 1];
            ++position;
        }
    }
    window[position] = incoming;
}

static void apply_median_filter(
    const uint16_t *input,
    uint16_t *output
) {
    uint16_t window[MEDIAN_WINDOW];
    for (uint32_t i = 0; i < MEDIAN_WINDOW; ++i) {
        const int32_t source_index =
            static_cast<int32_t>(i) -
            static_cast<int32_t>(MEDIAN_RADIUS);
        window[i] = input[reflected_sample_index(source_index)];
    }
    sort_median_window(window);
    output[0] = window[MEDIAN_RADIUS];

    for (uint32_t center = 1; center < BUFFER_SIZE; ++center) {
        const int32_t outgoing_index =
            static_cast<int32_t>(center) -
            static_cast<int32_t>(MEDIAN_RADIUS) - 1;
        const int32_t incoming_index =
            static_cast<int32_t>(center) +
            static_cast<int32_t>(MEDIAN_RADIUS);
        replace_sorted_window_value(
            window,
            input[reflected_sample_index(outgoing_index)],
            input[reflected_sample_index(incoming_index)]
        );
        output[center] = window[MEDIAN_RADIUS];
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

static float accumulated_spectrum_power(
    const float *spectrum,
    uint32_t bin
) {
    return spectrum[bin] * spectrum[bin];
}

static float fir_gain_at_frequency(float frequency_hz) {
    const float angular_frequency =
        2.0f * PI * frequency_hz / adc::TARGET_SAMPLE_RATE;
    const float rotation_real = cosf(angular_frequency);
    const float rotation_imag = -sinf(angular_frequency);

    float phasor_real = 1.0f;
    float phasor_imag = 0.0f;
    float response_real = 0.0f;
    float response_imag = 0.0f;
    for (uint32_t tap = 0; tap < NUM_TAPS; ++tap) {
        response_real += FIR_COEFFICIENTS[tap] * phasor_real;
        response_imag += FIR_COEFFICIENTS[tap] * phasor_imag;

        const float next_real =
            phasor_real * rotation_real -
            phasor_imag * rotation_imag;
        phasor_imag =
            phasor_real * rotation_imag +
            phasor_imag * rotation_real;
        phasor_real = next_real;
    }
    return sqrtf(
        response_real * response_real +
        response_imag * response_imag
    );
}

static float median_spectrum_magnitude(
    const float *spectrum,
    float spectrum_scale,
    uint32_t first_bin,
    uint32_t last_bin,
    float *spectrum_sort_scratch
) {
    const uint32_t count = last_bin - first_bin + 1;
    for (uint32_t i = 0; i < count; ++i) {
        spectrum_sort_scratch[i] =
            spectrum[first_bin + i] * spectrum_scale;
    }
    for (uint32_t i = 1; i < count; ++i) {
        const float value = spectrum_sort_scratch[i];
        uint32_t position = i;
        while (
            position > 0 &&
            spectrum_sort_scratch[position - 1] > value
        ) {
            spectrum_sort_scratch[position] =
                spectrum_sort_scratch[position - 1];
            --position;
        }
        spectrum_sort_scratch[position] = value;
    }
    return spectrum_sort_scratch[count / 2];
}

static float interpolate_peak_bin(
    const float *spectrum,
    uint32_t bin
) {
    constexpr float minimum_magnitude = 1.0e-20f;
    const float left = logf(
        spectrum[bin - 1] > minimum_magnitude
            ? spectrum[bin - 1]
            : minimum_magnitude
    );
    const float center = logf(
        spectrum[bin] > minimum_magnitude
            ? spectrum[bin]
            : minimum_magnitude
    );
    const float right = logf(
        spectrum[bin + 1] > minimum_magnitude
            ? spectrum[bin + 1]
            : minimum_magnitude
    );
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
    return static_cast<float>(bin) + offset;
}

static float estimate_component_amplitude_mv(
    const float *spectrum,
    float spectrum_scale,
    uint32_t peak_bin,
    float fir_gain
) {
    // Integrating the complete +/-4-bin Blackman-Harris main lobe and
    // dividing by its equivalent noise bandwidth removes scalloping loss.
    const uint32_t first_bin =
        peak_bin > PEAK_MAIN_LOBE_RADIUS_BINS
            ? peak_bin - PEAK_MAIN_LOBE_RADIUS_BINS
            : 0;
    const uint32_t requested_last_bin =
        peak_bin + PEAK_MAIN_LOBE_RADIUS_BINS;
    const uint32_t last_bin =
        requested_last_bin < SPECTRUM_BIN_COUNT
            ? requested_last_bin
            : SPECTRUM_BIN_COUNT - 1;

    float main_lobe_square_sum = 0.0f;
    for (uint32_t bin = first_bin; bin <= last_bin; ++bin) {
        const float magnitude = spectrum[bin] * spectrum_scale;
        main_lobe_square_sum += magnitude * magnitude;
    }
    const float filtered_amplitude = sqrtf(
        main_lobe_square_sum / SPECTRUM_WINDOW_ENBW_BINS
    );

    // Undo the known FIR magnitude response at the interpolated frequency.
    return fir_gain > 1.0e-6f
        ? filtered_amplitude / fir_gain
        : 0.0f;
}

void analyze_accumulated_spectrum(
    float spectrum_scale,
    SignalDescription &result
) {
    const float *spectrum = SPECTRUM_SUM;
    float *spectrum_sort_scratch = SPECTRUM_SORT_SCRATCH;
    const float bin_width_hz =
        adc::TARGET_SAMPLE_RATE / static_cast<float>(BUFFER_SIZE);

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

    SpectrumPeak candidates[MAX_PEAK_CANDIDATES] = {};
    uint32_t candidate_count = 0;
    for (
        uint32_t peak_index = 0;
        peak_index < MAX_PEAK_CANDIDATES;
        ++peak_index
    ) {
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
            for (uint32_t i = 0; i < candidate_count; ++i) {
                const uint32_t distance =
                    bin > candidates[i].bin
                        ? bin - candidates[i].bin
                        : candidates[i].bin - bin;
                if (distance < PEAK_MIN_SEPARATION_BINS) {
                    separated = false;
                    break;
                }
            }
            if (separated && center > best.power) {
                best = SpectrumPeak{.bin = bin, .power = center};
            }
        }
        if (best.power == 0.0f) {
            break;
        }
        candidates[candidate_count++] = best;
    }

    const float noise_floor = median_spectrum_magnitude(
        spectrum,
        spectrum_scale,
        first_peak_bin,
        last_peak_bin,
        spectrum_sort_scratch
    );
    const float filtered_snr_threshold =
        noise_floor * COMPONENT_MIN_SNR;

    result.component_count = 0;
    for (
        uint32_t i = 0;
        i < candidate_count &&
        result.component_count < MAX_COMPONENTS;
        ++i
    ) {
        const uint32_t bin = candidates[i].bin;
        const float interpolated_bin =
            interpolate_peak_bin(spectrum, bin);
        const float frequency_hz = interpolated_bin * bin_width_hz;
        const float fir_gain = fir_gain_at_frequency(frequency_hz);
        const float amplitude_mv = estimate_component_amplitude_mv(
            spectrum,
            spectrum_scale,
            bin,
            fir_gain
        );
        const float compensated_snr_threshold =
            fir_gain > 1.0e-6f
                ? filtered_snr_threshold / fir_gain
                : filtered_snr_threshold;
        const float amplitude_threshold =
            compensated_snr_threshold > COMPONENT_MIN_AMPLITUDE_MV
                ? compensated_snr_threshold
                : COMPONENT_MIN_AMPLITUDE_MV;
        if (amplitude_mv < amplitude_threshold) {
            continue;
        }

        const uint32_t output_index = result.component_count++;
        result.component_frequency[output_index] = frequency_hz;
        result.component_amplitude[output_index] = amplitude_mv;
    }

    // Report components on the positive frequency axis in ascending order.
    for (uint32_t i = 1; i < result.component_count; ++i) {
        const float frequency = result.component_frequency[i];
        const float amplitude = result.component_amplitude[i];
        uint32_t position = i;
        while (
            position > 0 &&
            result.component_frequency[position - 1] > frequency
        ) {
            result.component_frequency[position] =
                result.component_frequency[position - 1];
            result.component_amplitude[position] =
                result.component_amplitude[position - 1];
            --position;
        }
        result.component_frequency[position] = frequency;
        result.component_amplitude[position] = amplitude;
    }
}

SignalDescription analyze_signal(uint16_t *buffer) {
    float min_voltage = 32768;
    float max_voltage = -32768;
    float voltage_sum = 0.0f;
    float voltage_square_sum = 0.0;

    // Since voltage conversion is monotonic, filtering ADC codes gives the
    // same median as filtering the converted voltages.
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
    float window_square_sum = 0.0f;
    for (uint32_t i = 0; i < FFT_SIZE; ++i) {
        const float window = FFT_OUTPUT[i];
        window_sum += window;
        window_square_sum += window * window;
        FFT_INPUT[i] *= window;
    }
    SPECTRUM_WINDOW_ENBW_BINS =
        static_cast<float>(FFT_SIZE) *
        window_square_sum /
        (window_sum * window_sum);

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

static void clear_plot(SignalPlot &plot, uint16_t sample_count) {
    plot.sample_count = sample_count;
    for (uint32_t i = 0; i < sample_count; ++i) {
        plot.samples[i] = 0.0f;
    }
}

static float interpolated_median_voltage(float sample_index) {
    uint32_t left = static_cast<uint32_t>(sample_index);
    if (left >= BUFFER_SIZE - 1) {
        left = BUFFER_SIZE - 2;
    }
    const float fraction = sample_index - static_cast<float>(left);
    const float left_voltage = compute_adc_voltage(MEDIAN_BUFFER[left]);
    const float right_voltage = compute_adc_voltage(MEDIAN_BUFFER[left + 1]);
    return left_voltage + fraction * (right_voltage - left_voltage);
}

bool find_wave(
    const SignalDescription &signal,
    bool one_wave,
    SignalPlot &plot
) {
    clear_plot(plot, static_cast<uint16_t>(PLOT_SIZE));
    if (
        signal.component_count == 0 ||
        signal.component_frequency[0] <= 0.0f
    ) {
        return false;
    }

    const float period_samples =
        adc::TARGET_SAMPLE_RATE / signal.component_frequency[0];
    const float cycle_count = one_wave ? 1.0f : 3.0f;
    const float source_span = period_samples * cycle_count;
    const uint32_t source_span_ceil =
        static_cast<uint32_t>(ceilf(source_span));
    if (
        source_span_ceil + MEDIAN_RADIUS + 2 >= BUFFER_SIZE ||
        period_samples < 2.0f
    ) {
        return false;
    }

    // Start at the first voltage maximum in a fundamental period. This gives
    // every redraw a deterministic phase without exposing the median buffer.
    const uint32_t latest_start =
        BUFFER_SIZE - source_span_ceil - 2;
    uint32_t start = MEDIAN_RADIUS;
    uint32_t search_end =
        start + static_cast<uint32_t>(ceilf(period_samples));
    if (search_end > latest_start) {
        search_end = latest_start;
    }
    for (uint32_t i = start + 1; i <= search_end; ++i) {
        // ADC conversion is descending, so the smallest code is the maximum
        // voltage.
        if (MEDIAN_BUFFER[i] < MEDIAN_BUFFER[start]) {
            start = i;
        }
    }

    float min_voltage = interpolated_median_voltage(
        static_cast<float>(start)
    );
    float max_voltage = min_voltage;
    const uint32_t last_source_sample =
        start + source_span_ceil + 1;
    for (uint32_t i = start + 1; i <= last_source_sample; ++i) {
        const float voltage = compute_adc_voltage(MEDIAN_BUFFER[i]);
        if (voltage < min_voltage) {
            min_voltage = voltage;
        }
        if (voltage > max_voltage) {
            max_voltage = voltage;
        }
    }

    const float voltage_span = max_voltage - min_voltage;
    if (voltage_span <= 1.0e-6f) {
        return false;
    }
    const float center_voltage = 0.5f * (min_voltage + max_voltage);
    const float plot_scale =
        2.0f * PLOT_VERTICAL_RANGE / voltage_span;

    for (uint32_t i = 0; i < PLOT_SIZE; ++i) {
        const float position =
            static_cast<float>(start) +
            source_span * static_cast<float>(i) /
                static_cast<float>(PLOT_SIZE - 1);
        float value =
            (interpolated_median_voltage(position) - center_voltage) *
            plot_scale;
        if (value < -PLOT_VERTICAL_RANGE) {
            value = -PLOT_VERTICAL_RANGE;
        } else if (value > PLOT_VERTICAL_RANGE) {
            value = PLOT_VERTICAL_RANGE;
        }
        plot.samples[i] = value;
    }
    return true;
}

void resample_spectrum(SignalPlot &plot) {
    clear_plot(plot, static_cast<uint16_t>(SPECTRUM_SIZE));

    const float bin_width_hz =
        adc::TARGET_SAMPLE_RATE / static_cast<float>(BUFFER_SIZE);
    uint32_t last_bin = static_cast<uint32_t>(
        floorf(PEAK_MAX_FREQUENCY_HZ / bin_width_hz)
    );
    if (last_bin >= SPECTRUM_BIN_COUNT) {
        last_bin = SPECTRUM_BIN_COUNT - 1;
    }

    float minimum = INFINITY;
    float maximum = 0.0f;
    for (uint32_t i = 0; i < SPECTRUM_SIZE; ++i) {
        const float source_position =
            static_cast<float>(last_bin) * static_cast<float>(i) /
            static_cast<float>(SPECTRUM_SIZE - 1);
        const uint32_t left = static_cast<uint32_t>(source_position);
        const uint32_t right = left < last_bin ? left + 1 : left;

        // Taking the larger adjacent bin preserves a narrow spectral peak
        // while mapping the 0--500 kHz positive spectrum to the display.
        // The normalization below removes any common accumulation scale.
        const float left_magnitude = SPECTRUM_SUM[left];
        const float right_magnitude = SPECTRUM_SUM[right];
        const float magnitude =
            left_magnitude > right_magnitude
                ? left_magnitude
                : right_magnitude;
        plot.samples[i] = magnitude;
        if (magnitude < minimum) {
            minimum = magnitude;
        }
        if (magnitude > maximum) {
            maximum = magnitude;
        }
    }

    const float magnitude_span = maximum - minimum;
    if (magnitude_span <= 0.0f) {
        return;
    }

    // Screen::send_graph1 applies a fixed positive offset after scaling by
    // the input span. Centering the spectrum around zero keeps its peak below
    // uint8_t saturation while retaining the complete display dynamic range.
    const float plot_scale =
        2.0f * PLOT_VERTICAL_RANGE / magnitude_span;
    for (uint32_t i = 0; i < SPECTRUM_SIZE; ++i) {
        plot.samples[i] =
            (plot.samples[i] - minimum) * plot_scale -
            PLOT_VERTICAL_RANGE;
    }
}
