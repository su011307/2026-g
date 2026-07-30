#include <arm_math.h>
#include <cstdint>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/spi.h>
#include <hardware/uart.h>
#include <math.h>
#include <pico/platform/compiler.h>
#include <pico/platform/sections.h>
#include <pico/stdio.h>
#include <pico/stdio_uart.h>
#include <pico/stdlib.h>
#include <pico/time.h>

#include "definition.hpp"
#include "display.hpp"
#include "dsp/fast_math_functions.h"
#include "initialize.hpp"
#include "signal.hpp"
#include "utility.hpp"

#include "read.pio.h"

// Centered median filter: radius 6 gives a 13-sample window.
constexpr uint32_t REPEAT_TIME = 3;
static_assert(REPEAT_TIME > 0);


const auto oled_spi = spi0;
const auto serial_uart = uart0;
const auto screen_uart = uart0;

PIO const adc_pio = pio0;
constexpr uint adc_sm = 0;

static uint16_t RAW_BUFFER[sig::BUFFER_SIZE] = {};
constexpr size_t ARENA_SAMPLE_COUNT = sizeof(RAW_BUFFER) / sizeof(RAW_BUFFER[0]);

enum class WorkMode : uint8_t {
    ShowOneCycle,
    ShowThreeCycle,
    ShowSpectrum,
    ShowDescription
};

static Screen sc;
constexpr uint8_t WORK_QUEUE_SIZE = 16;
constexpr uint8_t WORK_QUEUE_MASK = WORK_QUEUE_SIZE - 1;
constexpr uint8_t WORK_MODE_COUNT = 4;
static_assert((WORK_QUEUE_SIZE & WORK_QUEUE_MASK) == 0);
static_assert(
    static_cast<uint8_t>(WorkMode::ShowDescription) + 1 ==
    WORK_MODE_COUNT
);

static volatile WorkMode WORK_QUEUE[WORK_QUEUE_SIZE] = {};
static volatile uint8_t WORK_QUEUE_HEAD = 0;
static volatile uint8_t WORK_QUEUE_TAIL = 0;
static SignalPlot SCREEN_PLOT = {};

void __isr __not_in_flash_func(screen_irq_handler)();

void init_peripheral(
    spi_inst_t *const oled_spi_port,
    uart_inst_t *const screen_uart_port,
    uart_inst_t *const serial_uart_port
) {
    spi_init(oled_spi_port, 150'0000);
    uart_init(serial_uart_port, disp::DISP_UART_BAUDRATE);
    stdio_uart_init_full(
        screen_uart_port,
        disp::DISP_UART_BAUDRATE,
        disp::DISP_UART_TX_PIN,
        disp::DISP_UART_RX_PIN
    );
    stdio_set_translate_crlf(&stdio_uart, false);
    uart_set_translate_crlf(screen_uart_port, false);

    gpio_init(dbg::SERIAL_UART_TX_PIN);
    gpio_init(dbg::SERIAL_UART_RX_PIN);
    gpio_init(disp::DISP_UART_TX_PIN);
    gpio_init(disp::DISP_UART_RX_PIN);

    gpio_set_function(dbg::SERIAL_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(dbg::SERIAL_UART_RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(disp::DISP_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(disp::DISP_UART_RX_PIN, GPIO_FUNC_UART);

    const uint screen_uart_irq = UART_IRQ_NUM(screen_uart_port);
    irq_set_exclusive_handler(screen_uart_irq, screen_irq_handler);
    irq_set_priority(screen_uart_irq, PICO_LOWEST_IRQ_PRIORITY);
    irq_set_enabled(screen_uart_irq, true);
    uart_set_irq_enables(screen_uart_port, true, false);
}

void __isr __not_in_flash_func(screen_irq_handler)() {
    static uint8_t receive_state = 0;
    static uint8_t command = 0;
    static WorkMode selected_mode = WorkMode::ShowDescription;

    while (uart_is_readable(screen_uart)) {
        const uint8_t byte = static_cast<uint8_t>(
            uart_get_hw(screen_uart)->dr
        );
        switch (receive_state) {
            case 0:
                if (
                    byte == 0x2b ||
                    byte == 0x2d ||
                    byte == 0x31 ||
                    byte == 0x33 ||
                    byte == 0x76
                ) {
                    command = byte;
                    receive_state = 1;
                }
                break;
            case 1:
                receive_state = byte == 0xff ? 2 : 0;
                break;
            case 2:
                receive_state = byte == 0xff ? 3 : 0;
                break;
            case 3: {
                receive_state = 0;
                if (byte != 0xff) {
                    break;
                }

                switch (command) {
                    case 0x2b:
                        selected_mode = static_cast<WorkMode>(
                            (
                                static_cast<uint8_t>(selected_mode) +
                                1
                            ) % WORK_MODE_COUNT
                        );
                        break;
                    case 0x2d:
                        selected_mode = static_cast<WorkMode>(
                            (
                                static_cast<uint8_t>(selected_mode) +
                                WORK_MODE_COUNT - 1
                            ) % WORK_MODE_COUNT
                        );
                        break;
                    case 0x31:
                        selected_mode = WorkMode::ShowOneCycle;
                        break;
                    case 0x33:
                        selected_mode = WorkMode::ShowThreeCycle;
                        break;
                    case 0x76:
                        selected_mode = WorkMode::ShowSpectrum;
                        break;
                    default:
                        break;
                }

                const uint8_t head = WORK_QUEUE_HEAD;
                const uint8_t next_head =
                    (head + 1) & WORK_QUEUE_MASK;
                if (next_head != WORK_QUEUE_TAIL) {
                    WORK_QUEUE[head] = selected_mode;
                    WORK_QUEUE_HEAD = next_head;
                }
                break;
            }
            default:
                receive_state = 0;
                break;
        }
    }
}

static bool try_pop_work_mode(WorkMode &mode) {
    const uint8_t tail = WORK_QUEUE_TAIL;
    if (tail == WORK_QUEUE_HEAD) {
        return false;
    }
    mode = WORK_QUEUE[tail];
    WORK_QUEUE_TAIL = (tail + 1) & WORK_QUEUE_MASK;
    return true;
}

static uint8_t append_text(
    char *output,
    uint8_t index,
    const char *text
) {
    while (*text != '\0') {
        output[index++] = *text++;
    }
    return index;
}

static uint8_t append_float(
    char *output,
    uint8_t index,
    float value
) {
    return index + encode_float_to_string(
        value,
        reinterpret_cast<uint8_t *>(output + index)
    );
}

static void handle_show_one_cycle(
    const SignalDescription &signal
) {
    if (find_wave(signal, true, SCREEN_PLOT)) {
        sc.send_graph(SCREEN_PLOT.samples, SCREEN_PLOT.sample_count);
    } else {
        constexpr char message[] = "No fundamental";
        sc.send_text(message);
    }
}

static void handle_show_three_cycles(
    const SignalDescription &signal
) {
    if (find_wave(signal, false, SCREEN_PLOT)) {
        sc.send_graph(SCREEN_PLOT.samples, SCREEN_PLOT.sample_count);
    } else {
        constexpr char message[] = "No fundamental";
        sc.send_text(message);
    }
}

static void handle_show_spectrum(float spectrum_scale) {
    resample_spectrum(spectrum_scale, SCREEN_PLOT);
    sc.send_graph(SCREEN_PLOT.samples, SCREEN_PLOT.sample_count);
}

static void handle_show_description(
    const SignalDescription &signal,
    float repeat_scale
) {
    char description[80] = {};
    uint8_t length = append_text(description, 0, "F:");
    if (signal.component_count == 0) {
        length = append_text(description, length, "--");
    } else {
        length = append_float(
            description,
            length,
            signal.component_frequency[0]
        );
    }
    length = append_text(description, length, " Vpp:");
    length = append_float(
        description,
        length,
        signal.v_pp * repeat_scale
    );
    length = append_text(description, length, " Vrms:");
    length = append_float(
        description,
        length,
        signal.v_rms * repeat_scale
    );
    description[length] = '\0';
    sc.send_text(description);
}

int main() {
    init_peripheral(oled_spi, screen_uart, serial_uart);
    sleep_ms(100);
    const char greet[] = "Signal Anlalyser Set Up!\r\nWaiting For Data\r\n2026 G Solution";
    sc.send_text(greet);

    float greet_graph[512];
    for(int i=0;i<512;i++){
        greet_graph[i]=12*sin((double)(2*PI*i)/512);
    }
    sc.send_graph(greet_graph, 512);
    sc.send_graph(greet_graph, 512);


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
    /** AAA
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

    */

    handle_show_description(signal_sum, repeat_scale);

    while (1) {
        WorkMode request;
        if (!try_pop_work_mode(request)) {
            tight_loop_contents();
            continue;
        }

        switch (request) {
            case WorkMode::ShowOneCycle:
                handle_show_one_cycle(signal_sum);
                break;
            case WorkMode::ShowThreeCycle:
                handle_show_three_cycles(signal_sum);
                break;
            case WorkMode::ShowSpectrum:
                handle_show_spectrum(repeat_scale);
                break;
            case WorkMode::ShowDescription:
                handle_show_description(signal_sum, repeat_scale);
                break;
        }
    }
}
