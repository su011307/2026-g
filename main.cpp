#include <stdio.h>
#include <stdint.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/spi.h>
#include <hardware/uart.h>
#include <pico/platform/compiler.h>
#include <pico/platform/sections.h>
#include <pico/stdio.h>
#include <pico/stdio_uart.h>
#include <pico/stdlib.h>
#include <pico/time.h>

#include "definition.hpp"
#include "display.hpp"
#include "initialize.hpp"
#include "signal.hpp"
#include "utility.hpp"

#include "read.pio.h"

// Average each scalar measurement across repeated acquisitions.
constexpr uint32_t REPEAT_TIME = 5;
static_assert(REPEAT_TIME > 0);


const auto oled_spi = spi0;
const auto serial_uart = uart1;
const auto screen_uart = uart0;

PIO const adc_pio = pio0;
constexpr uint adc_sm = 0;

static uint16_t RAW_BUFFER[
    adc::STARTUP_DISCARD_SAMPLES + sig::BUFFER_SIZE
] = {};
constexpr size_t ARENA_SAMPLE_COUNT = sizeof(RAW_BUFFER) / sizeof(RAW_BUFFER[0]);
static uint16_t *const ADC_BUFFER =
    RAW_BUFFER + adc::STARTUP_DISCARD_SAMPLES;

enum class Command : uint8_t {
    Calibrate, /* 校准逻辑 */
    EndCalibrate,
    AddCompensate,
    SubCompensate,
    ShowOneCycle, /* 显示一个周期的波形 */
    ShowThreeCycle, /* 显示三个周期的波形 */
    ShowBriefDescription, /* 显示 Vpp、Vrms、基频 */
    ShowSpectrumPlot, /* 向屏幕发送频谱图 */
    ShowSpectrumPeak /* 显示谱峰频率及对应峰值，单位 mV */
};

static Screen sc;
constexpr uint8_t WORK_QUEUE_SIZE = 16;
constexpr uint8_t WORK_QUEUE_MASK = WORK_QUEUE_SIZE - 1;
constexpr uint8_t WORK_MODE_COUNT =
    static_cast<uint8_t>(Command::ShowSpectrumPeak) + 1;
static_assert((WORK_QUEUE_SIZE & WORK_QUEUE_MASK) == 0);
static_assert(WORK_MODE_COUNT == 9);

static volatile Command WORK_QUEUE[WORK_QUEUE_SIZE] = {};
static volatile uint8_t WORK_QUEUE_HEAD = 0;
static volatile uint8_t WORK_QUEUE_TAIL = 0;
static SignalPlot SCREEN_PLOT = {};

static volatile bool in_calibration = false;
static float ADC_COMPENSATE = 0.987;

static uint dma_chan;
static uint adc_offset;

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
    static Command cmd;

    while (uart_is_readable(screen_uart)) {
        const uint8_t byte = static_cast<uint8_t>(
            uart_get_hw(screen_uart)->dr
        );
        switch (receive_state) {
            case 0:
                if (
                    byte == 0x40 ||
                    byte == 0x41 ||
                    byte == 0x31 ||
                    byte == 0x33 ||
                    byte == 0x34 ||
                    byte == 0x35 ||
                    byte == 0x36 ||
                    byte == 0x37 ||
                    byte == 0x38
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
                    case 0x40:
                        cmd = Command::AddCompensate;
                        break;
                    case 0x41:
                        cmd = Command::SubCompensate;
                        break;
                    case 0x31:
                        cmd = Command::ShowOneCycle;
                        break;
                    case 0x33:
                        cmd = Command::ShowThreeCycle;
                        break;
                    case 0x34:
                        cmd = Command::ShowBriefDescription;
                        break;
                    case 0x35:
                        cmd = Command::ShowSpectrumPlot;
                        break;
                    case 0x36:
                        cmd = Command::ShowSpectrumPeak;
                        break;
                    case 0x37:
                        cmd = Command::Calibrate;
                        break;
                    case 0x38:
                        cmd = Command::EndCalibrate;
                        break;
                    default:
                        break;
                }

                const uint8_t head = WORK_QUEUE_HEAD;
                const uint8_t next_head =
                    (head + 1) & WORK_QUEUE_MASK;
                if (next_head != WORK_QUEUE_TAIL) {
                    WORK_QUEUE[head] = cmd;
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

static SignalDescription perform_acquisition();

static void send_brief_description(const SignalDescription &signal) {
    char buffer[256] = {};
    int pos = 0;
    constexpr float repeat_scale = 1.0f;

    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "Vpp= %.2f mV\r\nVrms= %.2f mv\r\n",
                    signal.v_pp * repeat_scale, signal.v_rms); // TODO: snprintf 是什么东西？这里暂时不要动，可能是原作保留的某个函数。**不要删除或修改**

    if (signal.component_count == 0) {
        snprintf(
            buffer + pos,
            sizeof(buffer) - pos,
            "Fundamental= not found"
        );
    } else {
        snprintf(
            buffer + pos,
            sizeof(buffer) - pos,
            "Fundamental= %.3f kHz",
            signal.component_frequency[0] / 1000.0f
        );
    }

    buffer[sizeof(buffer) - 1] = '\0';
    sc.send_text(buffer);
}

static void send_spectrum_peaks(const SignalDescription &signal) {
    if (signal.component_count == 0) {
        sc.send_text("No spectrum peak");
        return;
    }

    char buffer[256] = {};
    int pos = 0;
    for (
        uint32_t i = 0;
        i < signal.component_count && pos < static_cast<int>(sizeof(buffer));
        ++i
    ) {
        const int written = snprintf(
            buffer + pos,
            sizeof(buffer) - static_cast<size_t>(pos),
            "%.3f kHz %.2f mV\r\n",
            signal.component_frequency[i] / 1000.0f,
            signal.component_amplitude[i]
        );
        if (written < 0) {
            break;
        }
        if (written >= static_cast<int>(sizeof(buffer)) - pos) {
            pos = sizeof(buffer) - 1;
            break;
        }
        pos += written;
    }

    buffer[sizeof(buffer) - 1] = '\0';
    sc.send_text(buffer);
}

static void send_spectrum_plot(SignalPlot &plot) {
    resample_spectrum(plot);
    sc.send_spectrum_plot(plot.samples, plot.sample_count);
}

static void handle_show_cycles(
    const SignalDescription &signal,
    bool one_cycle
) {
    if (!find_wave(signal, one_cycle, SCREEN_PLOT)) {
        sc.send_text("No fundamental found");
        return;
    }

    sc.send_wave_plot(SCREEN_PLOT.samples, SCREEN_PLOT.sample_count);
}

static void clear() {
    float tmp[sig::PLOT_SIZE] = {};
    sc.send_text("");
    sc.send_wave_plot(tmp, sig::PLOT_SIZE);
    sc.send_spectrum_plot(tmp, sig::PLOT_SIZE);
}

static void dispatch_work_mode(Command request) {
    clear();
    sc.send_text("Computing...");

    // 除校准外，每个任务只采集、分析一次，再按模式选择输出。
    SignalDescription signal = perform_acquisition();
    for (int i = 0; i < signal.component_count; i++) {
        signal.component_frequency[i] *= 1.0013568;
    }
    switch (request) {
        case Command::ShowOneCycle:
            handle_show_cycles(signal, true);
            sleep_ms(100);
            sc.send_text("Finished.");
            break;
        case Command::ShowThreeCycle:
            handle_show_cycles(signal, false);
            sleep_ms(100);
            sc.send_text("Finished.");
            break;
        case Command::ShowBriefDescription:
            send_brief_description(signal);
            break;
        case Command::ShowSpectrumPlot:
            send_spectrum_plot(SCREEN_PLOT);
            sleep_ms(100);
            sc.send_text("Finished.");
            break;
        case Command::ShowSpectrumPeak:
            send_spectrum_peaks(signal);
            break;
        case Command::Calibrate:
            in_calibration = true;
            sc.send_text("Calibrating...");
            sleep_ms(100);
            char tune_text[64];
            sprintf(tune_text, "AM: %.3f ", ADC_COMPENSATE);
            sc.send_tune_text(tune_text);
            break;
        case Command::EndCalibrate:
            in_calibration = false;
            break;
        case Command::AddCompensate:
            if (!in_calibration) {
                break;
            }
            ADC_COMPENSATE += 0.001;
            sprintf(tune_text,"AM: %.3f ",ADC_COMPENSATE);
            sc.send_tune_text(tune_text);
            break;
        case Command::SubCompensate:
            if (!in_calibration) {
                break;
            }
            ADC_COMPENSATE -= 0.001;
            sprintf(tune_text,"AM: %.3f ",ADC_COMPENSATE);
            sc.send_tune_text(tune_text);
    }
}

static bool try_pop_work_mode(Command &mode) {
    const uint8_t tail = WORK_QUEUE_TAIL;
    if (tail == WORK_QUEUE_HEAD) {
        return false;
    }
    mode = WORK_QUEUE[tail];
    WORK_QUEUE_TAIL = (tail + 1) & WORK_QUEUE_MASK;
    return true;
}

int main() {
    init_peripheral(oled_spi, screen_uart, serial_uart);
    sleep_ms(100);
    const char greet[] = "Signal Anlalyser Set Up!\r\nWaiting For Data\r\n2026 G Solution";
    sc.send_text(greet);

    clear();
    sc.send_text("Initialized. Not calibrated yet.");

    if (pio_set_gpio_base(adc_pio, adc::PIO_GPIO_BASE) != PICO_OK) {
        panic("cannot select ADC PIO GPIO window");
    }
    adc_offset = pio_add_program(adc_pio, &read_program);

    pio_init(adc_pio, adc_sm, adc_offset);
    dma_chan = dma_init(adc_pio, adc_sm, RAW_BUFFER, ARENA_SAMPLE_COUNT);

    sleep_ms(1);

    while (1) {
        Command request;
        if (!try_pop_work_mode(request)) {
            tight_loop_contents();
            continue;
        }

        dispatch_work_mode(request);
    }
}

static SignalDescription perform_acquisition() {
    reset_signal_analysis();

    for (uint32_t repeat = 0; repeat < REPEAT_TIME; ++repeat) {
        pio_sm_clear_fifos(adc_pio, adc_sm);
        dma_channel_set_write_addr(dma_chan, RAW_BUFFER, false);
        dma_channel_set_trans_count(dma_chan, ARENA_SAMPLE_COUNT, false);
        dma_channel_start(dma_chan);
        pio_sm_set_enabled(adc_pio, adc_sm, true);
        dma_channel_wait_for_finish_blocking(dma_chan);
        pio_sm_set_enabled(adc_pio, adc_sm, false);

        unpack_adc_buffer(ADC_BUFFER);
        analyze_signal(ADC_BUFFER, ADC_COMPENSATE);
    }

    constexpr float repeat_scale =
        1.0f / static_cast<float>(REPEAT_TIME);

    SignalDescription result = {};
    analyze_accumulated_spectrum(repeat_scale, result);
    return result;
}
