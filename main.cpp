#include <cstdint>
#include <cstdio>
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

// An odd repeat count gives each scalar measurement a unique median.
constexpr uint32_t REPEAT_TIME = 5;
static_assert(REPEAT_TIME > 0);
static_assert((REPEAT_TIME & 1u) == 1u);


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

enum class WorkMode : uint8_t {
    Calibrate, /* 校准逻辑 */
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
    static_cast<uint8_t>(WorkMode::ShowSpectrumPeak) + 1;
static_assert((WORK_QUEUE_SIZE & WORK_QUEUE_MASK) == 0);
static_assert(WORK_MODE_COUNT == 6);

static volatile WorkMode WORK_QUEUE[WORK_QUEUE_SIZE] = {};
static volatile uint8_t WORK_QUEUE_HEAD = 0;
static volatile uint8_t WORK_QUEUE_TAIL = 0;
static SignalPlot SCREEN_PLOT = {};

static volatile bool in_calibration = false;

static uint dma_chan;
static uint adc_offset;

static void insert_sorted_measurement(
    float *measurements,
    uint32_t sorted_count,
    float value
) {
    uint32_t position = sorted_count;
    while (position > 0 && measurements[position - 1] > value) {
        measurements[position] = measurements[position - 1];
        --position;
    }
    measurements[position] = value;
}

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
    static WorkMode selected_mode = WorkMode::ShowOneCycle;

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
                    byte == 0x34 ||
                    byte == 0x35 ||
                    byte == 0x36 ||
                    byte == 0x37
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
                    case 0x34:
                        selected_mode = WorkMode::ShowBriefDescription;
                        break;
                    case 0x35:
                        selected_mode = WorkMode::ShowSpectrumPlot;
                        break;
                    case 0x36:
                        selected_mode = WorkMode::ShowSpectrumPeak;
                        break;
                    case 0x37:
                        selected_mode = WorkMode::Calibrate;
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

static SignalDescription perform_acquisition();

static void send_brief_description(const SignalDescription &signal) {
    char buffer[256] = {};
    int pos = 0;
    constexpr float repeat_scale = 1.0f;

    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "Vpp= %.2f mV\r\nVrms= %.2f mv\r\n",
                    signal.v_pp * repeat_scale,signal.v_rms); // TODO: snprintf 是什么东西？这里暂时不要动，可能是原作保留的某个函数。**不要删除或修改**

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

static void send_spectrum_plot() {
    resample_spectrum(SCREEN_PLOT);
    sc.send_graph1(SCREEN_PLOT.samples, SCREEN_PLOT.sample_count);
}

static void smoothen_signal_graph(float samples[]) {
    // 长度为 3 的滑窗，取平均值（缓解 ADC 采样的微小毛刺）。头尾两个数据点保持原样
    float previous = samples[0];
    float current = samples[1];
    for (uint32_t i = 1; i + 1 < sig::PLOT_SIZE; ++i) {
        const float next = samples[i + 1];
        samples[i] = (previous + current + next) / 3.0f;
        previous = current;
        current = next;
    }
}

static void handle_show_cycles(
    const SignalDescription &signal,
    bool one_cycle
) {
    if (!find_wave(signal, one_cycle, SCREEN_PLOT)) {
        sc.send_text("No fundamental");
        return;
    }

    smoothen_signal_graph(SCREEN_PLOT.samples);
    sc.send_graph(SCREEN_PLOT.samples, SCREEN_PLOT.sample_count);
}

static void clear() {
    float tmp[sig::PLOT_SIZE] = {};
    sc.send_text("");
    sc.send_graph(tmp, sig::PLOT_SIZE);
    sc.send_graph1(tmp, sig::PLOT_SIZE);
}

static void handle_calibration() {
    // 从 ADC 读取若干个采样（如 15 个），然后计算出平均 Vpp 和目标 Vpp 的偏差，然后计算出输出 Vpp 时的校准量
    // 出现这个情况，很可能是因为 max - min 算法，受 ADC 噪声影响。即便应用了中值滤波，也只能滤除大的尖峰，对于
    // 小毛刺没有很好的处理办法。RMS 按 1.5 LSB
}

static void dispatch_work_mode(WorkMode request) {
    if (request == WorkMode::Calibrate) {
        if (in_calibration) {
            handle_calibration();
        } else {
            sc.send_text(
                "To calibrate: set sig. to 50mV@500kHz, press button again"
            );
        }
        return;
    }

    if (in_calibration) {
        sc.send_text("Calibrating. Please follow the instruction.");
        return;
    }

    clear();

    // 除校准外，每个任务只采集、分析一次，再按模式选择输出。
    const SignalDescription signal = perform_acquisition();
    switch (request) {
        case WorkMode::ShowOneCycle:
            handle_show_cycles(signal, true);
            break;
        case WorkMode::ShowThreeCycle:
            handle_show_cycles(signal, false);
            break;
        case WorkMode::ShowBriefDescription:
            send_brief_description(signal);
            break;
        case WorkMode::ShowSpectrumPlot:
            send_spectrum_plot();
            break;
        case WorkMode::ShowSpectrumPeak:
            send_spectrum_peaks(signal);
            break;
        case WorkMode::Calibrate:
            break;
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

int main() {
    init_peripheral(oled_spi, screen_uart, serial_uart);
    sleep_ms(100);
    const char greet[] = "Signal Anlalyser Set Up!\r\nWaiting For Data\r\n2026 G Solution";
    sc.send_text(greet);

    // float greet_graph[512];
    // for (int i=0;i<512;i++){
    //     greet_graph[i]=12*sin((double)(2*PI*i)/512);
    // }
    // sc.send_graph(greet_graph, 512);
    //sc.send_graph(greet_graph, 512);

    //初始化PIO&DMA
    if (pio_set_gpio_base(adc_pio, adc::PIO_GPIO_BASE) != PICO_OK) {
        panic("cannot select ADC PIO GPIO window");
    }
    adc_offset = pio_add_program(adc_pio, &read_program);

    pio_init(adc_pio, adc_sm, adc_offset);
    dma_chan = dma_init(adc_pio, adc_sm, RAW_BUFFER, ARENA_SAMPLE_COUNT);

    sleep_ms(1);

    while (1) {
        WorkMode request;
        if (!try_pop_work_mode(request)) {
            tight_loop_contents();
            continue;
        }

        dispatch_work_mode(request);
    }
}

static SignalDescription perform_acquisition() {
    // 清空累积频谱
    reset_signal_analysis();

    float v_pp_measurements[REPEAT_TIME] = {};
    float v_rms_measurements[REPEAT_TIME] = {};
    for (uint32_t repeat = 0; repeat < REPEAT_TIME; ++repeat) {
        // ----- 启动 DMA 采样 -----
        pio_sm_clear_fifos(adc_pio, adc_sm);
        dma_channel_set_write_addr(dma_chan, RAW_BUFFER, false);
        dma_channel_set_trans_count(dma_chan, ARENA_SAMPLE_COUNT, false);
        dma_channel_start(dma_chan);
        pio_sm_set_enabled(adc_pio, adc_sm, true);
        dma_channel_wait_for_finish_blocking(dma_chan);
        pio_sm_set_enabled(adc_pio, adc_sm, false);

        // ----- 数据解包与信号分析 -----
        // 前 256 点覆盖 ADC 流水线及停钟后的稳定时间，只处理后续
        // 65536 个连续采样点。
        unpack_adc_buffer(ADC_BUFFER);
        const SignalDescription current = analyze_signal(ADC_BUFFER); // 内部填充 MEDIAN_BUFFER 并累加 SPECTRUM_SUM

        // FIXME: 似乎每 9-10 次稳定出现一个离群值？
        // 而且大部分采样点似乎稳定地高于实际 Vpp 约 10mV（离群值约 20mV）
        insert_sorted_measurement(
            v_pp_measurements,
            repeat,
            current.v_pp
        );
        insert_sorted_measurement(
            v_rms_measurements,
            repeat,
            current.v_rms
        );
    }

    // ----- 累积频谱分析 -----
    constexpr float repeat_scale =
        1.0f / static_cast<float>(REPEAT_TIME);

    SignalDescription result = {};
    result.v_pp = v_pp_measurements[REPEAT_TIME / 2];
    result.v_rms = v_rms_measurements[REPEAT_TIME / 2];

    analyze_accumulated_spectrum(repeat_scale, result);
    return result;
}
