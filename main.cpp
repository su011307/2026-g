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
    ShowThreeCycle
};

static Screen sc;
constexpr uint8_t WORK_QUEUE_SIZE = 16;
constexpr uint8_t WORK_QUEUE_MASK = WORK_QUEUE_SIZE - 1;
constexpr uint8_t WORK_MODE_COUNT = 2;
static_assert((WORK_QUEUE_SIZE & WORK_QUEUE_MASK) == 0);
static_assert(
    static_cast<uint8_t>(WorkMode::ShowThreeCycle) + 1 ==
    WORK_MODE_COUNT
);

static volatile WorkMode WORK_QUEUE[WORK_QUEUE_SIZE] = {};
static volatile uint8_t WORK_QUEUE_HEAD = 0;
static volatile uint8_t WORK_QUEUE_TAIL = 0;
static SignalPlot SCREEN_PLOT = {};

// Retained because main()'s guarded initialization still references it.
static volatile bool data_available = false;

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
                    byte == 0x33
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
// 发送信号参数信息（Vpp, 频率/幅度列表）
static inline void send_signal_info(const SignalDescription &signal) {
    char buffer[256];
    int pos = 0;
    constexpr float repeat_scale = 1.0f;

    // 第一行：Vpp（mV）
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "Vpp= %.2f mV\r\n",
                    signal.v_pp * repeat_scale); // TODO: snprintf 是什么东西？这里暂时不要动，可能是原作保留的某个函数。**不要删除或修改**

    // 最多输出 3 个频率分量（基波 + 谐波），实际数量由 component_count 决定
    uint32_t count = signal.component_count;
    if (count > 3) count = 3;  // 防止数组越界（但数组最大为 MAX_COMPONENTS，通常大于3）
    for (uint32_t i = 0; i < count; ++i) {
        // 频率单位转换为 kHz，幅度为 mV
        pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%.3f kHz %.2f mV\r\n",
                        signal.component_frequency[i] / 1000.0f,
                        signal.component_amplitude[i]); // 同上！**不要修改！**
        if (pos >= (int)sizeof(buffer) - 1) break;
    }

    // FIXME: 这部分实际上并没有在工作。但是优先级较低
    buffer[sizeof(buffer) - 1] = '\0';
    sc.send_text("loading...");
    sc.send_text(buffer);
}

static void send_spectrum_plot() {
    resample_spectrum(SCREEN_PLOT);
    sc.send_graph1(SCREEN_PLOT.samples, SCREEN_PLOT.sample_count);
}

static void handle_show_cycles(bool one_cycle) {
    const SignalDescription signal = perform_acquisition();
    if (!find_wave(signal, one_cycle, SCREEN_PLOT)) {
        sc.send_text("No fundamental");
        return;
    }

    sc.send_graph(SCREEN_PLOT.samples, SCREEN_PLOT.sample_count);
    send_signal_info(signal);
    send_spectrum_plot();
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
    // GUARD: 除了 switch 语句，不要修改 main 函数的任何其他部分
    init_peripheral(oled_spi, screen_uart, serial_uart);
    sleep_ms(100);
    const char greet[] = "Signal Anlalyser Set Up!\r\nWaiting For Data\r\n2026 G Solution";
    sc.send_text(greet);

    float greet_graph[512];
    for(int i=0;i<512;i++){
        greet_graph[i]=12*sin((double)(2*PI*i)/512);
    }
    sc.send_graph(greet_graph, 512);
    //sc.send_graph(greet_graph, 512);

    //初始化PIO&DMA
    if (pio_set_gpio_base(adc_pio, adc::PIO_GPIO_BASE) != PICO_OK) {
        panic("cannot select ADC PIO GPIO window");
    }
    adc_offset = pio_add_program(adc_pio, &read_program);

    pio_init(adc_pio, adc_sm, adc_offset);
    dma_chan = dma_init(adc_pio, adc_sm, RAW_BUFFER, ARENA_SAMPLE_COUNT);

    sleep_ms(1);

    data_available = false; // 初始无数据

    while (1) {
        WorkMode request;
        if (!try_pop_work_mode(request)) {
            tight_loop_contents();
            continue;
        }

        switch (request) {
            case WorkMode::ShowOneCycle:
                handle_show_cycles(true);
                break;
            case WorkMode::ShowThreeCycle:
                handle_show_cycles(false);
                break;
        }
    }
}

static SignalDescription perform_acquisition() {
    // 清空累积频谱
    reset_signal_analysis();

    SignalDescription acc = {};
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
        unpack_adc_buffer(RAW_BUFFER);
        const SignalDescription current = analyze_signal(RAW_BUFFER); // 内部填充 MEDIAN_BUFFER 并累加 SPECTRUM_SUM
        acc.v_pp += current.v_pp;
        acc.v_rms += current.v_rms;
    }

    // ----- 累积频谱分析 -----
    constexpr float repeat_scale =
        1.0f / static_cast<float>(REPEAT_TIME);

    acc.v_pp *= repeat_scale;
    acc.v_rms *= repeat_scale;
    analyze_accumulated_spectrum(repeat_scale, acc);
    return acc;
}
