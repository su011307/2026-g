#pragma once
#include <cstdint>
#include <pico/types.h>

namespace adc {
    constexpr float VOLT_CONVERT_CONSTANT = 0.002441406f;
    constexpr float ZERO_DRIFT_COMPENSATE = -40.0f;
    constexpr float TARGET_SAMPLE_RATE = 6.5e7;

    constexpr uint PIO_GPIO_BASE = 16;
    constexpr uint CAPTURE_BASE = 26;
    constexpr uint CAPTURE_COUNT = 14;
    constexpr uint DATA_COUNT = 12;
    constexpr uint CLK_PIN = 28;
    constexpr uint OTR_PIN = 39;
    constexpr uint PIPELINE_DELAY = 7;

    // LSB -> MSB. reference.pdf中的走线并不是连续的逻辑位序
    // 注意 AD9226 的 LSB 和 MSB 与引脚的逆序关系
    constexpr uint DATA_PINS[] = {
        38, 37, 36, 35,
        34, 33, 26, 27,
        32, 31, 30, 29
    };
};

namespace dac {
    // ad9744的控制引脚
    constexpr float TARGET_SAMPLE_RATE = 3e5;

    constexpr uint CLK_PIN    = 0;
    constexpr uint PD_PIN     = 1; // sleep pin，高电平有效
    constexpr uint DATA_BASE  = 2;
    constexpr uint DATA_COUNT = 14;

    constexpr uint DATA_PINS[] = {
        14, 15, 12, 13,
        10, 11, 8, 9,
        6, 7, 4, 5,
        2, 3
    }; // LSB -> MSB；DAC模块接口使用14位二补码，最高位是符号位
};

namespace dbg {
    constexpr uint SERIAL_UART_RX_PIN = 45;
    constexpr uint SERIAL_UART_TX_PIN = 44;
};

namespace sig {
    constexpr uint32_t BUFFER_SIZE = 65536;

    namespace flt {
        constexpr uint32_t MEDIAN_RADIUS = 7;
        constexpr uint32_t MEDIAN_WINDOW = 2 * MEDIAN_RADIUS + 1;
        constexpr uint32_t FIR_ORDER = 64;
        constexpr uint32_t NUM_TAPS = FIR_ORDER + 1;
        constexpr uint32_t FIR_BLOCK_SIZE = 256;
        constexpr float FIR_COEFFICIENTS[NUM_TAPS] = {
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
    };

    namespace fft {
        constexpr uint32_t FFT_SIZE = 4096;

        constexpr float COMPONENT_MIN_AMPLITUDE_MV = 2.0f;
        constexpr float COMPONENT_MIN_SNR = 6.0f;

        constexpr float PEAK_MIN_FREQUENCY_HZ = 10'000.0f;
        constexpr float PEAK_MAX_FREQUENCY_HZ = 500'000.0f;

        constexpr uint32_t DECIMATION_FACTOR = BUFFER_SIZE / FFT_SIZE;
        constexpr uint32_t SPECTRUM_BIN_COUNT = FFT_SIZE / 2 + 1;
        constexpr uint32_t MAX_COMPONENTS = 3;
        constexpr uint32_t MAX_PEAK_CANDIDATES = 12;
        constexpr uint32_t PEAK_MAIN_LOBE_RADIUS_BINS = 4;
        constexpr uint32_t PEAK_MIN_SEPARATION_BINS =
            2 * PEAK_MAIN_LOBE_RADIUS_BINS + 1;
    };

    static_assert(BUFFER_SIZE % fft::FFT_SIZE == 0);
    static_assert((fft::FFT_SIZE & (fft::FFT_SIZE - 1)) == 0);
    static_assert(BUFFER_SIZE % flt::FIR_BLOCK_SIZE == 0);
    static_assert(flt::FIR_BLOCK_SIZE % fft::DECIMATION_FACTOR == 0);
    static_assert((flt::MEDIAN_WINDOW & 1u) == 1u);
    static_assert(BUFFER_SIZE >= flt::MEDIAN_WINDOW);
};

namespace oled {
    // lcd的引脚
    constexpr uint SPI_MISO_PIN = 16;
    constexpr uint SPI_CS_PIN = 17;
    constexpr uint SCL_PIN = 18;
    constexpr uint SDA_PIN = 19;
    constexpr uint RST_PIN = 20;
    constexpr uint DC_PIN = 21;
    constexpr uint CS_PIN = 22;
    constexpr uint BL_PIN = 23;
};

namespace disp {
    constexpr uint DISP_UART_TX_PIN = 8;
    constexpr uint DISP_UART_RX_PIN = 9;
    constexpr uint DISP_UART_BAUDRATE = 115200;
};
