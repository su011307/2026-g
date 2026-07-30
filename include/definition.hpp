#pragma once
#include <cstdint>
#include <pico/types.h>

// lcd的引脚
constexpr uint SPI_MISO_PIN = 16;
constexpr uint SPI_CS_PIN = 17;
constexpr uint SPI_SCK_PIN = 18;
constexpr uint SPI_MOSI_PIN = 19;
constexpr uint LCD_SCL_PIN = SPI_SCK_PIN;
constexpr uint LCD_SDA_PIN = SPI_MOSI_PIN;
constexpr uint LCD_RST_PIN = 20;
constexpr uint LCD_DC_PIN = 21;
constexpr uint LCD_CS_PIN = 22;
constexpr uint LCD_BL_PIN = 23;

constexpr uint UART_RX_PIN = 45;
constexpr uint UART_TX_PIN = 44;

// ad9744的控制引脚
constexpr float DAC_TARGET_SAMPLE_RATE = 3e5;

constexpr uint DAC_CLK_PIN    = 0;
constexpr uint DAC_PD_PIN     = 1; // sleep pin，高电平有效
constexpr uint DAC_DATA_BASE  = 2;
constexpr uint DAC_DATA_COUNT = 14;

constexpr uint DAC_DATA_PINS[] = {
    14, 15, 12, 13,
    10, 11, 8, 9,
    6, 7, 4, 5,
    2, 3
}; // LSB -> MSB；DAC模块接口使用14位二补码，最高位是符号位

// ad9226的控制引脚
constexpr float ADC_TARGET_SAMPLE_RATE = 6.5e7;

constexpr uint ADC_PIO_GPIO_BASE = 16;
constexpr uint ADC_CAPTURE_BASE = 26;
constexpr uint ADC_CAPTURE_COUNT = 14;
constexpr uint ADC_DATA_COUNT = 12;
constexpr uint ADC_CLK_PIN = 28;
constexpr uint ADC_OTR_PIN = 39;
constexpr uint ADC_PIPELINE_DELAY = 7;

// LSB -> MSB. reference.pdf中的走线并不是连续的逻辑位序
constexpr uint ADC_DATA_PINS[] = {
    38, 37, 36, 35,
    34, 33, 26, 27,
    32, 31, 30, 29
};

// read.pio从GPIO26开始一次读取14位：
// raw[13:0] = {OTR,D0,D1,D2,D3,D4,D5,D8,D9,D10,D11,CLK,D7,D6}
constexpr uint16_t reverse_low_bits(uint16_t value, uint count) {
    uint16_t result = 0;
    for (uint i = 0; i < count; ++i) {
        result = static_cast<uint16_t>((result << 1) | ((value >> i) & 1u));
    }
    return result;
}

constexpr uint16_t adc_unpack_code(uint16_t raw) {
    const uint16_t d7_d6 = static_cast<uint16_t>((raw & 0x0003u) << 6);
    const uint16_t d11_d8 =
        static_cast<uint16_t>(reverse_low_bits((raw >> 3) & 0x000fu, 4) << 8);
    const uint16_t d5_d0 =
        reverse_low_bits(static_cast<uint16_t>((raw >> 7) & 0x003fu), 6);
    return static_cast<uint16_t>(d11_d8 | d7_d6 | d5_d0);
}

constexpr bool adc_unpack_otr(uint16_t raw) {
    return ((raw >> 13) & 1u) != 0;
}

namespace adc {
    constexpr float VOLT_CONVERT_CONSTANT = 0.002441406f;
    constexpr float ZERO_DRIFT_COMPENSATE = -40.0f;
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

namespace screen {
    constexpr uint DISP_UART_TX_PIN = 8;
    constexpr uint DISP_UART_RX_PIN = 9;
    constexpr uint DISP_UART_BAUDRATE = 115200;
};

static_assert(adc_unpack_code(1u << 0) == (1u << 6));   // GPIO26 -> D6
static_assert(adc_unpack_code(1u << 1) == (1u << 7));   // GPIO27 -> D7
static_assert(adc_unpack_code(1u << 3) == (1u << 11));  // GPIO29 -> D11
static_assert(adc_unpack_code(1u << 4) == (1u << 10));  // GPIO30 -> D10
static_assert(adc_unpack_code(1u << 5) == (1u << 9));   // GPIO31 -> D9
static_assert(adc_unpack_code(1u << 6) == (1u << 8));   // GPIO32 -> D8
static_assert(adc_unpack_code(1u << 7) == (1u << 5));   // GPIO33 -> D5
static_assert(adc_unpack_code(1u << 8) == (1u << 4));   // GPIO34 -> D4
static_assert(adc_unpack_code(1u << 9) == (1u << 3));   // GPIO35 -> D3
static_assert(adc_unpack_code(1u << 10) == (1u << 2));  // GPIO36 -> D2
static_assert(adc_unpack_code(1u << 11) == (1u << 1));  // GPIO37 -> D1
static_assert(adc_unpack_code(1u << 12) == (1u << 0));  // GPIO38 -> D0
static_assert(adc_unpack_code(1u << 2) == 0);           // GPIO28 is CLK
static_assert(adc_unpack_otr(1u << 13));                // GPIO39 is OTR
