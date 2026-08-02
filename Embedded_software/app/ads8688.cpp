#include <cstddef>
#include <cstdint>

#include "ads8688.pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "pico/types.h"

namespace {

constexpr uint SPI_PORT_INDEX = 0;
constexpr uint SPI_BAUD_RATE = 1500'000;
constexpr uint SPI_MISO_PIN = 16;
constexpr uint SPI_SCK_PIN = 18;
constexpr uint SPI_MOSI_PIN = 19;

constexpr uint UART_PORT_INDEX = 0;
constexpr uint UART_BAUD_RATE = 1'500'000;
constexpr uint UART_RX_PIN = 45;
constexpr uint UART_TX_PIN = 44;
constexpr uint UART_DATA_BITS = 8;
constexpr uint UART_STOP_BITS = 1;
constexpr uint UART_MAX_FRAME_SIZE = 9;
constexpr uint UART_BITS_PER_FRAME_BYTE = 10;

constexpr uint ADS_CS_PIN = 46;
constexpr uint ADS_RST_PIN = 47;
constexpr uint ADS_SAMPLE_RATE = 5'000;
constexpr uint ADS_POWER_UP_DELAY_MS = 15;
constexpr uint ADS_INITIALIZATION_CS_HIGH_US = 1;
constexpr uint ADS_DATA_FRAME_SIZE = 4;
constexpr uint ADS_DATA_MSB_INDEX = 2;
constexpr uint ADS_DATA_LSB_INDEX = 3;
constexpr uint ADS_PROGRAM_REGISTER_READBACK_INDEX = 2;
constexpr uint16_t ADS_STRAIGHT_BINARY_ZERO_CODE = 32768u;

constexpr uint PIO_INDEX = 0;
constexpr uint PIO_STATE_MACHINE = 0;
constexpr uint PIO_GPIO_BASE = 16;
constexpr uint PIO_IRQ_INDEX = 0;
constexpr uint PIO_CYCLES_PER_SAMPLE = 40;
constexpr uint PIO_CYCLES_FROM_IRQ_TO_CS_HIGH = 8;
constexpr uint PIO_CLOCK_RATE = ADS_SAMPLE_RATE * PIO_CYCLES_PER_SAMPLE;

// ADS8688 program-register write: AUTO_SEQ_EN (01h) = FFh.
constexpr uint8_t ADS_AUTO_SEQUENCE_FRAME[] = {0x03, 0xff, 0x00};
// ADS8688 program-register read: AUTO_SEQ_EN (01h), WR/RD = 0.
constexpr uint8_t ADS_AUTO_SEQUENCE_READ_FRAME[] = {0x02, 0x00, 0x00};
constexpr uint8_t ADS_AUTO_SEQUENCE_EXPECTED_READBACK = 0xff;
// ADS8688 command-register write: MAN_Ch_7 = DC00h. A complete 32-clock
// command frame is required to acquire channel 7 accurately.
constexpr uint8_t ADS_MANUAL_CHANNEL_7_FRAME[] = {
    0xd8, 0x00, 0x00, 0x00
};
constexpr uint8_t ADS_NO_OPERATION_FRAME[ADS_DATA_FRAME_SIZE] = {};
constexpr char ADS_SPI_WRITE_FAILURE_PREFIX[] =
    "ADS8688 SPI write failed: expected [255], got ";
constexpr char UART_LINE_END[] = "\r\n";

static_assert(1'000'000 % ADS_SAMPLE_RATE == 0);
static_assert(
    ADS_DATA_FRAME_SIZE * 8 * PIO_CLOCK_RATE <
    PIO_CYCLES_FROM_IRQ_TO_CS_HIGH * SPI_BAUD_RATE
);
static_assert(
    UART_MAX_FRAME_SIZE * UART_BITS_PER_FRAME_BYTE * ADS_SAMPLE_RATE <
    UART_BAUD_RATE
);

void ads_sio_transfer(
    spi_inst_t *spi,
    const uint8_t *tx,
    uint8_t *rx,
    uint length
) {
    gpio_put(ADS_CS_PIN, false);
    spi_write_read_blocking(spi, tx, rx, length);
    gpio_put(ADS_CS_PIN, true);
    busy_wait_us_32(ADS_INITIALIZATION_CS_HIGH_US);
}

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

[[noreturn]] void report_spi_write_failure(
    uart_inst_t *uart,
    uint8_t readback
) {
    uart_puts(uart, ADS_SPI_WRITE_FAILURE_PREFIX);

    uint8_t uart_frame[UART_MAX_FRAME_SIZE] = {};
    const uint uart_frame_size =
        encode_uart_frame(readback, uart_frame);
    uart_write_blocking(uart, uart_frame, uart_frame_size);
    uart_puts(uart, UART_LINE_END);

    while (true) {
        tight_loop_contents();
    }
}

void ads_cs_pio_init(PIO pio, uint offset) {
    pio_sm_claim(pio, PIO_STATE_MACHINE);

    pio_sm_config config = ads8688_cs_program_get_default_config(offset);
    sm_config_set_set_pins(&config, ADS_CS_PIN, 1);
    sm_config_set_clkdiv(
        &config,
        static_cast<float>(clock_get_hz(clk_sys)) /
            static_cast<float>(PIO_CLOCK_RATE)
    );

    pio_sm_init(pio, PIO_STATE_MACHINE, offset, &config);
    pio_sm_set_consecutive_pindirs(
        pio,
        PIO_STATE_MACHINE,
        ADS_CS_PIN,
        1,
        true
    );
    pio_sm_set_pins_with_mask64(
        pio,
        PIO_STATE_MACHINE,
        1ull << ADS_CS_PIN,
        1ull << ADS_CS_PIN
    );
    pio_interrupt_clear(pio, PIO_IRQ_INDEX);
    pio_gpio_init(pio, ADS_CS_PIN);
}

}  // namespace

int main() {
    spi_inst_t *const spi = spi_get_instance(SPI_PORT_INDEX);
    uart_inst_t *const uart = uart_get_instance(UART_PORT_INDEX);
    PIO const pio = pio_get_instance(PIO_INDEX);

    spi_init(spi, SPI_BAUD_RATE);
    spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(SPI_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI_PIN, GPIO_FUNC_SPI);

    uart_init(uart, UART_BAUD_RATE);
    uart_set_format(
        uart,
        UART_DATA_BITS,
        UART_STOP_BITS,
        UART_PARITY_NONE
    );
    uart_set_hw_flow(uart, false, false);
    uart_set_fifo_enabled(uart, true);
    gpio_set_function(
        UART_RX_PIN,
        UART_FUNCSEL_NUM(uart, UART_RX_PIN)
    );
    gpio_set_function(
        UART_TX_PIN,
        UART_FUNCSEL_NUM(uart, UART_TX_PIN)
    );

    gpio_init(ADS_CS_PIN);
    gpio_put(ADS_CS_PIN, true);
    gpio_set_dir(ADS_CS_PIN, GPIO_OUT);

    gpio_init(ADS_RST_PIN);
    gpio_set_dir(ADS_RST_PIN, GPIO_OUT);
    gpio_put(ADS_RST_PIN, 1);

    sleep_ms(ADS_POWER_UP_DELAY_MS);

    uint8_t initialization_rx[ADS_DATA_FRAME_SIZE] = {};
    ads_sio_transfer(
        spi,
        ADS_AUTO_SEQUENCE_FRAME,
        initialization_rx,
        sizeof(ADS_AUTO_SEQUENCE_FRAME)
    );

    uint8_t register_read_rx[sizeof(ADS_AUTO_SEQUENCE_READ_FRAME)] = {};
    ads_sio_transfer(
        spi,
        ADS_AUTO_SEQUENCE_READ_FRAME,
        register_read_rx,
        sizeof(ADS_AUTO_SEQUENCE_READ_FRAME)
    );
    const uint8_t auto_sequence_readback =
        register_read_rx[ADS_PROGRAM_REGISTER_READBACK_INDEX];
    // if (
    //     auto_sequence_readback !=
    //     ADS_AUTO_SEQUENCE_EXPECTED_READBACK
    // ) {
    //     report_spi_write_failure(uart, auto_sequence_readback);
    // } else {
    //     uart_puts(uart, "SPI write succeeded");
    // }

    ads_sio_transfer(
        spi,
        ADS_MANUAL_CHANNEL_7_FRAME,
        initialization_rx,
        sizeof(ADS_MANUAL_CHANNEL_7_FRAME)
    );

    pio_set_gpio_base(pio, PIO_GPIO_BASE);
    const uint pio_offset = pio_add_program(pio, &ads8688_cs_program);
    ads_cs_pio_init(pio, pio_offset);
    pio_sm_set_enabled(pio, PIO_STATE_MACHINE, true);

    while (true) {
        while (!pio_interrupt_get(pio, PIO_IRQ_INDEX)) {
            tight_loop_contents();
        }
        pio_interrupt_clear(pio, PIO_IRQ_INDEX);

        uint8_t adc_rx[ADS_DATA_FRAME_SIZE] = {};
        spi_write_read_blocking(
            spi,
            ADS_NO_OPERATION_FRAME,
            adc_rx,
            ADS_DATA_FRAME_SIZE
        );
        const uint16_t raw_data = static_cast<uint16_t>(
            (static_cast<uint16_t>(adc_rx[ADS_DATA_MSB_INDEX]) << 8) |
            adc_rx[ADS_DATA_LSB_INDEX]
        );
        const int32_t data =
            static_cast<int32_t>(raw_data) -
            static_cast<int32_t>(ADS_STRAIGHT_BINARY_ZERO_CODE);

        uint8_t uart_frame[UART_MAX_FRAME_SIZE] = {};
        const uint uart_frame_size =
            encode_uart_frame(data, uart_frame);
        uart_write_blocking(uart, uart_frame, uart_frame_size);
    }
}
