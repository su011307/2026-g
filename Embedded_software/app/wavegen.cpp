#include <hardware/address_mapped.h>
#include <hardware/gpio.h>
#include <hardware/platform_defs.h>
#include <hardware/regs/addressmap.h>
#include <hardware/structs/clocks.h>
#include <hardware/structs/io_bank0.h>
#include <hardware/uart.h>
#include <pico/platform/common.h>
#include <pico/assert.h>
#include <pico/time.h>
#include <stdio.h>
#include <stdint.h>
#include <pico/types.h>
#include <pico/stdlib.h>
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "driver_st7789.h"
#include "driver_st7789_rp2350.h"
#include "transmit.pio.h"
#include "read.pio.h"

#include "definition.h"

auto SPI_PORT = spi0;
auto UART_PORT = uart0;

constexpr uint LUT_SIZE = 64; // rp2350 dma ring buffer必须为2的幂
constexpr uint LUT_RING_BITS = 8; // log2(LUT_SIZE * sizeof(uint32_t)) = 8
// 正弦函数的lut。在线计算来不及，所以打表
// 64个样本点，计算脚本在script/compute_lut.py
alignas(256) static uint32_t SINUSOIDS[1u << LUT_RING_BITS] = {
    0x0000u, 0x3230u, 0x2F24u, 0x2858u, 0x3F0Cu, 0x153Cu, 0x34D1u, 0x0C45u,
    0x02A5u, 0x0F89u, 0x39A9u, 0x0B0Du, 0x3C9Du, 0x2DADu, 0x227Du, 0x09FDu,
    0x3FFDu, 0x09FDu, 0x227Du, 0x2DADu, 0x3C9Du, 0x0B0Du, 0x39A9u, 0x0F89u,
    0x02A5u, 0x0C45u, 0x34D1u, 0x153Cu, 0x3F0Cu, 0x2858u, 0x2F24u, 0x3230u,
    0x0000u, 0x1DCFu, 0x20DBu, 0x27A7u, 0x10F3u, 0x3AC3u, 0x1B2Eu, 0x07BAu,
    0x025Au, 0x0476u, 0x1656u, 0x08F2u, 0x1362u, 0x2252u, 0x2D82u, 0x0A02u,
    0x1002u, 0x0A02u, 0x2D82u, 0x2252u, 0x1362u, 0x08F2u, 0x1656u, 0x0476u,
    0x025Au, 0x07BAu, 0x1B2Eu, 0x3AC3u, 0x10F3u, 0x27A7u, 0x20DBu, 0x1DCFu
};


void transmit_pio_init(
    PIO pio, 
    uint sm, 
    uint offset, 
    uint pin
){
    for (auto i : DAC_DATA_PINS){
        pio_gpio_init(pio, i);
    }
    pio_gpio_init(pio, DAC_CLK_PIN);

    pio_sm_config c = transmit_program_get_default_config(offset);
    sm_config_set_out_pins(&c, pin, DAC_DATA_COUNT);
    sm_config_set_sideset_pins(&c, DAC_CLK_PIN);
    sm_config_set_out_shift(&c, true, true, DAC_DATA_COUNT);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);  // 8-deep TX
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_clkdiv(pio, sm, 1.0f); // 配置pio频率

    pio_sm_set_consecutive_pindirs(pio, sm, DAC_CLK_PIN, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, DAC_DATA_COUNT, true);
    // 设置所有引脚的初始状态
    pio_sm_set_pins_with_mask(pio, sm, 0u, 1u << DAC_CLK_PIN);
    pio_sm_set_pins_with_mask(pio, sm, 0u, ((1u << DAC_DATA_COUNT) - 1u) << pin);
}

uint transmit_dma_init(
    PIO pio, 
    uint sm,
    const uint32_t * const lut, 
    uint ring_bits
){
    uint chan = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment (&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_dreq(&cfg, pio_get_dreq(pio, sm, true));
    channel_config_set_ring(&cfg, false, ring_bits); // 配置环形缓冲区，每2^ring_bits个字节自动绕回lut首地址
    channel_config_set_high_priority(&cfg, true);

    dma_channel_configure(
        chan, &cfg,
        &pio->txf[sm],
        lut,
        dma_encode_endless_transfer_count(),
        false // 后面手动启用
    );
    return chan;
}

void sample_pio_init(
    PIO pio,
    uint sm,
    uint offset
){
    for (auto i : ADC_DATA_PINS){
        pio_gpio_init(pio, i);
        gpio_disable_pulls(i);
    }
    pio_gpio_init(pio, ADC_OTR_PIN);
    pio_gpio_init(pio, ADC_CLK_PIN);
    gpio_disable_pulls(ADC_OTR_PIN);
    gpio_disable_pulls(ADC_CLK_PIN);

    auto cfg = read_program_get_default_config(offset);
    sm_config_set_in_pins(&cfg, ADC_CAPTURE_BASE);
    sm_config_set_sideset_pins(&cfg, ADC_CLK_PIN);
    // 不可right shift
    sm_config_set_in_shift(&cfg, false, true, ADC_CAPTURE_COUNT);
    sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_RX);

    constexpr float PIO_CYCLES_PER_SAMPLE = 2.0f;
    const float clkdiv = static_cast<float>(clock_get_hz(clk_sys)) / (ADC_TARGET_SAMPLE_RATE * PIO_CYCLES_PER_SAMPLE);
    sm_config_set_clkdiv(&cfg, clkdiv);

    pio_sm_init(pio, sm, offset, &cfg);
    pio_sm_set_consecutive_pindirs(
        pio, sm, ADC_CAPTURE_BASE, ADC_CAPTURE_COUNT, false
    );
    pio_sm_set_consecutive_pindirs(pio, sm, ADC_CLK_PIN, 1, true);
    pio_sm_set_pins_with_mask64(pio, sm, 0u, 1ull << ADC_CLK_PIN);
}

int main(){
    stdio_init_all();

    spi_init(SPI_PORT, 1000*1000);
    uart_init(UART_PORT, 9600);

    gpio_set_function(SPI_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_CS_PIN, GPIO_FUNC_SIO);
    gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_dir(SPI_CS_PIN, GPIO_OUT);
    gpio_put(SPI_CS_PIN, 1);

    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_PORT, UART_RX_PIN));
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_PORT, UART_TX_PIN));

    uart_set_format(UART_PORT, 8, 1, UART_PARITY_NONE);

    // 配置ad9744
    gpio_init(DAC_PD_PIN);
    gpio_set_dir(DAC_PD_PIN, GPIO_OUT);
    gpio_put(DAC_PD_PIN, 1);

    PIO dac_pio = pio0;
    PIO adc_pio = pio1;
    uint dac_sm = 0;
    uint adc_sm = 0;

    pio_set_gpio_base(adc_pio, ADC_PIO_GPIO_BASE);
    uint dac_offset = pio_add_program(dac_pio, &transmit_program);
    uint adc_offset = pio_add_program(adc_pio, &read_program);

    transmit_pio_init(dac_pio, dac_sm, dac_offset, DAC_DATA_BASE);
    sample_pio_init(adc_pio, adc_sm, adc_offset);

    uint dma_chan = transmit_dma_init(dac_pio, dac_sm, SINUSOIDS, LUT_RING_BITS);
    dma_channel_start(dma_chan);

    pio_sm_put(dac_pio, dac_sm, 0x3fff);
    while (pio_sm_get_tx_fifo_level(dac_pio, dac_sm) == 0) {
        tight_loop_contents();
    } // 避免pio_sm_exec stall
    pio_sm_exec(dac_pio, dac_sm, pio_encode_pull(false, true)); // pull完之后dma会自动开始工作

    pio_sm_set_enabled(dac_pio, dac_sm, true);

    // ad9744终止休眠
    gpio_put(DAC_PD_PIN, 0);

    gpio_init(46);gpio_set_dir(46, GPIO_OUT);
    while (true) {
        tight_loop_contents(); // 占位，因为逻辑全部由dma和pio负责，主控不需要执行任何操作
    }
}
