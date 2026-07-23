#include <hardware/address_mapped.h>
#include <hardware/gpio.h>
#include <hardware/structs/io_bank0.h>
#include <hardware/uart.h>
#include <pico/platform/common.h>
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

#include "definition.h"

auto SPI_PORT = spi0;
auto UART_PORT = uart0;

constexpr uint LUT_SIZE = 64; // rp2350 dma ring buffer必须为2的幂
constexpr uint LUT_RING_BITS = 8; // log2(LUT_SIZE * sizeof(uint32_t)) = 8
// 正弦函数的lut。在线计算来不及，所以打表
// 64个样本点，计算脚本在script/compute_lut.py
alignas(32) static uint32_t SINUSOIDS[1u << LUT_RING_BITS] = {
    0x0000, 0x0330, 0x09ab, 0x069f, 0x0c9d, 0x0f8d, 0x210b, 0x2861, 
    0x29d4, 0x260a, 0x25d2, 0x2cb5, 0x2ec5, 0x2dcd, 0x2f48, 0x2fd6, 
    0x2ffe, 0x2fc0, 0x2f33, 0x2daa, 0x2e17, 0x2779, 0x26f7, 0x2bfd, 
    0x2ac8, 0x23a1, 0x2070, 0x0ed1, 0x05f1, 0x0bda, 0x08ca, 0x0264, 
    0x3d94, 0x3737, 0x3427, 0x3a0d, 0x312d, 0x1fa0, 0x1c5d, 0x153c, 
    0x1401, 0x190a, 0x1885, 0x11ea, 0x1257, 0x10ce, 0x1080, 0x1003, 
    0x102b, 0x10bc, 0x1231, 0x1139, 0x1349, 0x1a2f, 0x19f7, 0x1624, 
    0x179d, 0x1ef6, 0x3071, 0x3361, 0x3962, 0x3656, 0x3ce0, 0x0000
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
    pio_sm_set_clkdiv(pio, sm, 150.0f); // 配置pio频率

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
    uint offset,
    uint data_base
){
    for (auto i : ADC_DATA_PINS){
        pio_gpio_init(pio, i);
    }
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

    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &transmit_program);
    transmit_pio_init(pio, sm, offset, DAC_DATA_BASE);

    uint dma_chan = transmit_dma_init(pio, sm, SINUSOIDS, LUT_RING_BITS);
    dma_channel_start(dma_chan);

    pio_sm_put(pio, sm, 0x3fff);
    while (pio_sm_get_tx_fifo_level(pio, sm) == 0) {
        tight_loop_contents();
    } // 避免pio_sm_exec stall
    pio_sm_exec(pio, sm, pio_encode_pull(false, true)); // pull完之后dma会自动开始工作

    pio_sm_set_enabled(pio, sm, true);

    // ad9744终止休眠
    gpio_put(DAC_PD_PIN, 0);

    gpio_init(46);gpio_set_dir(46, GPIO_OUT);
    while (true) {
        tight_loop_contents(); // 占位，因为逻辑全部由dma和pio负责，主控不需要执行任何操作
    }
}