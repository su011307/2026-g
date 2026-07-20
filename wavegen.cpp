#include <stdio.h>
#include <stdint.h>
#include <pico/types.h>
#include <pico/stdlib.h>
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "driver_st7789.h"
#include "transmit.pio.h"

// lcd屏幕的引脚
#define SPI_PORT spi0
constexpr uint PIN_MISO = 16;
constexpr uint PIN_CS = 17;
constexpr uint PIN_SCK = 18;
constexpr uint PIN_MOSI = 19;
constexpr uint PIN_LCD_SCL = PIN_SCK;
constexpr uint PIN_LCD_SDA = PIN_MOSI;
constexpr uint PIN_LCD_RST = 20;
constexpr uint PIN_LCD_DC = 21;
constexpr uint PIN_LCD_CS = 22;
constexpr uint PIN_LCD_BL = 23;

// ad9744的控制引脚
constexpr uint DAC_CLK_PIN    = 0;
constexpr uint DAC_PD_PIN     = 1; // sleep pin，高电平有效
constexpr uint DAC_DATA_BASE  = 2;
constexpr uint DAC_DATA_COUNT = 14;
constexpr uint DAC_PINS[] = {
    14, 15, 12, 13,
    10, 11, 8, 9,
    6, 7, 4, 5,
    2, 3
}; // ad9744的数据引脚，最高位为符号位

// ---------------------------------------------------------------------------
// 正弦查找表 (静态, 离线算好后填入)。
//   每个 entry 是 32-bit word: out pins, 14 只取低 14 位, 高 18 位由 PIO 自动丢弃。
//
// "自然 14 位 DAC 代码" (bit k = AD9744 DBk) 必须按下式重排, 才能匹配这块
// PCB 上 GPIO2..15 的物理走线 (实质是 14 位值上"每 2 位一组, 组与组之间倒序"):
//
//   r |= ((code >> 12) & 0x3u)
//     | (((code >> 10) & 0x3u) <<  2)
//     | (((code >>  8) & 0x3u) <<  4)
//     | (((code >>  6) & 0x3u) <<  6)
//     | (((code >>  4) & 0x3u) <<  8)
//     | (((code >>  2) & 0x3u) << 10)
//     | (((code >>  0) & 0x3u) << 12);
//
// 13 位补码编码:
//   -4096 -> 0x1000,  -1 -> 0x1FFF,  0 -> 0x0000,  +4095 -> 0x0FFF
// (若 reference.pdf 把 AD9744 MODE 接到 DCOM=straight binary, 则先把
//  每个 code += 0x2000 偏置半量程, 再走上面的重排公式。)
// ---------------------------------------------------------------------------
constexpr uint LUT_SIZE      = 32;
constexpr uint LUT_RING_BITS = 7; // log2(LUT_SIZE * sizeof(uint32_t)) = log2(128) = 7

// 正弦函数的lut。在线计算来不及，所以打表
alignas(32) static uint32_t SINUSOIDS[LUT_SIZE] = {
    0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u,
    0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u,
    0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u,
    0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u,
};

void transmit_pio_init(
    PIO pio, 
    uint sm, 
    uint offset, 
    uint data_base
){
    for (auto i : DAC_PINS){
        pio_gpio_init(pio, i);
    }
    pio_gpio_init(pio, DAC_CLK_PIN);

    pio_sm_config c = transmit_program_get_default_config(offset);
    sm_config_set_out_pins(&c, data_base,    DAC_DATA_COUNT);
    sm_config_set_sideset_pins(&c, DAC_CLK_PIN);
    sm_config_set_out_shift(&c, true, true, DAC_DATA_COUNT);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);  // 8-deep TX
    pio_sm_init(pio, sm, offset, &c);

    pio_sm_set_consecutive_pindirs(pio, sm, DAC_CLK_PIN, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, data_base, DAC_DATA_COUNT, true);
    // 设置所有引脚的初始状态
    pio_sm_set_pins_with_mask(pio, sm, 0u, 1u << DAC_CLK_PIN);
    pio_sm_set_pins_with_mask(pio, sm, 0u, ((1u << DAC_DATA_COUNT) - 1u) << data_base);
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

int main(){
    stdio_init_all();

    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    // 配置ad9744
    gpio_init(DAC_PD_PIN);
    gpio_set_dir(DAC_PD_PIN, GPIO_OUT);
    gpio_put(DAC_PD_PIN, 1);

    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &transmit_program);
    transmit_pio_init(pio, sm, offset, DAC_DATA_BASE);
    pio_sm_set_clkdiv(pio, sm, 75.0f); // 配置pio频率

    uint dma_chan = transmit_dma_init(pio, sm, SINUSOIDS, LUT_RING_BITS);
    dma_channel_start(dma_chan);

    while (pio_sm_get_tx_fifo_level(pio, sm) == 0) {
        tight_loop_contents();
    } // 避免pio_sm_exec stall
    pio_sm_exec(pio, sm, pio_encode_pull(false, true)); // pull完之后dma会自动开始工作

    pio_sm_set_enabled(pio, sm, true);

    // ad9744终止休眠
    gpio_put(DAC_PD_PIN, 0);

    while (true) {
        tight_loop_contents(); // 占位，因为逻辑全部由dma和pio负责，主控不需要执行任何操作
    }
}