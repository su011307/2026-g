#include "initialize.hpp"

float sample_clock_divider() {
    constexpr float pio_cycle_per_sample = 2.0f;
    return static_cast<float>(clock_get_hz(clk_sys)) / 
        (ADC_TARGET_SAMPLE_RATE * pio_cycle_per_sample);
}

void pio_init(
    PIO pio,
    uint sm,
    uint offset
){
    for (auto i : ADC_DATA_PINS) {
        pio_gpio_init(pio, i);
        gpio_disable_pulls(i);
    }

    pio_gpio_init(pio, ADC_OTR_PIN);
    pio_gpio_init(pio, ADC_CLK_PIN);
    gpio_disable_pulls(ADC_OTR_PIN);
    gpio_disable_pulls(ADC_CLK_PIN);

    auto cfg = read_program_get_default_config(offset);
    sm_config_set_in_pins(&cfg, ADC_CAPTURE_BASE);
    sm_config_set_in_pin_count(&cfg, ADC_CAPTURE_COUNT);
    sm_config_set_sideset_pins(&cfg, ADC_CLK_PIN);
    sm_config_set_in_shift(&cfg, false, true, ADC_CAPTURE_COUNT);
    sm_config_set_clkdiv(&cfg, sample_clock_divider());

    if (pio_sm_init(pio, sm, offset, &cfg) != PICO_OK) {
        panic("cannot initialize ADC PIO state machine");
    }
    pio_sm_set_consecutive_pindirs(
        pio, sm, ADC_CAPTURE_BASE, ADC_CAPTURE_COUNT, false
    );
    pio_sm_set_consecutive_pindirs(
        pio, sm, ADC_CLK_PIN, 1, true
    );
    pio_sm_set_pins_with_mask64(
        pio, sm, 0, 1ull << ADC_CLK_PIN
    );
}

uint dma_init(
    PIO pio,
    uint sm,
    uint16_t *arena_ptr,
    size_t sample_count
){
    auto chan = dma_claim_unused_channel(true);
    auto cfg = dma_channel_get_default_config(chan);

    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, pio_get_dreq(pio, sm, false));
    channel_config_set_high_priority(&cfg, true);

    dma_channel_configure(chan, &cfg,
        arena_ptr,
        &pio -> rxf[sm],
        sample_count,
        false
    );

    return chan;
}