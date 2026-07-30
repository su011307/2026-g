#pragma once
#include <cstdint>

#include "definition.hpp"

typedef struct {
    float component_frequency[sig::fft::MAX_COMPONENTS];
    float component_amplitude[sig::fft::MAX_COMPONENTS];
    uint32_t component_count;
    float v_pp;
    float v_rms;
} SignalDescription;

void reset_signal_analysis();
SignalDescription analyze_signal(uint16_t *buffer);
void analyze_accumulated_spectrum(
    float spectrum_scale,
    SignalDescription &result
);
