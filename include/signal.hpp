#pragma once
#include <cstdint>

#include "definition.hpp"

struct SignalDescription {
    float component_frequency[sig::fft::MAX_COMPONENTS];
    float component_amplitude[sig::fft::MAX_COMPONENTS];
    uint32_t component_count;
    float v_pp;
    float v_rms;
};

struct SignalPlot {
    float samples[sig::PLOT_SIZE];
    uint16_t sample_count;
};

void reset_signal_analysis();
SignalDescription analyze_signal(uint16_t *buffer);
void analyze_accumulated_spectrum(
    float spectrum_scale,
    SignalDescription &result
);
bool find_wave(const SignalDescription &signal, bool one_wave, SignalPlot &plot);
void resample_spectrum(SignalPlot &plot);
