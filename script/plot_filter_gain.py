#!/usr/bin/env python3
"""Plot the current FIR filter's logarithmic gain from 10 kHz to 1 MHz."""

from pathlib import Path

import matplotlib
import numpy as np
from scipy.signal import freqz

from filter_design import CUTOFF_FREQ, NUM_TAPS, SAMPLE_RATE, coefficients

matplotlib.use("Agg")
from matplotlib import pyplot as plt


MIN_FREQUENCY_HZ = 10_000.0
MAX_FREQUENCY_HZ = 1_000_000.0
FREQUENCY_POINT_COUNT = 4096
MIN_GAIN_DB = -120.0
OUTPUT_PATH = (
    Path(__file__).resolve().parents[1]
    / "res"
    / "image"
    / "fir_gain_10khz_to_1mhz.png"
)


def main() -> None:
    frequencies_hz = np.geomspace(
        MIN_FREQUENCY_HZ,
        MAX_FREQUENCY_HZ,
        FREQUENCY_POINT_COUNT,
    )
    _, response = freqz(
        coefficients,
        worN=frequencies_hz,
        fs=SAMPLE_RATE,
    )
    gain_db = 20.0 * np.log10(
        np.maximum(np.abs(response), np.finfo(np.float64).tiny)
    )
    gain_db = np.maximum(gain_db, MIN_GAIN_DB)

    figure, axes = plt.subplots(figsize=(9, 5))
    axes.semilogx(frequencies_hz, gain_db, linewidth=1.5)
    axes.axvline(
        CUTOFF_FREQ,
        color="tab:red",
        linestyle="--",
        linewidth=1.0,
        label=f"Cutoff: {CUTOFF_FREQ / 1e3:g} kHz",
    )
    axes.set(
        xlim=(MIN_FREQUENCY_HZ, MAX_FREQUENCY_HZ),
        xlabel="Frequency (Hz, logarithmic scale)",
        ylabel="Gain (dB)",
        title=f"FIR frequency response ({NUM_TAPS} taps)",
    )
    axes.grid(True, which="both", linestyle=":", linewidth=0.7)
    axes.legend()
    figure.tight_layout()

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(OUTPUT_PATH, dpi=180)
    plt.close(figure)
    print(OUTPUT_PATH)


if __name__ == "__main__":
    main()
