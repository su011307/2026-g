#!/usr/bin/env python3
"""Generate all waveform text files listed in test_doc.pdf.

Output format follows sample_test_wave.txt:
    first column  = timestamp in seconds
    second column = instantaneous voltage in volts
No header is written.

Dependencies:
    numpy
    scipy

Default design choices:
- Record length: 2 ms, so the FFT-bin spacing is exactly 500 Hz.
- Sample rate: 20 MHz, so the 3.3 MHz interference case is below Nyquist.
- Every sinusoid starts at zero phase, matching the multi-tone peak-to-peak
  values printed in test_doc.pdf.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
from scipy.fft import rfft, rfftfreq

FREQUENCY_RESOLUTION_HZ = 500.0
OUTPUT_ROOT = Path("~/develop/wavegen/res/data/test_data").expanduser()
SAMPLE_RATE_HZ = 65_000_000.0
DURATION_S = 1.0 / FREQUENCY_RESOLUTION_HZ  # 2 ms
CASE_IDS: tuple[str, ...] = ()  # Empty means all cases; e.g. ("G1-10", "G3-04")
OVERWRITE = False


@dataclass(frozen=True)
class Tone:
    frequency_hz: float
    component_upp_mv: float
    harmonic: int
    is_interference: bool = False

    @property
    def peak_v(self) -> float:
        """Positive-frequency spectrum amplitude / sinusoidal peak voltage."""
        return self.component_upp_mv * 1e-3 / 2.0


@dataclass(frozen=True)
class TestCase:
    group: int
    index: int
    description: str
    signal_tones: tuple[Tone, ...]
    expected_signal_upp_mv: float
    expected_signal_urms_mv: float
    interference_tones: tuple[Tone, ...] = ()

    @property
    def case_id(self) -> str:
        return f"g{self.group}_{self.index:02d}"

    @property
    def all_tones(self) -> tuple[Tone, ...]:
        return self.signal_tones + self.interference_tones

    @property
    def base_frequency_hz(self) -> float:
        return min(t.frequency_hz for t in self.signal_tones)

    @property
    def filename(self) -> str:
        if len(self.signal_tones) == 1:
            tone = self.signal_tones[0]
            stem = (
                f"{self.case_id}_single_"
                f"{format_frequency_for_filename(tone.frequency_hz)}_"
                f"{tone.component_upp_mv:g}mVpp"
            )
        else:
            stem = f"{self.case_id}_multitone_{format_frequency_for_filename(self.base_frequency_hz)}_base"
        if self.interference_tones:
            noise = self.interference_tones[0]
            stem += f"_plus_{format_frequency_for_filename(noise.frequency_hz)}_{noise.component_upp_mv:g}mVpp_interference"
        return stem + ".txt"


def tone(freq_hz: float, upp_mv: float, harmonic: int, *, interference: bool = False) -> Tone:
    return Tone(freq_hz, upp_mv, harmonic, interference)


def single_case(
    group: int,
    index: int,
    freq_hz: float,
    upp_mv: float,
    expected_urms_mv: float,
    *,
    interference: Tone | None = None,
) -> TestCase:
    return TestCase(
        group=group,
        index=index,
        description="single-tone" if interference is None else "single-tone + interference",
        signal_tones=(tone(freq_hz, upp_mv, 1),),
        expected_signal_upp_mv=upp_mv,
        expected_signal_urms_mv=expected_urms_mv,
        interference_tones=() if interference is None else (interference,),
    )


# All component U_pp values below come from test_doc.pdf.
# A sinusoid's positive-frequency spectrum amplitude is U_pp / 2.
TEST_CASES: tuple[TestCase, ...] = (
    # Requirement 1: ua, 10 kHz to 200 kHz, total U_pp 100 mV to 250 mV.
    single_case(1, 1, 10_000, 100, 35.4),
    single_case(1, 2, 10_000, 250, 88.4),
    single_case(1, 3, 10_500, 100, 35.4),
    single_case(1, 4, 10_500, 250, 88.4),
    single_case(1, 5, 200_000, 100, 35.4),
    single_case(1, 6, 200_000, 250, 88.4),
    single_case(1, 7, 199_500, 100, 35.4),
    single_case(1, 8, 199_500, 250, 88.4),
    single_case(1, 9, 137_500, 167, 59.04),
    TestCase(
        1,
        10,
        "fundamental + 3rd + 4th harmonic",
        (
            tone(10_500, 124, 1),
            tone(31_500, 18, 3),
            tone(42_000, 111, 4),
        ),
        218.67,
        59.18,
    ),
    TestCase(
        1,
        11,
        "fundamental + 2nd + 7th harmonic",
        (
            tone(28_500, 63, 1),
            tone(57_000, 72, 2),
            # test_doc labels this Uppa4 in one place; the frequency is fa7.
            tone(199_500, 121, 7),
        ),
        234.37,
        54.54,
    ),
    # Requirement 2: ub, 10 kHz to 500 kHz, total U_pp 50 mV to 250 mV.
    single_case(2, 1, 10_000, 50, 17.7),
    single_case(2, 2, 10_500, 50, 17.7),
    single_case(2, 3, 500_000, 50, 17.7),
    single_case(2, 4, 500_000, 250, 88.4),
    single_case(2, 5, 499_500, 50, 17.7),
    single_case(2, 6, 499_500, 250, 88.4),
    single_case(2, 7, 349_500, 73, 25.81),
    TestCase(
        2,
        8,
        "fundamental + 5th + 6th harmonic",
        (
            tone(27_500, 50, 1),
            tone(137_500, 116, 5),
            tone(165_000, 77, 6),
        ),
        205.50,
        52.30,
    ),
    TestCase(
        2,
        9,
        "fundamental + 9th + 16th harmonic",
        (
            tone(31_000, 19, 1),
            tone(279_000, 11, 9),
            tone(496_000, 31, 16),
        ),
        57.25,
        13.43,
    ),
    # Requirement 3: generated input is ub + uJ; expected measurement excludes uJ.
    single_case(
        3,
        1,
        500_000,
        50,
        17.7,
        interference=tone(1_000_000, 200, 2, interference=True),
    ),
    TestCase(
        3,
        2,
        "fundamental + 2nd harmonic + interference",
        (
            tone(128_000, 50, 1),
            tone(256_000, 30, 2),
        ),
        69.37,
        20.62,
        (tone(1_024_000, 200, 8, interference=True),),
    ),
    TestCase(
        3,
        3,
        "fundamental + 9th + 16th harmonic + interference",
        (
            tone(31_000, 19, 1),
            tone(279_000, 11, 9),
            tone(496_000, 31, 16),
        ),
        57.25,
        13.43,
        (tone(1_023_000, 200, 33, interference=True),),
    ),
    TestCase(
        3,
        4,
        "fundamental + 2nd + 3rd harmonic + interference",
        (
            tone(165_000, 19, 1),
            tone(330_000, 11, 2),
            tone(495_000, 31, 3),
        ),
        51.18,
        13.43,
        # test_doc calls this f20=3.3 MHz but also contains an inconsistent Uppb33 label.
        (tone(3_300_000, 200, 20, interference=True),),
    ),
)


def format_frequency_for_filename(freq_hz: float) -> str:
    if freq_hz >= 1e6:
        return f"{freq_hz / 1e6:g}MHz"
    return f"{freq_hz / 1e3:g}kHz"


def format_frequency(freq_hz: float) -> str:
    if freq_hz >= 1e6:
        return f"{freq_hz / 1e6:.6g} MHz"
    return f"{freq_hz / 1e3:.6g} kHz"


def synthesize(timestamps_s: np.ndarray, tones: Sequence[Tone]) -> np.ndarray:
    voltage_v = np.zeros_like(timestamps_s, dtype=np.float64)
    for item in tones:
        voltage_v += item.peak_v * np.sin(2.0 * np.pi * item.frequency_hz * timestamps_s)
    return voltage_v


def true_rms_mv(tones: Sequence[Tone]) -> float:
    # Orthogonal harmonics over a complete base period: RMS^2 is the sum of each tone's RMS^2.
    return math.sqrt(sum((item.peak_v / math.sqrt(2.0)) ** 2 for item in tones)) * 1e3


def fft_peak_amplitude_v(
    voltage_v: np.ndarray,
    sample_rate_hz: float,
    target_frequency_hz: float,
) -> tuple[float, float]:
    spectrum = rfft(voltage_v)
    frequencies = rfftfreq(voltage_v.size, d=1.0 / sample_rate_hz)
    bin_index = int(np.argmin(np.abs(frequencies - target_frequency_hz)))
    peak_amplitude_v = 2.0 * abs(spectrum[bin_index]) / voltage_v.size
    return float(frequencies[bin_index]), float(peak_amplitude_v)


def validate_case_definition(case: TestCase) -> None:
    base = case.base_frequency_hz
    all_signal_frequencies = [item.frequency_hz for item in case.signal_tones]

    for item in case.signal_tones:
        ratio = item.frequency_hz / base
        if not math.isclose(ratio, item.harmonic, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError(
                f"{case.case_id}: {item.frequency_hz:g} Hz is not harmonic {item.harmonic} "
                f"of base {base:g} Hz"
            )
        bins = item.frequency_hz / FREQUENCY_RESOLUTION_HZ
        if not math.isclose(bins, round(bins), rel_tol=0.0, abs_tol=1e-12):
            raise ValueError(f"{case.case_id}: {item.frequency_hz:g} Hz is not on the 500 Hz grid")

    calculated_rms = true_rms_mv(case.signal_tones)
    if abs(calculated_rms - case.expected_signal_urms_mv) > 0.06:
        raise ValueError(
            f"{case.case_id}: calculated RMS {calculated_rms:.4f} mV disagrees with "
            f"test_doc value {case.expected_signal_urms_mv:.4f} mV"
        )

    if case.group == 1:
        if not all(10_000 <= f <= 200_000 for f in all_signal_frequencies):
            raise ValueError(f"{case.case_id}: requirement-1 signal frequency out of range")
        if not 100 <= case.expected_signal_upp_mv <= 250:
            raise ValueError(f"{case.case_id}: requirement-1 U_pp out of range")
    elif case.group in (2, 3):
        if not all(10_000 <= f <= 500_000 for f in all_signal_frequencies):
            raise ValueError(f"{case.case_id}: requirement-{case.group} signal frequency out of range")
        if not 50 <= case.expected_signal_upp_mv <= 250:
            raise ValueError(f"{case.case_id}: requirement-{case.group} U_pp out of range")
    else:
        raise ValueError(f"Unknown group {case.group}")

    if case.group == 3:
        if len(case.interference_tones) != 1:
            raise ValueError(f"{case.case_id}: requirement 3 needs exactly one interference tone")
        noise = case.interference_tones[0]
        if not math.isclose(noise.component_upp_mv, 200.0, abs_tol=1e-12):
            raise ValueError(f"{case.case_id}: interference must be 200 mVpp")
        if noise.frequency_hz < 1_000_000:
            raise ValueError(f"{case.case_id}: interference must be at least 1 MHz")
    elif case.interference_tones:
        raise ValueError(f"{case.case_id}: interference is only valid in requirement 3")


def validate_global_settings(sample_rate_hz: float, duration_s: float) -> int:
    if sample_rate_hz <= 0 or duration_s <= 0:
        raise ValueError("Sample rate and duration must be positive")

    number_of_samples_float = sample_rate_hz * duration_s
    number_of_samples = int(round(number_of_samples_float))
    if not math.isclose(number_of_samples_float, number_of_samples, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError("sample_rate * duration must be an integer number of samples")

    actual_bin_hz = sample_rate_hz / number_of_samples
    if actual_bin_hz > FREQUENCY_RESOLUTION_HZ + 1e-9:
        raise ValueError(
            f"FFT-bin spacing is {actual_bin_hz:g} Hz; it must be <= {FREQUENCY_RESOLUTION_HZ:g} Hz"
        )

    highest_frequency = max(t.frequency_hz for case in TEST_CASES for t in case.all_tones)
    if sample_rate_hz / 2.0 <= highest_frequency:
        raise ValueError(
            f"Nyquist frequency {sample_rate_hz / 2:g} Hz must exceed the highest tone "
            f"{highest_frequency:g} Hz"
        )
    return number_of_samples


def select_cases(case_ids: Iterable[str] | None) -> tuple[TestCase, ...]:
    if not case_ids:
        return TEST_CASES
    requested = {item.upper() for item in case_ids}
    result = tuple(case for case in TEST_CASES if case.case_id.upper() in requested)
    missing = requested - {case.case_id.upper() for case in result}
    if missing:
        valid = ", ".join(case.case_id for case in TEST_CASES)
        raise ValueError(f"Unknown case(s): {', '.join(sorted(missing))}. Valid case IDs: {valid}")
    return result


def print_case_report(
    case: TestCase,
    output_path: Path,
    signal_voltage_v: np.ndarray,
    input_voltage_v: np.ndarray,
    sample_rate_hz: float,
) -> None:
    sampled_signal_upp_mv = np.ptp(signal_voltage_v) * 1e3
    sampled_signal_urms_mv = np.sqrt(np.mean(np.square(signal_voltage_v))) * 1e3

    print(f"[{case.case_id}] {case.description}")
    print(f"  file: {output_path}")
    print(
        "  expected clean-signal time-domain values: "
        f"Upp={case.expected_signal_upp_mv:.2f} mV, "
        f"Urms={case.expected_signal_urms_mv:.2f} mV, "
        f"f1={format_frequency(case.base_frequency_hz)}"
    )
    print(
        "  generated-sample check: "
        f"Upp={sampled_signal_upp_mv:.5f} mV, "
        f"Urms={sampled_signal_urms_mv:.5f} mV"
    )
    print("  expected displayed positive-frequency spectrum peaks (peak amplitude, not Vpp):")
    for item in case.signal_tones:
        fft_frequency_hz, fft_amplitude_v = fft_peak_amplitude_v(
            input_voltage_v, sample_rate_hz, item.frequency_hz
        )
        print(
            f"    h{item.harmonic:<2d} {format_frequency(item.frequency_hz):>12s}: "
            f"{item.peak_v * 1e3:8.3f} mVpk "
            f"[FFT check {format_frequency(fft_frequency_hz)}, {fft_amplitude_v * 1e3:.3f} mVpk]"
        )

    if case.interference_tones:
        print("  generated raw-input interference peak (must be rejected for requirement 3):")
        for item in case.interference_tones:
            fft_frequency_hz, fft_amplitude_v = fft_peak_amplitude_v(
                input_voltage_v, sample_rate_hz, item.frequency_hz
            )
            print(
                f"    {format_frequency(item.frequency_hz):>12s}: "
                f"{item.peak_v * 1e3:8.3f} mVpk "
                f"[FFT check {format_frequency(fft_frequency_hz)}, {fft_amplitude_v * 1e3:.3f} mVpk]"
            )
    print()


def output_subdirectory(root: Path, group: int) -> Path:
    names = {
        1: "requirement_1_ua",
        2: "requirement_2_ub",
        3: "requirement_3_ub_plus_interference",
    }
    return root / names[group]


def generate(
    output_root: Path,
    sample_rate_hz: float,
    duration_s: float,
    cases: Sequence[TestCase],
    overwrite: bool,
) -> None:
    number_of_samples = validate_global_settings(sample_rate_hz, duration_s)
    for case in cases:
        validate_case_definition(case)

    timestamps_s = np.arange(number_of_samples, dtype=np.float64) / sample_rate_hz
    actual_bin_hz = sample_rate_hz / number_of_samples

    print("Waveform generation settings")
    print(f"  sample rate       : {sample_rate_hz:g} Sa/s")
    print(f"  sample interval   : {1.0 / sample_rate_hz:.9e} s")
    print(f"  record duration   : {duration_s:.9e} s")
    print(f"  samples per file  : {number_of_samples}")
    print(f"  FFT-bin spacing   : {actual_bin_hz:g} Hz")
    print("  phase / DC offset : all sine phases = 0 rad; DC offset = 0 V")
    print("  voltage convention: component spectrum amplitude = component U_pp / 2")
    print("  50-ohm note       : files contain the actual input voltage; no x2 or /2 load correction is applied")
    print()

    for case in cases:
        directory = output_subdirectory(output_root, case.group)
        directory.mkdir(parents=True, exist_ok=True)
        output_path = directory / case.filename
        if output_path.exists() and not overwrite:
            raise FileExistsError(
                f"Refusing to overwrite {output_path}; set OVERWRITE = True"
            )

        signal_voltage_v = synthesize(timestamps_s, case.signal_tones)
        input_voltage_v = signal_voltage_v + synthesize(timestamps_s, case.interference_tones)

        # Match sample_test_wave.txt: two headerless columns in scientific notation.
        np.savetxt(
            output_path,
            np.column_stack((timestamps_s, input_voltage_v)),
            fmt="   %.7e   %.7e",
        )

        print_case_report(
            case,
            output_path,
            signal_voltage_v,
            input_voltage_v,
            sample_rate_hz,
        )


def main() -> int:
    try:
        cases = select_cases(CASE_IDS)
        generate(
            output_root=OUTPUT_ROOT,
            sample_rate_hz=SAMPLE_RATE_HZ,
            duration_s=DURATION_S,
            cases=cases,
            overwrite=OVERWRITE,
        )
    except (ValueError, FileExistsError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
