"""
AT+STREAM telemetry: control loop rate and filter behaviour. Hold the robot still.
"""

import math
import statistics

import pytest

from at_console import parse_stream

# 1000 / IMU_SAMPLE_RATE_MS: the control loop prints one line per sample
LOOP_RATE_HZ = 100
DURATION_S = 3.0


@pytest.fixture(scope="module")
def samples(robot):
    collected = robot.stream(DURATION_S)
    if not collected:
        pytest.fail("AT+STREAM=1 produced no samples: is the IMU delivering data?")
    return collected


def test_loop_rate(samples):
    rate = len(samples) / DURATION_S
    # Lines are dropped rather than delaying the loop, so a low rate can also mean
    # the UART queue overflowed (e.g. a much lower UART_BAUDRATE)
    assert 0.85 * LOOP_RATE_HZ <= rate <= 1.05 * LOOP_RATE_HZ, f"{rate:.1f} samples/s"


def test_values_are_finite(samples):
    bad = [s for s in samples if not all(map(math.isfinite, (s.acc_deg, s.kalman, s.comp)))]
    assert not bad, f"{len(bad)} samples with nan/overflow, first: {bad[0] if bad else None}"


@pytest.mark.parametrize("name", ["kalman", "comp"])
def test_filters_agree_with_accelerometer_at_rest(samples, name):
    offset = statistics.fmean(getattr(s, name) - s.acc_deg for s in samples)
    assert abs(offset) < 2.0, f"{name} is {offset:+.2f}° away from the accelerometer"


def test_complementary_is_smoother_than_accelerometer(samples):
    acc = statistics.pstdev(s.acc_deg for s in samples)
    comp = statistics.pstdev(s.comp for s in samples)
    assert comp <= acc, f"std: complementary {comp:.3f}° vs accelerometer {acc:.3f}°"


def test_stream_stops(robot):
    robot.expect_ok("AT+STREAM=1")
    robot.stop_stream()
    assert not parse_stream(robot.read_for(0.5)), "telemetry still arriving after AT+STREAM=0"
