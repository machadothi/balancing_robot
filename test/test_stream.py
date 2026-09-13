"""
USB telemetry (AT+STREAM): completeness, timing and filter behaviour. Hold the robot still.
"""

import math
import statistics

import pytest

from at_console import parse_stream

pytestmark = pytest.mark.usb_only

# IMU_SAMPLE_RATE_MS: the control loop submits one record per sample
SAMPLE_PERIOD_MS = 10
LOOP_RATE_HZ = 1000 / SAMPLE_PERIOD_MS
DURATION_S = 5.0

FLOAT_FIELDS = ("acc_deg", "kalman", "comp", "tilt", "p", "i", "d", "out")


@pytest.fixture(scope="module")
def records(robot):
    collected = robot.stream(DURATION_S)
    if len(collected) < 2:
        pytest.fail(f"AT+STREAM=1 produced {len(collected)} records: is the IMU delivering data?")
    return collected


def test_no_lost_records(records):
    gaps = [(a.seq, b.seq) for a, b in zip(records, records[1:]) if b.seq != a.seq + 1]
    assert not gaps, (
        f"{len(gaps)} gaps in {len(records)} records, first {gaps[0][0]} -> {gaps[0][1]}; "
        f"firmware drop counter {records[0].drops} -> {records[-1].drops} "
        "(unchanged means the loss was on the link or the host)"
    )


def test_firmware_dropped_nothing(records):
    assert records[-1].drops == records[0].drops, \
        f"telemetry queue overflowed: drops {records[0].drops} -> {records[-1].drops}"


def test_loop_rate(records):
    rate = len(records) / DURATION_S
    assert 0.9 * LOOP_RATE_HZ <= rate <= 1.05 * LOOP_RATE_HZ, f"{rate:.1f} records/s"


def test_sample_period(records):
    periods = [b.t - a.t for a, b in zip(records, records[1:])]
    assert statistics.fmean(periods) == pytest.approx(SAMPLE_PERIOD_MS, rel=0.02)
    assert SAMPLE_PERIOD_MS - 2 <= min(periods) and max(periods) <= SAMPLE_PERIOD_MS + 2, \
        f"period jitter {min(periods)}..{max(periods)} ms"


def test_values_are_finite(records):
    bad = [r for r in records if not all(math.isfinite(getattr(r, f)) for f in FLOAT_FIELDS)]
    assert not bad, f"{len(bad)} records with nan/overflow, first: {bad[0] if bad else None}"


@pytest.mark.parametrize("name", ["kalman", "comp"])
def test_filters_agree_with_accelerometer_at_rest(records, name):
    offset = statistics.fmean(getattr(r, name) - r.acc_deg for r in records)
    assert abs(offset) < 2.0, f"{name} is {offset:+.2f}° away from the accelerometer"


def test_complementary_is_smoother_than_accelerometer(records):
    acc = statistics.pstdev(r.acc_deg for r in records)
    comp = statistics.pstdev(r.comp for r in records)
    assert comp <= acc, f"std: complementary {comp:.3f}° vs accelerometer {acc:.3f}°"


def test_stream_stops(robot):
    robot.expect_ok("AT+STREAM=1")
    robot.stop_stream()
    assert not parse_stream(robot.read_for(0.5)), "telemetry still arriving after AT+STREAM=0"
