"""
IMU data over the AT console. Hold the robot still (any orientation).
"""

import math
import statistics
import time

import pytest

SAMPLES = 10


def read_axes(robot, sensor):
    return [robot.query_float(f"{sensor}_{axis}") for axis in "XYZ"]


def averaged_axes(robot, sensor, samples=SAMPLES):
    readings = [read_axes(robot, sensor) for _ in range(samples)]
    return [statistics.fmean(axis) for axis in zip(*readings)]


@pytest.fixture(scope="module", autouse=True)
def imu_ready(robot):
    """Fail early with a clear message if the IMU never delivers data."""
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if any(read_axes(robot, "ACC")):
            return
        time.sleep(0.2)
    pytest.fail("accelerometer still reads 0,0,0: the IMU is not initialised or not wired")


def test_gravity_magnitude_is_one_g(robot):
    magnitude = math.dist((0.0, 0.0, 0.0), averaged_axes(robot, "ACC"))
    assert 0.9 <= magnitude <= 1.1, f"|a| = {magnitude:.3f} g, expected about 1 g at rest"


def test_gyro_is_quiet_at_rest(robot):
    gx, gy, gz = averaged_axes(robot, "GYRO")
    # X includes GYRO_CALIBRATION_OFFSET; Y and Z are not calibrated
    assert abs(gx) < 1.0, f"gyro X reads {gx:.3f} °/s at rest: recalibrate GYRO_CALIBRATION_OFFSET"
    assert abs(gy) < 5.0 and abs(gz) < 5.0, f"gyro Y/Z at rest: {gy:.3f}, {gz:.3f} °/s"


def test_readings_are_live(robot):
    values = {robot.query("GYRO_X") for _ in range(SAMPLES)}
    assert len(values) > 1, f"GYRO_X returned {values} {SAMPLES} times: data is frozen"


def test_angle_follows_accelerometer(robot):
    ax, ay, _ = averaged_axes(robot, "ACC")
    expected = math.degrees(math.atan2(ay, -ax)) - 90.0    # calc_angle_from_accel() - 90
    angle = statistics.fmean(robot.query_float("ANGLE") for _ in range(SAMPLES))
    error = (angle - expected + 180.0) % 360.0 - 180.0
    assert abs(error) < 3.0, f"ANGLE {angle:.2f}° but accelerometer gives {expected:.2f}°"
