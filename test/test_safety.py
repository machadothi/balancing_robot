"""
Safety behaviour: no spurious resets, tilt and IMU-loss cut-offs, balancing.
"""

import time

import pytest

from at_console import BANNER


def test_no_reset_while_idle(robot):
    received = robot.read_for(5.0)
    assert BANNER not in received, "startup banner printed again: the board reset (watchdog?)"


def test_no_reset_under_telemetry_load(robot):
    robot.expect_ok("AT+STREAM=1")
    try:
        received = robot.read_for(5.0)
    finally:
        robot.stop_stream()
    assert BANNER not in received, "startup banner printed while streaming: the board reset"


def test_enable_and_stop_without_pid(robot):
    """With the PID off, ENABLE only leaves standby: nothing moves. Needs a working IMU."""
    robot.expect_ok("AT+PIDOFF")
    robot.expect_ok("AT+ENABLE")
    time.sleep(0.2)
    assert robot.query("STATUS").startswith("ENABLED"), \
        "motors disabled themselves within 200 ms: no IMU samples (stall cut-off)"
    robot.expect_ok("AT+STOP")
    assert robot.query("STATUS").startswith("DISABLED")


@pytest.mark.interactive
def test_tilt_cutoff(robot, operator):
    operator.prompt("Lay the robot on its side (more than 45° from upright) and keep it there.")
    robot.expect_ok("AT+ENABLE")
    time.sleep(0.2)
    assert robot.query("STATUS").startswith("DISABLED"), "balancing stayed enabled beyond 45°"


@pytest.mark.interactive
def test_imu_loss_stops_motors(robot, operator):
    if not operator.confirm("Can you disconnect the IMU (e.g. its SDA wire) while the robot "
                            "is held? Answer no for the F407 board's on-board IMU."):
        pytest.skip("IMU cannot be disconnected on this setup")
    robot.expect_ok("AT+PIDOFF")
    robot.expect_ok("AT+ENABLE")
    assert robot.query("STATUS").startswith("ENABLED")

    operator.prompt("Disconnect the IMU now.")
    time.sleep(0.2)
    status = robot.query("STATUS")
    operator.prompt("Reconnect the IMU. If it was unpowered, reset the board before continuing.")
    assert status.startswith("DISABLED"), f"motors stayed enabled without IMU samples: {status}"


@pytest.mark.motors
@pytest.mark.interactive
def test_balances_when_released(robot, operator):
    operator.prompt("Hold the robot upright on the floor, ready to catch it. It starts "
                    "balancing when you press Enter: let go gently.")
    robot.expect_ok("AT+ENABLE")
    time.sleep(3.0)
    status = robot.query("STATUS")
    robot.expect_ok("AT+STOP")
    assert status == "ENABLED,BALANCED", f"after 3 s of balancing: {status}"
