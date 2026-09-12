"""
Direct wheel drive with the PID off. Lift the robot so the wheels spin freely.

Needs --motors. With --interactive you also confirm each wheel direction.
"""

import time

import pytest

pytestmark = pytest.mark.motors

PATTERNS = [
    ((30, 30), "both wheels turning forward"),
    ((-30, -30), "both wheels turning backward"),
    ((30, -30), "left wheel forward, right wheel backward"),
]


@pytest.fixture
def manual_drive(robot):
    robot.expect_ok("AT+PIDOFF")
    robot.expect_ok("AT+ENABLE")
    yield robot
    robot.expect_ok("AT+STOP")


@pytest.mark.parametrize("speeds, description", PATTERNS, ids=[d for _, d in PATTERNS])
def test_wheel_pattern(manual_drive, pytestconfig, operator, speeds, description):
    left, right = speeds
    manual_drive.expect_ok(f"AT+SPEED={left},{right}")
    assert manual_drive.query("SPEED") == f"{left:.1f},{right:.1f}"

    if pytestconfig.getoption("--interactive"):
        seen = operator.confirm(f"Do you see {description}?")
    else:
        time.sleep(1.0)
        seen = True

    status = manual_drive.query("STATUS")
    manual_drive.expect_ok("AT+SPEED=0,0")
    assert status.startswith("ENABLED"), f"motors cut off while driving ({status}): IMU stall?"
    assert seen, f"expected {description}: check motor wiring or port mapping"


def test_stop_zeroes_reported_speeds(manual_drive):
    manual_drive.expect_ok("AT+SPEED=20,20")
    manual_drive.expect_ok("AT+STOP")
    assert manual_drive.query("SPEED") == "0.0,0.0"
