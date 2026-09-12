"""
AT console protocol: replies, error codes, parameter validation and formatting.

Safe in any position: nothing here drives the motors.
"""

import re

import pytest

# ROBOT_DEFAULT_KP/KI/KD in src/robot/robot.c, as printed by AT+KP? (4 decimals)
DEFAULT_GAINS = {"KP": "25.0000", "KI": "0.5000", "KD": "0.8000"}


def test_at_replies_ok(robot):
    assert robot.send("AT").ok


def test_version_format(robot):
    assert re.fullmatch(r"\d+\.\d+\.\d+", robot.query("VERSION"))


def test_commands_are_case_insensitive(robot):
    assert robot.send("at+version?").data == robot.query("VERSION")


def test_status_format(robot):
    assert re.fullmatch(r"(ENABLED|DISABLED),(BALANCED|UNBALANCED)", robot.query("STATUS"))


def test_stop_leaves_motors_disabled(robot):
    robot.expect_ok("AT+STOP")
    assert robot.query("STATUS").startswith("DISABLED")


@pytest.mark.parametrize("command, code", [
    # 1: known command that failed
    ("AT+SAVE", 1),
    ("AT+LOAD", 1),
    # 2: unknown command
    ("AT+NOSUCH?", 2),
    ("AT+NOSUCH", 2),
    ("AT+NOSUCH=1", 2),
    # 3: invalid parameter: missing, not a number, not finite
    ("AT+KP=", 3),
    ("AT+KP=abc", 3),
    ("AT+KP=1.5x", 3),
    ("AT+KP=nan", 3),
    ("AT+KP=inf", 3),
    ("AT+SPEED=10", 3),
    ("AT+SPEED=abc,10", 3),
    ("AT+SPEED=10,nan", 3),
    # 4: out of range
    ("AT+KP=-1", 4),
    ("AT+TURN=100.5", 4),
    ("AT+VELOCITY=-101", 4),
    ("AT+SPEED=0,101", 4),
    ("AT+STREAM=2", 4),
])
def test_error_codes(robot, command, code):
    response = robot.send(command)
    assert response.status == "ERROR" and response.error == code, f"{command} -> {response}"


def test_rejected_value_is_not_applied(robot):
    before = robot.query("KP")
    robot.send("AT+KP=nan")
    assert robot.query("KP") == before


@pytest.mark.parametrize("line, message", [
    ("HELLO", "must start with AT"),
    ("AT+", "Invalid syntax"),
])
def test_malformed_lines(robot, line, message):
    response = robot.send(line)
    assert response.status == "ERROR" and message in response.text, f"{line} -> {response}"


@pytest.mark.parametrize("name", DEFAULT_GAINS)
def test_gain_round_trip(robot, name):
    robot.expect_ok(f"AT+{name}=12.3456")
    assert robot.query(name) == "12.3456"


def test_default_restores_startup_gains(robot):
    for name in DEFAULT_GAINS:
        robot.expect_ok(f"AT+{name}=1")
    robot.expect_ok("AT+DEFAULT")
    assert {name: robot.query(name) for name in DEFAULT_GAINS} == DEFAULT_GAINS


@pytest.mark.parametrize("sent, shown", [
    ("0", "0.00"),
    ("-0.5", "-0.50"),         # sign of values between -1 and 0
    ("-0.004", "0.00"),        # rounds to zero, no "-0.00"
    ("1.996", "2.00"),         # carry into the integer part
    ("-99.999", "-100.00"),
])
def test_fixed_point_formatting(robot, sent, shown):
    robot.expect_ok(f"AT+TURN={sent}")
    assert robot.query("TURN") == shown


def test_target_is_stored(robot):
    robot.expect_ok("AT+TARGET=-42.5")
    assert robot.query("TARGET") == "-42.50"
