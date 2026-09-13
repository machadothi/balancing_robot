"""
Hardware-in-the-loop tests over the robot's AT console. See test/README.md.
"""

import os

import pytest
import serial

from at_console import AtConsole


def pytest_addoption(parser):
    group = parser.getgroup("robot", "balancing robot hardware tests")
    group.addoption("--port", default=os.environ.get("ROBOT_PORT", "/dev/ttyUSB0"),
                    help="console serial port (env ROBOT_PORT, default %(default)s)")
    group.addoption("--baud", type=int, default=int(os.environ.get("ROBOT_BAUD", "921600")),
                    help="firmware UART_BAUDRATE (env ROBOT_BAUD, default %(default)s)")
    group.addoption("--assert-dtr-rts", action="store_true",
                    help="assert DTR/RTS on open; they are released by default so "
                         "auto-reset circuits leave the board running")
    group.addoption("--motors", action="store_true",
                    help="run tests that drive the wheels (lift the robot first)")
    group.addoption("--interactive", action="store_true",
                    help="run tests that ask you to move the robot or unplug the IMU")
    group.addoption("--bluetooth", action="store_true",
                    help="--port is the Bluetooth console: skip tests that need the USB "
                         "telemetry stream or startup banner")


def pytest_configure(config):
    config.addinivalue_line("markers", "motors: drives the wheels; needs --motors")
    config.addinivalue_line("markers", "interactive: needs an operator; needs --interactive")
    config.addinivalue_line("markers", "usb_only: needs the USB console; skipped with --bluetooth")


def pytest_collection_modifyitems(config, items):
    skip_motors = pytest.mark.skip(reason="drives the wheels: lift the robot and pass --motors")
    skip_operator = pytest.mark.skip(reason="needs an operator: pass --interactive")
    skip_usb = pytest.mark.skip(reason="telemetry and banner are only sent on the USB console")
    for item in items:
        if "usb_only" in item.keywords and config.getoption("--bluetooth"):
            item.add_marker(skip_usb)
        if "motors" in item.keywords and not config.getoption("--motors"):
            item.add_marker(skip_motors)
        if "interactive" in item.keywords and not config.getoption("--interactive"):
            item.add_marker(skip_operator)


def make_safe(console: AtConsole) -> None:
    """Telemetry off, motors stopped, PID on, default gains."""
    console.stop_stream()
    for command in ("AT+STOP", "AT+PIDON", "AT+DEFAULT"):
        try:
            console.send(command)
        except TimeoutError:
            pass


@pytest.fixture(scope="session")
def robot(pytestconfig):
    port = pytestconfig.getoption("--port")
    try:
        console = AtConsole(port, pytestconfig.getoption("--baud"),
                            assert_dtr_rts=pytestconfig.getoption("--assert-dtr-rts"))
    except serial.SerialException as exc:
        pytest.exit(f"cannot open {port}: {exc} (use --port or ROBOT_PORT)", returncode=2)

    try:
        console.wait_ready()
    except TimeoutError as exc:
        console.close()
        pytest.exit(f"no AT console on {port}: {exc}", returncode=2)

    make_safe(console)
    yield console
    make_safe(console)
    console.close()


@pytest.fixture(autouse=True)
def restore_robot(request):
    """Every test leaves the robot stopped, whatever it did or asserted."""
    yield
    if "robot" in request.fixturenames:
        make_safe(request.getfixturevalue("robot"))


class Operator:
    """Asks a person for help during --interactive tests, bypassing output capture."""

    def __init__(self, config):
        self._capture = config.pluginmanager.getplugin("capturemanager")

    def _ask(self, text: str) -> str:
        with self._capture.global_and_fixture_disabled():
            return input(text)

    def prompt(self, message: str) -> None:
        self._ask(f"\n>>> {message}\n    Press Enter to continue... ")

    def confirm(self, question: str) -> bool:
        return self._ask(f"\n>>> {question} [y/N] ").strip().lower() in ("y", "yes")


@pytest.fixture
def operator(pytestconfig):
    return Operator(pytestconfig)
