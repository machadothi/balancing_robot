"""
Serial client for the robot's AT command console.

Protocol, as implemented in src/cmd/at_cmd.c:
  - commands end with CR; typed characters are echoed (UART_ECHO_ENABLED)
  - a response ends with "OK", "ERROR:<code>", "ERROR:<text>" or "+CMD:value"
  - "> " is printed after every response, without a newline
  - AT+STREAM=1 interleaves "acc_deg: .. | kalman: .. | comp: .." lines
"""

from __future__ import annotations

import re
import time
from dataclasses import dataclass, field

import serial

PROMPT = "> "
BANNER = "=== Balancing Robot"

STREAM_RE = re.compile(
    r"acc_deg:\s*(?P<acc_deg>\S+)\s*\|\s*kalman:\s*(?P<kalman>\S+)\s*\|\s*comp:\s*(?P<comp>\S+)"
)
_ERROR_CODE_RE = re.compile(r"ERROR:(\d+)")


@dataclass
class Response:
    command: str
    status: str                             # "OK", "DATA" or "ERROR"
    lines: list[str] = field(default_factory=list)
    error: int | None = None                # numeric code; None for text errors

    @property
    def ok(self) -> bool:
        return self.status == "OK"

    @property
    def text(self) -> str:
        return "\n".join(self.lines)

    @property
    def data(self) -> str:
        """Value of a "+CMD:value" response."""
        if self.status != "DATA":
            raise AssertionError(f"{self.command!r} returned no data: {self}")
        return self.lines[-1].split(":", 1)[1]


@dataclass
class StreamSample:
    acc_deg: float
    kalman: float
    comp: float


def _strip_prompts(line: str) -> str:
    line = line.replace("\r", "")
    while line.startswith(PROMPT):
        line = line[len(PROMPT):]
    return line.strip()


def parse_stream(text: str) -> list[StreamSample]:
    """Complete telemetry lines in `text`; the first and last line may be cut off."""
    samples = []
    for line in text.split("\n")[1:-1]:
        match = STREAM_RE.fullmatch(_strip_prompts(line))
        if not match:
            continue
        try:
            samples.append(StreamSample(*(float(match[k]) for k in ("acc_deg", "kalman", "comp"))))
        except ValueError:      # "ovf" from the firmware formatter
            samples.append(StreamSample(float("nan"), float("nan"), float("nan")))
    return samples


class AtConsole:
    def __init__(self, port: str, baud: int = 921600, timeout: float = 1.0,
                 assert_dtr_rts: bool = False):
        self.timeout = timeout
        self.serial = serial.Serial()
        self.serial.port = port
        self.serial.baudrate = baud
        self.serial.timeout = 0.02
        # Auto-bootloader circuits (the Hiwonder F407 board) reset the MCU or select
        # the ROM bootloader when DTR/RTS are asserted, so keep them released on open
        self.serial.dtr = assert_dtr_rts
        self.serial.rts = assert_dtr_rts
        self.serial.open()

    def close(self) -> None:
        self.serial.close()

    # -- raw I/O ---------------------------------------------------------------

    def read_for(self, seconds: float) -> str:
        """Everything received during the next `seconds`."""
        deadline = time.monotonic() + seconds
        chunks = []
        while time.monotonic() < deadline:
            chunks.append(self.serial.read(4096).decode(errors="replace"))
        return "".join(chunks)

    def drain(self, quiet: float = 0.05, limit: float = 1.0) -> str:
        """Discard input until the line is quiet for `quiet` s (at most `limit` s)."""
        deadline = time.monotonic() + limit
        received = []
        while time.monotonic() < deadline:
            chunk = self.read_for(quiet)
            if not chunk:
                break
            received.append(chunk)
        return "".join(received)

    # -- commands --------------------------------------------------------------

    def send(self, command: str, timeout: float | None = None) -> Response:
        """Send one command line and wait for the line that ends its response."""
        timeout = timeout or self.timeout
        self.drain()
        self.serial.write(command.encode() + b"\r")

        deadline = time.monotonic() + timeout
        buffer, lines = "", []
        while time.monotonic() < deadline:
            buffer += self.serial.read(4096).decode(errors="replace")
            while "\n" in buffer:
                raw, buffer = buffer.split("\n", 1)
                line = _strip_prompts(raw)
                if not line or line.upper() == command.upper() or STREAM_RE.fullmatch(line):
                    continue            # blank, echo of the command, or telemetry
                lines.append(line)
                if line == "OK":
                    return Response(command, "OK", lines[:-1])
                if line.startswith("ERROR:"):
                    code = _ERROR_CODE_RE.fullmatch(line)
                    return Response(command, "ERROR", lines, int(code[1]) if code else None)
                if line.startswith("+"):
                    return Response(command, "DATA", lines)
        raise TimeoutError(
            f"no complete response to {command!r} within {timeout} s "
            f"(lines {lines!r}, partial {buffer!r})"
        )

    def expect_ok(self, command: str) -> Response:
        response = self.send(command)
        assert response.ok, f"{command} -> {response}"
        return response

    def query(self, name: str) -> str:
        response = self.send(f"AT+{name}?")
        assert response.status == "DATA" and response.lines[-1].startswith(f"+{name}:"), \
            f"AT+{name}? -> {response}"
        return response.data

    def query_float(self, name: str) -> float:
        return float(self.query(name))

    def wait_ready(self, timeout: float = 10.0) -> None:
        """Wait until the robot task has registered its AT handlers."""
        deadline = time.monotonic() + timeout
        last: object = None
        while time.monotonic() < deadline:
            try:
                last = self.send("AT+STATUS?")
                if last.status == "DATA":
                    return
            except TimeoutError as exc:
                last = exc
            time.sleep(0.2)
        raise TimeoutError(f"robot not ready after {timeout} s (last reply: {last})")

    # -- telemetry -------------------------------------------------------------

    def stream(self, seconds: float) -> list[StreamSample]:
        """Collect AT+STREAM samples for `seconds`."""
        self.expect_ok("AT+STREAM=1")
        try:
            text = self.read_for(seconds)
        finally:
            self.stop_stream()
        return parse_stream(text)

    def stop_stream(self, attempts: int = 3) -> None:
        # Telemetry can corrupt the reply to AT+STREAM=0 itself, so retry
        for _ in range(attempts):
            try:
                if self.send("AT+STREAM=0").ok:
                    break
            except TimeoutError:
                pass
        self.drain()
