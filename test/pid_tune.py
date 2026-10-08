#!/usr/bin/env python3
"""PID tuning tool for the balancing robot.

Records the control loop's telemetry (tilt, P/I/D terms, output) over the USB
console, prints and analyses it, and runs experiments: setpoint steps and gain
sweeps. Captures are CSV files in test/pid_logs/ with the gains used in a
JSON header line, so runs can be compared later.

  status                         gains, setpoint, angle, motor state
  gains [--kp N] [--ki N] [--kd N]
  check-mount                    guided check: angle sign, gyro vs accelerometer, wheel direction
  capture [-s SECONDS] [--enable] [--name NAME]
  show FILE [--every N] [--plot PNG]
  analyze FILE
  step [--angle DEG] [--hold S] [--repeat N]     setpoint steps while balancing
  sweep {kp,ki,kd} VALUE... [-s SECONDS]         one capture per value, compared
  motor-test                     wheels in the air: dead zone, direction, wiring (encoders)
  noise [-s SECONDS]             robot held still: motor command produced by sensor noise
  deadband [--write]             wheels in the air: measure each motor's dead zone, apply it

With --bt MAC, commands go over the Bluetooth console and telemetry is read
from USB, so command replies and the 100 Hz stream never share a line.

Typical session (robot balancing, e.g. armed with the board button):
  pid_tune.py check-mount
  pid_tune.py capture -s 10 --name baseline && pid_tune.py analyze test/pid_logs/<file>
  pid_tune.py sweep kp 15 20 25
  pid_tune.py step --angle 2
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import socket
import sys
import time
from dataclasses import asdict
from pathlib import Path

from at_console import AtConsole, is_telemetry, parse_telemetry_line

LOG_DIR = Path(__file__).resolve().parent / "pid_logs"
OUTPUT_LIMIT = 255.0        # MOTOR_COMMAND_MAX
FALL_ANGLE = 45.0           # MAX_TILT_ANGLE: the firmware cuts the motors beyond it
FIELDS = ["host_s", "sp", "seq", "t", "acc_deg", "kalman", "comp", "tilt", "p", "i", "d", "out", "v", "spe", "drops"]


# =============================================================================
# Robot access
# =============================================================================

class _RfcommPort:
    """Serial-like RFCOMM socket: the Bluetooth console without `rfcomm bind` (no root)."""

    def __init__(self, mac: str, channel: int = 1):
        self.sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        self.sock.settimeout(10)
        self.sock.connect((mac, channel))
        self.sock.settimeout(0.02)

    def read(self, size: int) -> bytes:
        try:
            return self.sock.recv(size)
        except (TimeoutError, BlockingIOError):
            return b""

    def write(self, data: bytes) -> None:
        self.sock.sendall(data)

    def close(self) -> None:
        self.sock.close()


class BtConsole(AtConsole):
    """AtConsole over Bluetooth: same protocol, no echo, no telemetry."""

    def __init__(self, mac: str, timeout: float = 2.0):
        self.timeout = timeout
        self.serial = _RfcommPort(mac)


def connect(args) -> tuple[AtConsole, AtConsole]:
    """(command console, telemetry console): the same USB console without --bt.

    Telemetry only streams on USB. With --bt and no USB cable, both are the
    Bluetooth console: commands work, captures stay empty.
    """
    try:
        tele = AtConsole(args.port, args.baud)
    except Exception:
        if not args.bt:
            raise
        print(f"note: {args.port} not available, telemetry disabled (commands over Bluetooth)")
        tele = None
    cmd = BtConsole(args.bt) if args.bt else tele
    cmd.wait_ready()
    return cmd, tele or cmd


def read_gains(con: AtConsole) -> dict:
    gains = {name.lower(): con.query_float(name) for name in ("KP", "KI", "KD", "SETPOINT")}
    gains["outlimit_counts"] = round(con.query_float("OUTLIMIT") * 2.55, 1)
    return gains


def status_text(con: AtConsole) -> str:
    return con.query("STATUS")


def record(cmd: AtConsole, tele: AtConsole, seconds: float, setpoint: float, schedule=None) -> list[dict]:
    """Telemetry for `seconds`; `schedule` is [(time_s, setpoint)] applied on the way.

    Setpoint changes are written without waiting for their reply, so no record
    is lost; the reply line is skipped like any other non-telemetry line.
    """
    events = sorted(schedule or [])
    samples, buffer = [], ""
    cmd.expect_ok("AT+STREAM=1")
    start = time.monotonic()
    try:
        while (now := time.monotonic() - start) < seconds:
            while events and events[0][0] <= now:
                _, setpoint = events.pop(0)
                cmd.serial.write(f"AT+SETPOINT={setpoint:.2f}\r".encode())
            buffer += tele.serial.read(4096).decode(errors="replace")
            *lines, buffer = buffer.split("\n")
            for line in lines:
                rec = parse_telemetry_line(line) if is_telemetry(line) else None
                if rec is not None:
                    samples.append({"host_s": round(now, 4), "sp": setpoint, **asdict(rec)})
    finally:
        cmd.stop_stream()
        if tele is not cmd:
            tele.drain()
    return samples


def save(samples: list[dict], meta: dict, name: str) -> Path:
    LOG_DIR.mkdir(exist_ok=True)
    path = LOG_DIR / f"{time.strftime('%Y%m%d-%H%M%S')}_{name}.csv"
    with path.open("w", newline="") as f:
        f.write("# " + json.dumps(meta) + "\n")
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(samples)
    return path


def load(path: str) -> tuple[dict, list[dict]]:
    with open(path) as f:
        meta = json.loads(f.readline()[2:])
        rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]
    return meta, rows


# =============================================================================
# Analysis
# =============================================================================

def rms(values) -> float:
    values = list(values)
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else 0.0


def mean(values) -> float:
    values = list(values)
    return sum(values) / len(values) if values else 0.0


def oscillation(times, values, hysteresis=0.2) -> float:
    """Dominant frequency (Hz) from zero crossings of the de-meaned signal."""
    if len(values) < 10:
        return 0.0
    m = mean(values)
    crossings, state = 0, None
    for v in values:
        v -= m
        if v > hysteresis and state != 1:
            crossings += state is not None
            state = 1
        elif v < -hysteresis and state != -1:
            crossings += state is not None
            state = -1
    duration = times[-1] - times[0]
    return crossings / 2 / duration if duration > 0 else 0.0


def balancing_part(rows: list[dict]) -> tuple[list[dict], float | None]:
    """Rows while the controller was active, and the time of a fall if any."""
    active = [r for r in rows if r["out"] != 0 or r["p"] != 0]
    if not active:
        return [], None
    # Only a fall after balancing started counts (the robot lies down before arming)
    start = active[0]["host_s"]
    fall = next((r["host_s"] for r in rows if r["host_s"] > start and abs(r["tilt"]) > FALL_ANGLE), None)
    if fall is not None:
        active = [r for r in active if r["host_s"] < fall]
    return active, fall


def metrics(rows: list[dict], limit: float = OUTPUT_LIMIT) -> dict:
    active, fall = balancing_part(rows)
    if len(active) < 20:
        return {"samples": len(rows), "active": len(active), "fall_s": fall}
    t = [r["host_s"] for r in active]
    err = [r["tilt"] - r["sp"] for r in active]
    out = [r["out"] for r in active]
    flips = sum(1 for a, b in zip(out, out[1:]) if a * b < 0)
    return {
        "samples": len(rows),
        "active": len(active),
        "duration_s": round(t[-1] - t[0], 2),
        "fall_s": fall,
        "tilt_rms": round(rms(err), 3),
        "tilt_mean": round(mean(err), 3),
        "tilt_peak": round(max(abs(e) for e in err), 2),
        "osc_hz": round(oscillation(t, err), 2),
        "sat_pct": round(100 * sum(abs(o) >= limit - 0.5 for o in out) / len(out), 1),
        "out_rms": round(rms(out), 1),
        "out_flips_per_s": round(flips / max(t[-1] - t[0], 1e-3), 1),
        "p_rms": round(rms(r["p"] for r in active), 1),
        "i_mean": round(mean(r["i"] for r in active), 1),
        "d_rms": round(rms(r["d"] for r in active), 1),
        "drops": int(max(r["drops"] for r in rows) - min(r["drops"] for r in rows)),
    }


def suggestions(m: dict, gains: dict) -> list[str]:
    tips = []
    if m.get("active", 0) < 20:
        return ["The controller never ran: the motors were off. Arm with the button and lift "
                "the robot upright, or pass --enable with the robot held upright."]
    if m["fall_s"] is not None:
        tips.append(f"Fell after {m['fall_s']:.1f} s: check the steps below and `check-mount`.")
    if m["sat_pct"] > 5:
        tips.append(f"Output saturated {m['sat_pct']}% of the time: the command is bigger than the "
                    "motors can deliver. Lower KP (about 20-30%) and KD together.")
    if m["osc_hz"] >= 4 and m["d_rms"] > m["p_rms"]:
        tips.append(f"Fast oscillation ({m['osc_hz']} Hz) dominated by the D term: KD amplifies "
                    "sensor noise. Lower KD in 20% steps (the gyro-rate D input in TODO fixes the root cause).")
    elif m["osc_hz"] >= 4:
        tips.append(f"Fast oscillation ({m['osc_hz']} Hz): the loop is too aggressive. Lower KP by ~20%.")
    elif 0.5 <= m["osc_hz"] < 4 and m["tilt_rms"] > 1.0:
        tips.append(f"Slow, large oscillation ({m['osc_hz']} Hz, {m['tilt_rms']} deg RMS): under-damped. "
                    "Raise KD by ~25%, or lower KP.")
    if m["out_flips_per_s"] > 20:
        tips.append(f"The output changes sign {m['out_flips_per_s']} times/s (motor buzz): too much D "
                    "or noise around the deadband. Lower KD.")
    if abs(m["tilt_mean"]) > 0.5:
        tips.append(f"Average tilt error {m['tilt_mean']:+.2f} deg: the balance point is off. "
                    f"Try `AT+SETPOINT={gains.get('setpoint', 0) + m['tilt_mean']:.2f}` "
                    "(or move weight) instead of relying on KI.")
    if gains.get("ki", 0) > 0 and abs(m["i_mean"]) > 0.8 * gains["ki"] * 100:
        tips.append("The I term sits near its clamp: something constantly pushes one way "
                    "(balance point, uneven motors). Fix that first; then KI can stay small.")
    if not tips:
        tips.append("No obvious problem. Next: `step --angle 2` for the dynamic response, or a "
                    "`sweep` around the current gains to find the least tilt RMS.")
    return tips


def step_metrics(rows: list[dict]) -> list[dict]:
    """Rise time, overshoot, settling and steady error for each setpoint change."""
    results = []
    changes = [k for k in range(1, len(rows)) if rows[k]["sp"] != rows[k - 1]["sp"]]
    for n, k in enumerate(changes):
        end = changes[n + 1] if n + 1 < len(changes) else len(rows)
        seg = rows[k:end]
        before = rows[max(0, k - 50):k]
        if len(seg) < 20 or not before:
            continue
        start_level = mean(r["tilt"] for r in before)
        target = seg[0]["sp"]
        delta = target - start_level
        if abs(delta) < 0.2:
            continue
        t0 = seg[0]["host_s"]
        progress = [((r["tilt"] - start_level) / delta, r["host_s"] - t0) for r in seg]
        t10 = next((t for p, t in progress if p >= 0.1), None)
        t90 = next((t for p, t in progress if p >= 0.9), None)
        band = max(0.3, 0.1 * abs(delta))
        outside = [r["host_s"] - t0 for r in seg if abs(r["tilt"] - target) > band]
        tail = seg[int(len(seg) * 0.7):]
        results.append({
            "at_s": round(t0, 2),
            "from": round(start_level, 2),
            "to": target,
            "rise_s": round(t90 - t10, 3) if t10 is not None and t90 is not None else None,
            "overshoot_pct": round(100 * max(0.0, max(p for p, _ in progress) - 1), 1),
            "settle_s": round(outside[-1], 2) if outside else 0.0,
            "steady_err": round(mean(r["tilt"] for r in tail) - target, 2),
        })
    return results


def print_table(rows: list[dict], columns: list[str]) -> None:
    if not rows:
        print("(no results)")
        return
    widths = {c: max(len(c), *(len(f"{r.get(c)}") for r in rows)) for c in columns}
    print("  ".join(c.rjust(widths[c]) for c in columns))
    for r in rows:
        print("  ".join(f"{r.get(c)}".rjust(widths[c]) for c in columns))


def report(path: Path | str) -> None:
    meta, rows = load(str(path))
    m = metrics(rows, meta.get("outlimit_counts", OUTPUT_LIMIT))
    print(f"{path}\n  gains: kp={meta.get('kp')} ki={meta.get('ki')} kd={meta.get('kd')} "
          f"setpoint={meta.get('setpoint')}  status at start: {meta.get('status')}")
    for key, value in m.items():
        print(f"  {key:16} {value}")
    steps = step_metrics(rows)
    if steps:
        print("\n  setpoint steps:")
        print_table(steps, ["at_s", "from", "to", "rise_s", "overshoot_pct", "settle_s", "steady_err"])
    print("\n  suggestions:")
    for tip in suggestions(m, meta):
        print("   - " + tip)


# =============================================================================
# Commands
# =============================================================================

def cmd_status(args) -> None:
    cmd, tele = connect(args)
    print(f"status   {status_text(cmd)}")
    for key, value in read_gains(cmd).items():
        print(f"{key:8} {value}")
    print(f"angle    {cmd.query_float('ANGLE')}")


def cmd_gains(args) -> None:
    cmd, tele = connect(args)
    for name in ("kp", "ki", "kd"):
        value = getattr(args, name)
        if value is not None:
            cmd.expect_ok(f"AT+{name.upper()}={value}")
    print(read_gains(cmd))


def cmd_capture(args) -> None:
    cmd, tele = connect(args)
    gains = read_gains(cmd)
    if args.enable:
        cmd.expect_ok("AT+ENABLE")
    status = status_text(cmd)
    if status.startswith("DISABLED"):
        print("note: motors are off, so only the sensor side is recorded "
              "(arm with the button, or use --enable while holding the robot upright)")
    print(f"recording {args.seconds} s ...")
    samples = record(cmd, tele, args.seconds, gains["setpoint"])
    path = save(samples, {**gains, "status": status, "seconds": args.seconds}, args.name)
    print(f"{len(samples)} records -> {path}")
    report(path)


def cmd_show(args) -> None:
    meta, rows = load(args.file)
    print(f"# {meta}")
    print(f"{'time':>6} {'sp':>6} {'tilt':>7} {'p':>7} {'i':>7} {'d':>7} {'out':>7}   tilt (+-10 deg)")
    for r in rows[::args.every]:
        pos = int(max(-10, min(10, r["tilt"])) * 2) + 20
        bar = [" "] * 41
        bar[20] = "|"
        bar[pos] = "*"
        print(f"{r['host_s']:6.2f} {r['sp']:6.2f} {r['tilt']:7.2f} {r['p']:7.1f} {r['i']:7.1f} "
              f"{r['d']:7.1f} {r['out']:7.1f}   {''.join(bar)}")
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        t = [r["host_s"] for r in rows]
        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(11, 6))
        ax1.plot(t, [r["tilt"] for r in rows], label="tilt")
        ax1.plot(t, [r["sp"] for r in rows], "--", label="setpoint")
        ax1.plot(t, [r["acc_deg"] for r in rows], alpha=0.3, label="accelerometer")
        ax1.set_ylabel("deg")
        ax1.legend()
        for key in ("p", "i", "d", "out"):
            ax2.plot(t, [r[key] for r in rows], label=key)
        ax2.set_ylabel("counts")
        ax2.set_xlabel("s")
        ax2.legend()
        fig.suptitle(f"kp={meta.get('kp')} ki={meta.get('ki')} kd={meta.get('kd')}")
        fig.savefig(args.plot, dpi=120)
        print(f"plot -> {args.plot}")


def cmd_analyze(args) -> None:
    report(args.file)


def cmd_step(args) -> None:
    cmd, tele = connect(args)
    gains = read_gains(cmd)
    status = status_text(cmd)
    if not status.startswith("ENABLED"):
        sys.exit("the robot must be balancing: arm it with the button and lift it upright first")
    base = gains["setpoint"]
    schedule, t = [], args.settle
    for _ in range(args.repeat):
        schedule += [(t, base + args.angle), (t + args.hold, base)]
        t += 2 * args.hold
    seconds = t + args.settle
    print(f"steps of {args.angle:+} deg around {base} for {seconds:.0f} s ...")
    try:
        samples = record(cmd, tele, seconds, base, schedule)
    finally:
        cmd.send(f"AT+SETPOINT={base:.2f}")
    path = save(samples, {**gains, "status": status, "step_angle": args.angle}, f"step{args.angle:+g}")
    print(f"{len(samples)} records -> {path}")
    report(path)


def cmd_sweep(args) -> None:
    cmd, tele = connect(args)
    gains = read_gains(cmd)
    original = gains[args.param]
    results = []
    try:
        for value in args.values:
            if not status_text(cmd).startswith("ENABLED"):
                print("the robot is not balancing (fell or stopped): sweep ended")
                break
            cmd.expect_ok(f"AT+{args.param.upper()}={value}")
            time.sleep(args.settle)
            samples = record(cmd, tele, args.seconds, gains["setpoint"])
            meta = {**gains, args.param: value, "status": "sweep"}
            path = save(samples, meta, f"sweep_{args.param}{value:g}")
            results.append({args.param: value, **metrics(samples, gains["outlimit_counts"]), "file": path.name})
    finally:
        cmd.send(f"AT+{args.param.upper()}={original}")
        print(f"{args.param} restored to {original}")
    print_table(results, [args.param, "tilt_rms", "tilt_peak", "osc_hz", "sat_pct", "out_flips_per_s", "fall_s"])
    ok = [r for r in results if r.get("tilt_rms") is not None and r.get("fall_s") is None]
    if ok:
        best = min(ok, key=lambda r: r["tilt_rms"])
        print(f"\nleast tilt RMS: {args.param}={best[args.param]} ({best['tilt_rms']} deg); "
              f"set it with: pid_tune.py gains --{args.param} {best[args.param]}")


def cmd_check_mount(args) -> None:
    cmd, tele = connect(args)

    def average_tilt(seconds=1.5):
        rows = record(cmd, tele, seconds, 0.0)
        return mean(r["tilt"] for r in rows), rows

    input("1/4  Hold the robot upright and still, then press Enter ")
    upright, _ = average_tilt()
    print(f"     upright reads {upright:+.2f} deg"
          + ("  (ok)" if abs(upright) < 3 else f"  -> trim with AT+SETPOINT={upright:.2f} or fix the mounting"))

    input("2/4  Lean it FORWARD about 20 deg (the direction it should drive to catch itself), Enter ")
    forward, _ = average_tilt()
    lean = forward - upright
    print(f"     forward reads {forward:+.2f} deg" + ("  (ok: positive)" if lean > 5 else
          "  -> WRONG: leaning forward must be positive; negate BOARD_TILT_ACC_NUM and BOARD_TILT_RATE"
          if lean < -5 else "  -> barely changed: wrong axis pair in BOARD_TILT_*"))

    input("3/4  Rock it slowly back and forth for 5 s after pressing Enter ")
    _, rows = average_tilt(5.0)
    swing = rms([r["acc_deg"] - mean(x["acc_deg"] for x in rows) for r in rows])
    lag = rms([r["comp"] - r["acc_deg"] for r in rows])
    print(f"     swing {swing:.1f} deg RMS, filter vs accelerometer {lag:.1f} deg RMS"
          + ("  (ok: gyro agrees)" if swing > 2 and lag < 0.5 * swing else
             "  -> the gyro fights the accelerometer: negate BOARD_TILT_RATE" if swing > 2 else
             "  -> rock it more"))

    answer = input("4/4  Wheels off the ground for a 1 s motor test? [y/N] ")
    if answer.lower().startswith("y"):
        cmd.expect_ok("AT+PIDOFF")
        cmd.expect_ok("AT+ENABLE")
        cmd.expect_ok("AT+SPEED=25,25")
        time.sleep(1.0)
        cmd.expect_ok("AT+STOP")
        cmd.expect_ok("AT+PIDON")
        answer = input("     Did both wheels drive toward the FRONT (the side that read positive)? [y/N] ")
        print("     ok" if answer.lower().startswith("y") else
              "     -> swap that motor's forward/reverse channels in motor_hiwonder.c (or its cable)")


def safe_stop(cmd: AtConsole, attempts: int = 4) -> None:
    """Stop the wheels even if a command or the link just failed."""
    for _ in range(attempts):
        try:
            cmd.send("AT+SPEED=0,0")
            cmd.send("AT+STOP")
            return
        except Exception:
            time.sleep(0.5)
    print("WARNING: could not confirm the motors stopped - switch the board off")


def cmd_motor_test(args) -> None:
    """Wheels in the air: dead zone, direction, wiring and speed per command, from the encoders."""
    cmd, tele = connect(args)
    encoders = lambda: [int(x) for x in cmd.query("ENC").split(",")]
    rows = []
    cmd.expect_ok("AT+PIDOFF")
    cmd.expect_ok("AT+ENABLE")
    try:
        for index, wheel in enumerate(("left", "right")):
            for pct in args.levels:
                speeds = [0, 0]
                speeds[index] = pct
                try:
                    cmd.expect_ok(f"AT+SPEED={speeds[0]},{speeds[1]}")
                    time.sleep(args.settle)
                    before, t0 = encoders(), time.monotonic()
                    time.sleep(args.window)
                    after, t1 = encoders(), time.monotonic()
                finally:
                    cmd.send("AT+SPEED=0,0")   # never leave a wheel running between steps
                rate = [(a - b) / (t1 - t0) for a, b in zip(after, before)]
                rows.append({"wheel": wheel, "cmd_pct": pct, "own_cps": round(rate[index]),
                             "other_cps": round(rate[1 - index])})
            time.sleep(0.3)
    finally:
        safe_stop(cmd)
        try:
            cmd.send("AT+PIDON")
        except Exception:
            pass
    print_table(rows, ["wheel", "cmd_pct", "own_cps", "other_cps"])

    print()
    for wheel in ("left", "right"):
        mine = [r for r in rows if r["wheel"] == wheel]
        moving = [r for r in mine if abs(r["own_cps"]) > args.threshold]
        if not moving:
            print(f"{wheel}: never moved (motor cable, port number, or the dead zone is above {max(args.levels)}%)")
            continue
        first = min((r for r in moving if r["cmd_pct"] > 0), key=lambda r: r["cmd_pct"], default=None)
        forward = [r for r in moving if r["cmd_pct"] > 0]
        sign_ok = all(r["own_cps"] > 0 for r in forward) and all(r["own_cps"] < 0 for r in moving if r["cmd_pct"] < 0)
        crosstalk = any(abs(r["other_cps"]) > abs(r["own_cps"]) for r in moving)
        top = max(forward, key=lambda r: r["cmd_pct"], default=None)
        print(f"{wheel}: starts at {first['cmd_pct'] if first else '?'}% "
              f"(~{round(first['cmd_pct'] * 2.55) if first else '?'} counts; `deadband` measures it precisely)"
              + (f", {top['own_cps']} counts/s at {top['cmd_pct']}%" if top else "")
              + ("" if sign_ok else "; encoder counts backwards for a forward command (encoder or motor polarity)")
              + ("; the OTHER encoder moves more: left/right wiring swapped" if crosstalk else ""))


def measure_thresholds(cmd: AtConsole, wheel: int, top: int, threshold: float) -> tuple[int | None, int | None]:
    """(start-from-rest %, keep-running %) for one wheel, in 1 % steps up to `top`."""
    def set_wheel(pct):
        speeds = [0, 0]
        speeds[wheel] = pct
        cmd.send(f"AT+SPEED={speeds[0]},{speeds[1]}")

    def rate(window):
        a, t0 = int(cmd.query("ENC").split(",")[wheel]), time.monotonic()
        time.sleep(window)
        b = int(cmd.query("ENC").split(",")[wheel])
        return (b - a) / (time.monotonic() - t0)

    start = None
    for pct in range(5, top + 1):
        set_wheel(0)
        time.sleep(0.5)
        set_wheel(pct)
        time.sleep(0.4)
        if rate(0.5) > threshold:
            start = pct
            break

    keep = None
    if start is not None:
        set_wheel(top)
        time.sleep(0.8)
        for pct in range(top - 1, 4, -1):
            set_wheel(pct)
            time.sleep(0.3)
            if rate(0.4) <= threshold:
                break
            keep = pct
    set_wheel(0)
    return start, keep


def write_board_deadband(board: str, left: int, right: int) -> Path:
    """Replace BOARD_MOTOR_DEADBAND_LEFT/RIGHT in the board config."""
    path = Path(__file__).resolve().parent.parent / "src" / "board" / board / "board_config.h"
    text = path.read_text()
    for side, value in (("LEFT", left), ("RIGHT", right)):
        pattern = rf"(#define BOARD_MOTOR_DEADBAND_{side}\s+)\d+"
        if not re.search(pattern, text):
            sys.exit(f"{path}: no BOARD_MOTOR_DEADBAND_{side} line to update; add one first")
        text = re.sub(pattern, rf"\g<1>{value}", text)
    path.write_text(text)
    return path


def cmd_deadband(args) -> None:
    """Wheels in the air: each motor's dead zone, from its encoder; applied with AT+DEADBAND."""
    cmd, tele = connect(args)
    old = cmd.query("DEADBAND")
    results = {}
    cmd.expect_ok("AT+PIDOFF")
    cmd.expect_ok("AT+ENABLE")
    try:
        for wheel, name in enumerate(("left", "right")):
            print(f"measuring {name} wheel ...", flush=True)
            results[name] = measure_thresholds(cmd, wheel, args.max, args.threshold)
    finally:
        safe_stop(cmd)
        try:
            cmd.send("AT+PIDON")
        except Exception:
            pass

    counts = {}
    for name, (start, keep) in results.items():
        if keep is None:
            sys.exit(f"{name}: did not turn up to {args.max}% (motor, cable or encoder?); nothing changed")
        counts[name] = round(keep * 2.55)
        print(f"{name:5}: starts from rest at {start}%, keeps turning down to {keep}% "
              f"-> dead zone {counts[name]} counts")
        if start - keep >= 5:
            print(f"       ({start - keep}% between starting and running: strong static friction)")

    cmd.expect_ok(f"AT+DEADBAND={counts['left']},{counts['right']}")
    print(f"\nAT+DEADBAND={counts['left']},{counts['right']} applied (was {old}); lost at reset unless written")
    if args.write:
        path = write_board_deadband(args.board, counts["left"], counts["right"])
        print(f"{path} updated: rebuild and flash to keep it")
    else:
        print(f"keep it: rerun with --write, or set BOARD_MOTOR_DEADBAND_LEFT/RIGHT in "
              f"src/board/{args.board}/board_config.h")


def cmd_noise(args) -> None:
    """Robot held still (stand): how much motor command sensor noise produces through the PID."""
    cmd, tele = connect(args)
    gains = read_gains(cmd)
    rows = record(cmd, tele, 2.0, gains["setpoint"])
    trim = mean(r["tilt"] for r in rows)
    noise = rms([r["tilt"] - trim for r in rows])
    print(f"resting tilt {trim:+.2f} deg, sensor noise {noise:.3f} deg RMS")
    cmd.expect_ok(f"AT+SETPOINT={max(-10.0, min(10.0, trim)):.2f}")
    cmd.expect_ok("AT+PIDON")
    cmd.expect_ok("AT+ENABLE")
    try:
        samples = record(cmd, tele, args.seconds, round(trim, 2))
    finally:
        cmd.send("AT+STOP")
        cmd.send(f"AT+SETPOINT={gains['setpoint']:.2f}")
    path = save(samples, {**gains, "status": "noise test (stand)", "trim": trim}, "noise")
    m = metrics(samples)
    print(f"{len(samples)} records -> {path}")
    print(f"  output from noise: {m.get('out_rms')} counts RMS, sign flips {m.get('out_flips_per_s')}/s, "
          f"P {m.get('p_rms')} / D {m.get('d_rms')} counts RMS")
    deadband = args.deadband or min(int(x) for x in cmd.query("DEADBAND").split(","))
    if m.get("d_rms", 0) > deadband:
        kd_max = gains["kd"] * deadband / m["d_rms"]
        print(f"  D turns noise alone into {m['d_rms']} counts, above the {deadband}-count dead zone: "
              f"the motors twitch constantly. KD up to about {kd_max:.2f} keeps noise below it.")
    else:
        print(f"  D noise stays below the {deadband}-count dead zone at KD={gains['kd']}: fine.")
    if m.get("p_rms", 0) > deadband:
        print(f"  P turns noise into {m['p_rms']} counts: KP {gains['kp']} is high for this sensor noise.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--bt", metavar="MAC", help="send commands over Bluetooth (e.g. 98:DA:60:05:77:01)")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("status").set_defaults(func=cmd_status)

    p = sub.add_parser("gains")
    for name in ("kp", "ki", "kd"):
        p.add_argument(f"--{name}", type=float)
    p.set_defaults(func=cmd_gains)

    sub.add_parser("check-mount").set_defaults(func=cmd_check_mount)

    p = sub.add_parser("motor-test", help="wheels in the air: dead zone, direction, wiring")
    p.add_argument("--levels", type=float, nargs="+", default=[5, 10, 15, 20, 25, 30, -20])
    p.add_argument("--settle", type=float, default=0.4, help="seconds to spin up at each level")
    p.add_argument("--window", type=float, default=0.6, help="seconds counted")
    p.add_argument("--threshold", type=float, default=20, help="counts/s that count as moving")
    p.set_defaults(func=cmd_motor_test)

    p = sub.add_parser("deadband", help="wheels in the air: measure each motor's dead zone and apply it")
    p.add_argument("--max", type=int, default=40, help="highest command tried, percent")
    p.add_argument("--threshold", type=float, default=30, help="counts/s that count as turning")
    p.add_argument("--write", action="store_true", help="also update the board config")
    p.add_argument("--board", default="f407")
    p.set_defaults(func=cmd_deadband)

    p = sub.add_parser("noise", help="robot held still: motor command produced by sensor noise")
    p.add_argument("-s", "--seconds", type=float, default=5.0)
    p.add_argument("--deadband", type=float, help="dead zone in counts (default: AT+DEADBAND?)")
    p.set_defaults(func=cmd_noise)

    p = sub.add_parser("capture")
    p.add_argument("-s", "--seconds", type=float, default=10.0)
    p.add_argument("--enable", action="store_true", help="send AT+ENABLE first (hold the robot upright!)")
    p.add_argument("--name", default="capture")
    p.set_defaults(func=cmd_capture)

    p = sub.add_parser("show")
    p.add_argument("file")
    p.add_argument("--every", type=int, default=5, help="print every Nth record (5 = 20 per second)")
    p.add_argument("--plot", metavar="PNG", help="also save a plot (needs matplotlib)")
    p.set_defaults(func=cmd_show)

    p = sub.add_parser("analyze")
    p.add_argument("file")
    p.set_defaults(func=cmd_analyze)

    p = sub.add_parser("step")
    p.add_argument("--angle", type=float, default=2.0, help="step size in degrees (keep it small)")
    p.add_argument("--hold", type=float, default=2.0, help="seconds at each level")
    p.add_argument("--repeat", type=int, default=3)
    p.add_argument("--settle", type=float, default=2.0, help="seconds before the first and after the last step")
    p.set_defaults(func=cmd_step)

    p = sub.add_parser("sweep")
    p.add_argument("param", choices=["kp", "ki", "kd"])
    p.add_argument("values", type=float, nargs="+")
    p.add_argument("-s", "--seconds", type=float, default=6.0)
    p.add_argument("--settle", type=float, default=1.0, help="seconds after each change before recording")
    p.set_defaults(func=cmd_sweep)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
