"""RiskyBird flight bench: build, program, load, commit, stop -- the flight session's FPGA side.

`rb fly` replaces tools/flight/legacy/flight.sh, the script every reload and commit of the
2026-09-23/24 campaign went through. It resolves the adapters itself (by JTAG IDCODE, like
`rb bench`), reuses `rb debug` for every load and GDB step, and checks what the old script
checked by eye: the boot banner, the ESP link, the frame rate, the commit.

    rb fly commit  --mode t21-hover-1.0m       program SRAM, load, verify, commit -> ready to fly
    rb fly load    --mode t21-hover-1.0m       the same without the commit key (dry bench)
    rb fly status                               ESP link: frames/s, gap, kill, flags (resets the ESP!)
    rb fly stop                                 halt the core -> the ESP cuts all motors in 120 ms
    rb fly where                                halt, backtrace, resume
    rb fly program [--flash]                    FullDDR shell into SRAM (or flash, then SRAM)
    rb fly imu                                  guided IMU sign test (motor-incapable image)
    rb fly motors --duty 20 --ms 1500           spin each motor alone (motor identification)

Linux and macOS. Serial capture is plain POSIX termios, no pyserial. Every session writes
results/fly/<timestamp>/{fpga,esp}.log with host timestamps, and copies the build record.

Two hard facts this module is built around, both measured on the drone bench:
  * Re-programming the FullDDR shell with the battery connected is when the unexplained
    motor spin-up happens, so program/load/commit/imu refuse without --pack-out (or a typed
    'pack out' at the prompt).
  * Opening the ESP32-C6's USB serial resets the ESP. That drops its SoftAP for ~8 s and cuts
    the motors through the failsafe. Nothing here opens it while a flight could be running
    without saying so first.
"""

from __future__ import annotations

import os
import re
import select
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass, field
from pathlib import Path

from .boards import ROOT

FLIGHT_GDB = ROOT / "hardware" / "flight"
FREERUN = FLIGHT_GDB / "freerun.gdb"
DEFAULT_TARGET = "arty200t"
DEFAULT_CONFIG = "RocketArty200TDroneFullDDRConfig"
RESULTS = ROOT / "results" / "fly"

BANNER_COMMIT_CAPABLE = "COMMIT GATE -- build is COMMIT-CAPABLE"
BANNER_COMMITTED = "*** COMMITTED TO A FLIGHT TEST ***"
BANNER_DISARMED = "AUTOFLIGHT: disarmed on boot"

_RB_RE = {
    "up": re.compile(r"\bup=(\d+)"),
    "ml": re.compile(r"\bml=(\w+)"),
    "fr": re.compile(r"\bfr=(\d+)"),
    "gap": re.compile(r"\bgap=(\d+)"),
    "kill": re.compile(r"\bkill=(\d+)"),
    "crc": re.compile(r"\bcrc=(\d+)"),
    "cl": re.compile(r"\bcl=(\d+)"),
    "flags": re.compile(r"\bfl=0x[0-9a-fA-F]+\[([^\]]*)\]"),
    "duty": re.compile(r"\bduty=\[([^\]]*)\]"),
}


class FlyError(Exception):
    """An operator-facing flight-bench problem."""


# ---------------------------------------------------------------------------------------------
# RBLINK: the ESP's own link-health line
# ---------------------------------------------------------------------------------------------

def parse_rblink(line: str) -> dict | None:
    """Fields of one `RBLINK v1 ...` line, or None if the line is not one."""
    if "RBLINK v1" not in line:
        return None
    out: dict = {}
    for key, rx in _RB_RE.items():
        m = rx.search(line)
        if m is None:
            continue
        value = m.group(1)
        if key == "duty":
            out[key] = [int(v) for v in value.split()]
        elif key in ("ml", "flags"):
            out[key] = value.strip()
        else:
            out[key] = int(value)
    return out


def link_summary(lines: list[str]) -> dict:
    """Frame rate and the worst gap over a window of RBLINK lines (needs >= 2 lines)."""
    samples = [s for s in (parse_rblink(l) for l in lines) if s and "up" in s and "fr" in s]
    if len(samples) < 2:
        return {"ok": False, "why": "fewer than two RBLINK lines"}
    first, last = samples[0], samples[-1]
    span_ms = last["up"] - first["up"]
    if span_ms <= 0:
        return {"ok": False, "why": "ESP rebooted inside the window"}
    return {
        "ok": True,
        "frames_per_s": (last["fr"] - first["fr"]) * 1000.0 / span_ms,
        "gap_max_ms": max(s.get("gap", 0) for s in samples),
        "ml": last.get("ml"),
        "kill": last.get("kill"),
        "flags": last.get("flags", ""),
        "clients": last.get("cl"),
        "crc": last.get("crc"),
        "duty": last.get("duty"),
    }


# ---------------------------------------------------------------------------------------------
# Serial taps (POSIX termios; Linux and macOS)
# ---------------------------------------------------------------------------------------------

@dataclass
class Tap:
    """Background reader of one serial port: timestamped log file + in-memory lines."""

    path: str
    log: Path
    baud: int = 115200
    writable: bool = False
    lines: list[str] = field(default_factory=list)
    _fd: int | None = None
    _stop: threading.Event = field(default_factory=threading.Event)
    _thread: threading.Thread | None = None
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def _open(self) -> int:
        flags = (os.O_RDWR if self.writable else os.O_RDONLY) | os.O_NOCTTY | os.O_NONBLOCK
        fd = os.open(self.path, flags)
        tty.setraw(fd)
        attrs = termios.tcgetattr(fd)
        speed = getattr(termios, f"B{self.baud}")
        attrs[4] = attrs[5] = speed
        attrs[2] &= ~termios.HUPCL        # do not drop DTR on close
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        return fd

    def start(self) -> "Tap":
        self.log.parent.mkdir(parents=True, exist_ok=True)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def _run(self) -> None:
        buf = b""
        with open(self.log, "ab", buffering=0) as out:
            while not self._stop.is_set():
                if self._fd is None:
                    try:
                        self._fd = self._open()
                    except OSError:
                        time.sleep(0.5)       # the ESP re-enumerates after a reset
                        continue
                try:
                    ready, _, _ = select.select([self._fd], [], [], 0.2)
                    if not ready:
                        continue
                    chunk = os.read(self._fd, 4096)
                    if not chunk:
                        raise OSError("port closed")
                except OSError:
                    try:
                        os.close(self._fd)
                    except OSError:
                        pass
                    self._fd = None
                    continue
                buf += chunk
                *complete, buf = buf.split(b"\n")
                for raw in complete:
                    text = raw.decode("utf-8", "replace").rstrip("\r")
                    out.write(f"{time.time():.3f}\t{text}\n".encode())
                    with self._lock:
                        self.lines.append(text)

    def write(self, data: bytes) -> None:
        deadline = time.time() + 5
        while self._fd is None and time.time() < deadline:
            time.sleep(0.05)
        if self._fd is None:
            raise FlyError(f"{self.path}: not open")
        os.write(self._fd, data)

    def snapshot(self) -> list[str]:
        with self._lock:
            return list(self.lines)

    def mark(self) -> int:
        with self._lock:
            return len(self.lines)

    def since(self, mark: int) -> list[str]:
        with self._lock:
            return self.lines[mark:]

    def wait_for(self, text: str, timeout: float, since: int = 0) -> str | None:
        deadline = time.time() + timeout
        while time.time() < deadline:
            for line in self.since(since):
                if text in line:
                    return line
            time.sleep(0.1)
        return None

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass


# ---------------------------------------------------------------------------------------------
# Bench discovery
# ---------------------------------------------------------------------------------------------

@dataclass
class Bench:
    bitstream: dict | None
    debug: dict | None
    console: str | None
    esp: str | None


def _holders(path: str) -> list[str]:
    """Other processes with this device open (Linux /proc; empty elsewhere or if unknown)."""
    if not path.startswith("/dev/") or not Path("/proc").is_dir():
        return []
    try:
        target = os.path.realpath(path)
    except OSError:
        return []
    found = []
    for pid_dir in Path("/proc").iterdir():
        if not pid_dir.name.isdigit() or int(pid_dir.name) == os.getpid():
            continue
        try:
            for fd in (pid_dir / "fd").iterdir():
                if os.path.realpath(fd) == target:
                    cmd = (pid_dir / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
                    found.append(f"pid {pid_dir.name}: {cmd[:160]}")
                    break
        except (PermissionError, FileNotFoundError, ProcessLookupError, OSError):
            continue
    return found


def discover(args) -> Bench:
    """Bitstream and debug adapters by IDCODE; FPGA console and ESP ports."""
    from . import cli, artifacts

    adapters = [a for a in cli.ftdi_adapters() if a["pid"] in cli.MPSSE_PRODUCT_IDS]
    bit = dbg = None
    want_dbg = getattr(args, "debug_location", None) or os.environ.get("RB_FTDI_LOCATION")
    want_bit = getattr(args, "bitstream_location", None)
    probed = []
    for a in adapters:
        if want_dbg and a["location"] == want_dbg:
            dbg = a
            continue
        if want_bit and a["location"] == want_bit:
            bit = a
            continue
        idcode = cli.probe_idcode(a["location"])
        probed.append((a, idcode))
    for a, idcode in probed:
        if bit is None and "config TAP" in idcode and "Alveo" not in idcode:
            bit = a
    for a, idcode in probed:
        if dbg is None and "Rocket debug TAP" in idcode:
            dbg = a
    if dbg is None and bit is not None:
        # No design running reads all ones on the debug chain, and an adapter whose console
        # another process holds may not probe at all -- so neither result is an identity.
        # What is: the drone's two adapters are the same model on the same hub. Pick that
        # one adapter, or refuse; never fall back to "some other adapter on the hub".
        hub = bit["location"].rsplit(".", 1)[0]
        twins = [a for a, _ in probed
                 if a is not bit and a["location"].rsplit(".", 1)[0] == hub
                 and a.get("product") == bit.get("product")]
        if len(twins) == 1:
            dbg = twins[0]
        elif twins:
            raise FlyError("more than one candidate debug adapter ("
                           + ", ".join(a["location"] for a in twins) + "); pass --debug-location")
    if dbg is not None:
        held = _holders(cli._tty_for(dbg, 1))
        if held:
            raise FlyError(f"the FPGA console ({cli._tty_for(dbg, 1)}) is held open by another process:\n  "
                           + "\n  ".join(held)
                           + "\n  Another session may be using the drone. Stop that first.")
    console = getattr(args, "console", None) or os.environ.get("RB_CONSOLE_PORT")
    if not console and dbg is not None:
        tty_path = cli._tty_for(dbg, 1)
        console = tty_path if tty_path.startswith("/dev/") else None
    esp = getattr(args, "esp_port", None)
    if not esp:
        try:
            esp = str(artifacts.select_serial_port("esp", None))
        except (FileNotFoundError, RuntimeError):
            esp = None
    return Bench(bit, dbg, console, esp)


def _require(bench: Bench, *, bitstream=False, debug=False, console=False, esp=False) -> None:
    missing = []
    if bitstream and bench.bitstream is None:
        missing.append("bitstream adapter (config TAP) -- pass --bitstream-location")
    if debug and bench.debug is None:
        missing.append("debug adapter -- pass --debug-location (rb probe-ftdi --idcode)")
    if console and not bench.console:
        missing.append("FPGA console port -- pass --console (or set RB_CONSOLE_PORT)")
    if esp and not bench.esp:
        missing.append("ESP32-C6 USB port -- pass --esp-port (or set RB_ESP_PORT)")
    if missing:
        raise FlyError("could not find:\n  " + "\n  ".join(missing))


def _confirm_pack_out(args) -> None:
    if getattr(args, "pack_out", False):
        return
    print("The LiPo must be OUT: re-programming the FullDDR shell with a pack connected is when")
    print("the drone's unexplained motor spin-up happens. (Pass --pack-out to skip this prompt.)")
    if not sys.stdin.isatty():
        raise FlyError("refusing without --pack-out (no terminal to confirm on)")
    if input("Type 'pack out' to continue: ").strip().lower() != "pack out":
        raise FlyError("not confirmed")


# ---------------------------------------------------------------------------------------------
# Steps
# ---------------------------------------------------------------------------------------------

def _session_dir() -> Path:
    path = RESULTS / time.strftime("%Y%m%d-%H%M%S")
    path.mkdir(parents=True, exist_ok=True)
    return path


def program_sram(args, bench: Bench) -> None:
    """The flight shell into SRAM (default), or flash then SRAM with --flash."""
    from . import cli

    target = cli.get_target(args.target)
    board = target.fpga_board.with_config(args.config)
    location = bench.bitstream["location"]
    if getattr(args, "flash", False):
        cli.program_fpga(board, "auto", location)
        return
    if not board.bitstream.is_file():
        cli.program_fpga(board, "auto", location)   # raises the full 'where to get it' error
        return
    print(f"rb fly: {board.config} -> SRAM via {location}")
    cli.run(
        ["openFPGALoader", "--board", board.openfpgaloader_board,
         *cli.openfpgaloader_probe_selector(location), "--write-sram", "--bitstream", board.bitstream],
        environment="host", cwd=ROOT,
    )
    cli.require_fpga_configured(location)


def rb_debug(args, bench: Bench, mode: str, script: Path, *, workload="flight_controller",
             no_load=False, program=False) -> None:
    """One `rb debug` invocation, exactly as an operator would type it."""
    from . import cli

    argv = ["debug", workload, "--target", args.target, "--config", args.config,
            "--mode", mode, "--usb-location", bench.debug["location"], "--gdb-script", str(script)]
    if no_load:
        argv.append("--no-load")
    ns = cli.build_parser().parse_args(argv)
    rc = ns.func(ns)
    if rc not in (0, None):
        raise FlyError(f"rb {' '.join(argv)} exited {rc}")


def build(args, mode: str, workload="flight_controller") -> tuple[Path, Path]:
    from . import cli, boards

    target = cli.get_target(args.target)
    board = target.fpga_board.with_config(args.config)
    wl = cli.get_workload(workload)
    elf = cli.zephyr_build(target, wl, mode, getattr(args, "pristine", "auto"), board)
    return elf, elf.parent.parent / "rb-build.txt"


def _sha(path: Path) -> str:
    import hashlib
    return hashlib.sha1(path.read_bytes()).hexdigest()[:16]


def _taps(bench: Bench, session: Path, *, fpga=True, esp=True, fpga_writable=False) -> dict:
    taps = {}
    if fpga and bench.console:
        taps["fpga"] = Tap(bench.console, session / "fpga.log", writable=fpga_writable).start()
    if esp and bench.esp:
        taps["esp"] = Tap(bench.esp, session / "esp.log").start()
    return taps


def _wait_link(esp: Tap, since: int, timeout: float = 25.0, need: int = 8) -> dict:
    """Wait until the ESP reports the link up for `need` consecutive RBLINK lines (~1.6 s at
    5 Hz), then summarise only those: lines from before the FPGA started sending carry fr=0
    and would drag the rate down. The rate uses the ESP's own `up=` clock, because its USB
    serial delivers buffered lines in bursts and host timestamps are meaningless for rate."""
    deadline = time.time() + timeout
    summary: dict = {"ok": False, "why": "no RBLINK lines"}
    while time.time() < deadline:
        run: list[str] = []
        for line in esp.since(since):
            parsed = parse_rblink(line)
            if parsed is None:
                continue
            run = run + [line] if parsed.get("ml") == "up" else []
        if len(run) >= need:
            summary = link_summary(run[-need:])
            if summary.get("ok"):
                return summary
        elif run:
            summary = {"ok": False, "why": f"link up for only {len(run)} lines so far", "ml": "up"}
        time.sleep(0.5)
    return summary


def load_and_check(args, *, commit: bool) -> int:
    from . import cli

    _confirm_pack_out(args)
    bench = discover(args)
    _require(bench, bitstream=not args.no_program, debug=True, console=True, esp=True)
    session = _session_dir()
    elf, record = build(args, args.mode)
    loaded = _sha(elf)
    if not args.no_program:
        program_sram(args, bench)
    taps = _taps(bench, session)
    try:
        print("rb fly: opening the ESP console can reset the ESP; giving it time to boot")
        time.sleep(3.0)
        fpga_mark, esp_mark = taps["fpga"].mark(), taps["esp"].mark()
        rb_debug(args, bench, args.mode, FREERUN)
        _remember_mode(args.mode)
        if _sha(elf) != loaded:
            raise FlyError("the ELF was rebuilt during the load; rerun")
        banner = taps["fpga"].wait_for(BANNER_COMMIT_CAPABLE, 15, fpga_mark) or \
            taps["fpga"].wait_for(BANNER_DISARMED, 1, fpga_mark)
        link = _wait_link(taps["esp"], esp_mark)
        boot = "\n  ".join(l for l in taps["fpga"].since(fpga_mark)
                           if any(k in l for k in ("COMMIT GATE", "DUTY CHAIN -- WORST", "AUTOFLIGHT:",
                                                     "BATTERY SENSE", "gyro-cal")))
        print("rb fly: boot banners\n  " + (boot or "(none captured)"))
        if not banner or not link.get("ok") or link.get("ml") != "up":
            raise FlyError(f"NO-GO: banner={'yes' if banner else 'NO'} link={link}. "
                           f"Logs: {session}")
        print(f"rb fly: ESP link up: {link['frames_per_s']:.0f} frames/s, worst gap "
              f"{link['gap_max_ms']} ms (failsafe 120), kill={link['kill']}, flags [{link['flags']}]")
        if commit:
            mark = taps["fpga"].mark()
            rb_debug(args, bench, args.mode, FLIGHT_GDB / "commit.gdb", no_load=True)
            if not taps["fpga"].wait_for(BANNER_COMMITTED, 10, mark):
                raise FlyError(f"the commit key was written but the gate never reported it. Logs: {session}")
            print("rb fly: " + BANNER_COMMITTED)
        (session / "rb-build.txt").write_text(record.read_text())
        print(f"rb fly: build record {record}\nrb fly: session logs {session}")
        if commit:
            print("READY: pack in, pull the JTAG ribbon then USB-C, confirm both streams on the ground\n"
                  "station for 10 s, level floor, then RESET on the panel. ESTOP (E) stops it.")
        return 0
    finally:
        for tap in taps.values():
            tap.stop()


def status(args) -> int:
    bench = discover(args)
    _require(bench, esp=True)
    print("rb fly: opening the ESP console can RESET the ESP (SoftAP drops ~8 s, motors cut by the failsafe).")
    if not args.yes and sys.stdin.isatty():
        if input("Nothing is flying? [y/N] ").strip().lower() != "y":
            return 1
    session = _session_dir()
    taps = _taps(bench, session, fpga=False)
    try:
        time.sleep(3.0 + args.seconds)
        lines = [l for l in taps["esp"].snapshot() if "RBLINK" in l]
        s = link_summary(lines[-12:])
        print(lines[-1] if lines else "(no RBLINK lines)")
        if s.get("ok"):
            print(f"frames/s {s['frames_per_s']:.0f}  worst gap {s['gap_max_ms']} ms  ml={s['ml']}  "
                  f"kill={s['kill']}  flags [{s['flags']}]  clients={s['clients']}")
        return 0 if s.get("ok") and s.get("ml") == "up" else 2
    finally:
        taps["esp"].stop()


LAST_MODE = RESULTS / "last-mode"


def _remember_mode(mode: str, workload: str = "flight_controller") -> None:
    RESULTS.mkdir(parents=True, exist_ok=True)
    LAST_MODE.write_text(f"{workload} {mode}\n")


def _last_mode() -> tuple[str, str]:
    try:
        workload, mode = LAST_MODE.read_text().split()
        return workload, mode
    except (OSError, ValueError):
        return "flight_controller", ""


def gdb_step(args, script: Path) -> int:
    """stop / where. GDB needs the symbols of the image that is RUNNING, so the mode defaults
    to the last one `rb fly load/commit` put on the board."""
    bench = discover(args)
    _require(bench, debug=True)
    workload, mode = _last_mode()
    mode = args.mode or mode
    if not mode:
        raise FlyError("which image is running? pass --mode (no rb fly load/commit recorded one)")
    rb_debug(args, bench, mode, script, workload=workload, no_load=True)
    return 0


# ---------------------------------------------------------------------------------------------
# IMU sign test (motor-incapable image)
# ---------------------------------------------------------------------------------------------

_IMU_RE = re.compile(r"iter=(\d+) a=\[([-0-9. ]+)\] g=\[([-0-9. ]+)\]")


def _imu_samples(lines: list[str]) -> list[tuple[int, list[float], list[float]]]:
    out = []
    for line in lines:
        m = _IMU_RE.search(line)
        if m:
            out.append((int(m.group(1)), [float(v) for v in m.group(2).split()],
                        [float(v) for v in m.group(3).split()]))
    return out


def imu_verdict(step: str, samples) -> tuple[bool, str]:
    """Check one guided move against the body-frame convention (x fwd, y left, z up)."""
    if len(samples) < 5:
        return False, "too few samples"
    a = [s[1] for s in samples]
    g = [s[2] for s in samples]
    if step == "level":
        az = sum(v[2] for v in a) / len(a)
        return az > 8.5, f"az mean {az:+.2f} (want > +8.5)"
    if step == "nose-up":
        ax = max(v[0] for v in a); gy = min(v[1] for v in g)
        return ax > 1.5 and gy < -0.3, f"ax max {ax:+.2f} (want > +1.5), gy min {gy:+.2f} (want < -0.3)"
    if step == "right-wing-down":
        ay = max(v[1] for v in a); gx = max(v[0] for v in g)
        return ay > 1.5 and gx > 0.3, f"ay max {ay:+.2f} (want > +1.5), gx max {gx:+.2f} (want > +0.3)"
    if step == "nose-left":
        gz = max(v[2] for v in g)
        return gz > 0.3, f"gz max {gz:+.2f} (want > +0.3)"
    return False, "unknown step"


def imu(args) -> int:
    _confirm_pack_out(args)
    bench = discover(args)
    _require(bench, bitstream=not args.no_program, debug=True, console=True)
    session = _session_dir()
    build(args, "imu-test")
    if not args.no_program:
        program_sram(args, bench)
    taps = _taps(bench, session, esp=False)
    try:
        rb_debug(args, bench, "imu-test", FREERUN)
        _remember_mode("imu-test")
        if not taps["fpga"].wait_for("iter=", 15):
            raise FlyError("no IMU lines on the console")
        results = []
        for step, text in (("level", "Hold it LEVEL and still"),
                           ("nose-up", "Tilt NOSE UP ~30 deg and back"),
                           ("right-wing-down", "Tilt RIGHT WING DOWN ~30 deg and back"),
                           ("nose-left", "Turn it NOSE-LEFT (counter-clockwise from above) ~90 deg")):
            input(f"\n{text} -- press Enter, then do it within 4 s ")
            mark = taps["fpga"].mark()
            time.sleep(4.0)
            ok, why = imu_verdict(step, _imu_samples(taps["fpga"].since(mark)))
            results.append(ok)
            print(f"  {step:16s} {'PASS' if ok else 'FAIL'}  {why}")
        print(f"\nrb fly imu: {'ALL PASS' if all(results) else 'FAILED'}  (logs {session})")
        return 0 if all(results) else 2
    finally:
        for tap in taps.values():
            tap.stop()


# ---------------------------------------------------------------------------------------------
# Motor identification (motor_duty)
# ---------------------------------------------------------------------------------------------

def motors(args) -> int:
    print("MOTOR TEST: spins each selected motor ALONE at "
          f"{args.duty}% for {args.ms} ms. Pack IN, props on or off, hands clear.")
    print("Stops: each step ends itself; the ESP cuts all four 120 ms after frames stop; pull the pack.")
    if not args.yes:
        if not sys.stdin.isatty() or input("Type 'spin' to run it: ").strip().lower() != "spin":
            raise FlyError("not confirmed")
    if not 5 <= args.duty <= 25:
        raise FlyError("--duty must be 5..25 (motor_duty's own ceiling is 25)")
    bench = discover(args)
    _require(bench, debug=True, console=True, esp=True)
    session = _session_dir()
    build(args, "default", workload="motor_duty")
    taps = _taps(bench, session, fpga_writable=True)
    try:
        time.sleep(3.0)
        rb_debug(args, bench, "default", FREERUN, workload="motor_duty")
        _remember_mode("default", "motor_duty")
        fpga = taps["fpga"]
        if not fpga.wait_for("Keys:", 15):
            raise FlyError("motor_duty banner not seen on the console")
        mark = fpga.mark(); fpga.write(b"+"); time.sleep(0.3); fpga.write(b"-"); time.sleep(0.5)
        current = None
        for line in fpga.since(mark):
            m = re.search(r"duty=(\d+)\.(\d+)%", line)
            if m:
                current = int(m.group(1))
        if current is None:
            raise FlyError("could not read the current duty")
        step = args.duty - current
        keys = (b"]" * (abs(step) // 5) if step > 0 else b"[" * (abs(step) // 5))
        keys += (b"+" * (abs(step) % 5) if step > 0 else b"-" * (abs(step) % 5))
        for k in keys:
            fpga.write(bytes([k])); time.sleep(0.15)
        applied = {}
        for motor in args.motor:
            esp_mark = taps["esp"].mark()
            for k in (b"n", str(motor).encode(), b"g"):
                fpga.write(k); time.sleep(0.15)
            time.sleep(args.ms / 1000.0)
            fpga.write(b"x"); time.sleep(1.0)
            duties = [parse_rblink(l).get("duty") for l in taps["esp"].since(esp_mark) if "RBLINK" in l]
            applied[motor] = max((d for d in duties if d), key=lambda d: max(d), default=None)
            print(f"  motor {motor}: ESP applied {applied[motor]}")
        print(f"rb fly motors: done (logs {session}). Tell the operator: corner, spin direction from above, air down?")
        return 0
    finally:
        for tap in taps.values():
            tap.stop()
