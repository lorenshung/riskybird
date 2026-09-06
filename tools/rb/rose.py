"""RiskyBird integration for the FPGA-free RoSE lockstep Spike flow."""

from __future__ import annotations

import hashlib
import html
import json
import os
import re
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

from .boards import ROOT, ZEPHYR_BACKEND
from .environments import ZEPHYR_PREFIX


ROSE = ROOT / "backends" / "RoSE"
ROSE_SIM = ROSE / "soc" / "sim"
ROSE_CHIPYARD = ROSE_SIM / "chipyard"
SPIKE = ROSE_CHIPYARD / "toolchains" / "riscv-tools" / "riscv-isa-sim"
SPIKE_BUILD = SPIKE / "build"
BRIDGE_SOURCE = ROSE / "soc" / "src" / "main" / "cc"
BRIDGE_BINARY = ROSE_SIM / "rose_spike_sim"
ROSE_MODULE = ROSE / "soc" / "sw" / "zephyr-rose"
GYM_SOURCE = ZEPHYR_BACKEND / "tools" / "gym-pybullet-drones"
GYM_CONFIG = ROSE / "deploy" / "config" / "config_gym_PyBulletDroneMPCEnv-v0.yaml"
ROSE_ENV = ROOT / ".rb" / "envs" / "rose"
GENERATED = ROOT / ".rb" / "generated" / "rose-spike"
COMPAT = ROOT / "tools" / "rb" / "compat"
DEFAULT_GUEST = (
    ROOT / "build" / "zephyr" / "rose-spike" / "mpc" / "rose"
    / "zephyr" / "zephyr.elf"
)


class RoseError(RuntimeError):
    """Concise failure from the RoSE integration."""


def step_cycles(clock_mhz: float, period_ms: float) -> int:
    if clock_mhz <= 0 or period_ms <= 0:
        raise RoseError("clock and control period must be positive")
    cycles = round(clock_mhz * 1_000_000 * period_ms / 1_000)
    if cycles < 1 or cycles > 0xFFFFFFFF:
        raise RoseError(f"control grant {cycles} cycles is outside the 32-bit protocol")
    return cycles


def _python_candidates(explicit: Path | None = None) -> list[Path]:
    candidates = [
        explicit,
        ROSE_ENV / "bin" / "python",
        ROSE / "deploy" / ".venv-rose" / "bin" / "python",
        ZEPHYR_PREFIX / "bin" / "python",
    ]
    return [path for path in candidates if path is not None]


def python_environment(python: Path | None = None) -> dict[str, str]:
    env = dict(os.environ)
    path = os.pathsep.join((str(COMPAT), str(GYM_SOURCE), str(ROOT)))
    if env.get("PYTHONPATH"):
        path += os.pathsep + env["PYTHONPATH"]
    env.update(
        {
            "PYTHONPATH": path,
            "PYTHONUNBUFFERED": "1",
            "PYTHONNOUSERSITE": "1",
            "MPLCONFIGDIR": str(ROOT / ".rb" / "matplotlib"),
            "ROSE_DIR": str(ROSE),
            "RB_ROOT": str(ROOT),
        }
    )
    (ROOT / ".rb" / "matplotlib").mkdir(parents=True, exist_ok=True)
    return env


def _python_works(path: Path) -> tuple[bool, str]:
    if not path.is_file():
        return False, "not installed"
    code = (
        "import yaml,numpy,gymnasium,pybullet; "
        "from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary; "
        "print('host dependencies import')"
    )
    try:
        result = subprocess.run(
            [str(path), "-c", code],
            cwd=ROOT,
            env=python_environment(path),
            capture_output=True,
            text=True,
            timeout=20,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, str(exc)
    detail = (result.stdout or result.stderr).strip().splitlines()
    return result.returncode == 0, detail[-1] if detail else f"exit {result.returncode}"


def select_python(explicit: Path | None = None) -> Path:
    failures = []
    for candidate in _python_candidates(explicit):
        ok, detail = _python_works(candidate)
        if ok:
            return candidate
        failures.append(f"{candidate}: {detail}")
    raise RoseError(
        "no Python environment can run the RoSE host; run `rb env setup rose`\n  "
        + "\n  ".join(failures)
    )


def setup_environment() -> Path:
    python = ROSE_ENV / "bin" / "python"
    if not python.is_file():
        subprocess.run(
            ["/usr/bin/python3.12", "-m", "venv", str(ROSE_ENV)],
            cwd=ROOT,
            check=True,
        )
    subprocess.run(
        [str(python), "-m", "pip", "install", "-r", str(ROSE / "deploy" / "requirements.txt")],
        cwd=ROOT,
        check=True,
    )
    return python


def environment_shell() -> int:
    python = select_python(ROSE_ENV / "bin" / "python")
    env = python_environment(python)
    env["VIRTUAL_ENV"] = str(python.parent.parent)
    env["PATH"] = str(python.parent) + os.pathsep + env.get("PATH", "")
    env["PS1"] = "(rb:rose) \\u@\\h:\\w\\$ "
    return subprocess.call(["/bin/bash", "--noprofile", "--norc"], cwd=ROOT, env=env)


def doctor(require_ready: bool = False, explicit_python: Path | None = None) -> int:
    sim_h = SPIKE / "riscv" / "sim.h"
    libs = tuple(SPIKE_BUILD / name for name in (
        "libriscv.a", "libfesvr.a", "libsoftfloat.a", "libdisasm.a", "libfdt.a"
    ))
    py = None
    py_ok = False
    py_detail = "no candidate"
    for candidate in _python_candidates(explicit_python):
        py_ok, py_detail = _python_works(candidate)
        if py_ok:
            py = candidate
            break
    checks = [
        ("RoSE backend", ROSE.is_dir(), str(ROSE), True),
        ("Spike source", sim_h.is_file(), str(sim_h), True),
        ("Spike libraries", all(path.is_file() for path in libs), str(SPIKE_BUILD), True),
        ("RoSE Zephyr module", (ROSE_MODULE / "zephyr" / "module.yml").is_file(), str(ROSE_MODULE), True),
        ("TinyMPC source", (ZEPHYR_BACKEND / "samples" / "drone_control" / "tinympc" / "external" / "admm" / "include" / "admm.hpp").is_file(), str(ZEPHYR_BACKEND), True),
        ("Host Python", py_ok, f"{py or ''} ({py_detail})", True),
        ("Bridge harness", BRIDGE_BINARY.is_file(), str(BRIDGE_BINARY), require_ready),
        ("MPC guest", DEFAULT_GUEST.is_file(), str(DEFAULT_GUEST), require_ready),
    ]
    print("RoSE executor: rose-spike (lockstep, FPGA-free)")
    failed = False
    for label, passed, detail, required in checks:
        state = "ok" if passed else ("MISSING" if required else "build")
        print(f"{label:20} {state:7} {detail}")
        failed |= required and not passed
    print("U250 / FireSim       unused  this executor never invokes FireSim")
    return 1 if failed else 0


def _chipyard_environment() -> dict[str, str]:
    script = 'source "$1" >/dev/null 2>&1; env -0'
    result = subprocess.run(
        ["/bin/bash", "-c", script, "rb-rose", str(ROSE_CHIPYARD / "env.sh")],
        cwd=ROSE_CHIPYARD,
        capture_output=True,
        check=True,
    )
    env: dict[str, str] = {}
    for item in result.stdout.split(b"\0"):
        if b"=" in item:
            key, value = item.split(b"=", 1)
            env[key.decode(errors="replace")] = value.decode(errors="replace")
    return env


def _generated_sim_header() -> Path:
    source = SPIKE / "riscv" / "sim.h"
    text = source.read_text(encoding="utf-8")
    marker = "protected: /* RiskyBird RoSE generated access */"
    if marker not in text and "RoSE lockstep: expose step()" not in text:
        needle = "  void step(size_t n); // step through simulation"
        if needle not in text:
            raise RoseError(f"cannot locate sim_t::step() in {source}")
        text = text.replace(
            needle,
            f"{marker}\n{needle}\nprivate:",
            1,
        )
    target = GENERATED / "riscv" / "sim.h"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(text, encoding="utf-8")
    return target


def build_bridge(force: bool = False) -> Path:
    inputs = [
        Path(__file__),
        BRIDGE_SOURCE / "rose_sync_client.cc",
        BRIDGE_SOURCE / "rose_spike" / "rose_spike_sim.cc",
        SPIKE / "riscv" / "sim.h",
        *(SPIKE_BUILD / name for name in (
            "libriscv.a", "libfesvr.a", "libsoftfloat.a", "libdisasm.a", "libfdt.a"
        )),
    ]
    missing = [path for path in inputs if not path.is_file()]
    if missing:
        raise RoseError("bridge build input is missing: " + ", ".join(str(p) for p in missing))
    if not force and BRIDGE_BINARY.is_file() and all(
        BRIDGE_BINARY.stat().st_mtime >= path.stat().st_mtime for path in inputs
    ):
        print(f"RoSE bridge is up to date: {BRIDGE_BINARY}")
        return BRIDGE_BINARY

    _generated_sim_header()
    flags = [
        "-std=c++2a", "-O2", "-D_GNU_SOURCE", "-include", "sys/syscall.h",
        f"-I{GENERATED}", f"-I{SPIKE}", f"-I{SPIKE / 'riscv'}",
        f"-I{SPIKE / 'fesvr'}", f"-I{SPIKE / 'softfloat'}", f"-I{SPIKE_BUILD}",
    ]
    command = [
        "g++", *flags,
        str(BRIDGE_SOURCE / "rose_sync_client.cc"),
        str(BRIDGE_SOURCE / "rose_spike" / "rose_spike_sim.cc"),
        "-Wl,--start-group",
        *(str(SPIKE_BUILD / name) for name in (
            "libriscv.a", "libfesvr.a", "libsoftfloat.a", "libdisasm.a", "libfdt.a"
        )),
        "-Wl,--end-group", "-lpthread", "-ldl", "-o", str(BRIDGE_BINARY),
    ]
    print("+", " ".join(command), flush=True)
    subprocess.run(command, cwd=ROSE_CHIPYARD, env=_chipyard_environment(), check=True)
    print(BRIDGE_BINARY)
    return BRIDGE_BINARY


def _new_run_dir(output: Path | None) -> Path:
    if output is None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        output = ROOT / "results" / "runs" / f"{stamp}_rose_mpc_hover"
    elif not output.is_absolute():
        output = ROOT / output
    if output.exists() and any(output.iterdir()):
        raise RoseError(f"output directory is not empty: {output}")
    (output / "logs").mkdir(parents=True, exist_ok=True)
    return output


def _json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _repository_state() -> tuple[str | None, bool | None]:
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True,
            text=True, check=True,
        ).stdout.strip()
        dirty = bool(subprocess.run(
            ["git", "status", "--short"], cwd=ROOT, capture_output=True,
            text=True, check=True,
        ).stdout.strip())
        return commit, dirty
    except (OSError, subprocess.CalledProcessError):
        return None, None


def _tail(path: Path, lines: int = 20) -> str:
    if not path.is_file():
        return ""
    return "\n".join(path.read_text(encoding="utf-8", errors="replace").splitlines()[-lines:])


def _wait_for(path: Path, needle: str, process: subprocess.Popen[Any], timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if needle in (path.read_text(errors="replace") if path.is_file() else ""):
            return
        if process.poll() is not None:
            raise RoseError(f"synchronizer exited before readiness:\n{_tail(path)}")
        time.sleep(0.05)
    raise RoseError(f"timed out waiting for synchronizer readiness:\n{_tail(path)}")


def _stop(process: subprocess.Popen[Any] | None) -> None:
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)


def _events(path: Path) -> list[dict[str, Any]]:
    events = []
    if not path.is_file():
        return events
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return events


def summarize(trace: Path, guest_log: Path, requested_steps: int, period_ms: float,
              sync_rc: int | None, timed_out: bool) -> dict[str, Any]:
    events = _events(trace)
    physics = [event for event in events if event.get("event") == "physics_step"]
    packets = [event for event in events if event.get("event") == "bridge_packet"]
    z_errors = [float(event["z_error_m"]) for event in physics if "z_error_m" in event]
    tail_size = max(1, len(z_errors) // 5) if z_errors else 0
    tail = z_errors[-tail_size:] if tail_size else []
    differential = 0
    applied_controls: list[float] = []
    nominal_bound_violations = 0
    action_space_violations = 0
    for event in physics:
        action = event.get("action") or []
        applied_controls.extend(float(value) for value in action[:4])
        if len(action) >= 4 and max(action[:4]) - min(action[:4]) > 1e-3:
            differential += 1
        if len(action) >= 4 and any(value < -0.583 or value > 0.417 for value in action[:4]):
            nominal_bound_violations += 1
        if len(action) >= 4 and any(value < -1.0 or value > 1.0 for value in action[:4]):
            action_space_violations += 1
    guest_text = guest_log.read_text(encoding="utf-8", errors="replace") if guest_log.is_file() else ""
    solves = [
        {"iteration": int(match.group(1)), "status": int(match.group(2)), "solve_ns": int(match.group(3))}
        for match in re.finditer(r"RB_MPC_ROSE iter=(\d+) status=(-?\d+).*?solve_ns=(\d+)", guest_text)
    ]
    state_requests = sum(event.get("cmd") == 0x12 for event in packets)
    control_updates = sum(event.get("cmd") == 0x20 for event in packets)
    nonconverged = sum(item["status"] != 0 for item in solves)
    execution_ok = bool(physics) and sync_rc == 0 and not timed_out
    if not execution_ok:
        verdict = "error" if sync_rc not in (0, None) else "incomplete"
    elif len(physics) < requested_steps:
        verdict = "incomplete"
    elif requested_steps < 100:
        verdict = "smoke-pass" if state_requests and control_updates and "RB_MPC_ROSE ready" in guest_text else "smoke-incomplete"
    elif tail and max(abs(value) for value in tail) <= 0.15 and differential:
        verdict = "hover-pass"
    else:
        verdict = "unstable"
    warnings = []
    if nonconverged:
        warnings.append(f"{nonconverged} solver call(s) reached the iteration cap")
    if nominal_bound_violations:
        warnings.append(
            f"{nominal_bound_violations} applied step(s) exceeded TinyMPC's nominal [-0.583, 0.417] control bounds"
        )
    if action_space_violations:
        warnings.append(
            f"{action_space_violations} applied step(s) exceeded the environment's declared [-1, 1] action space"
        )
    return {
        "schema": "riskybird.rose-mpc-summary.v1",
        "verdict": verdict,
        "execution_ok": execution_ok,
        "requested_steps": requested_steps,
        "physics_steps": len(physics),
        "simulated_time_s": len(physics) * period_ms / 1000.0,
        "state_requests": state_requests,
        "control_updates": control_updates,
        "solver_completions": len(solves),
        "solver_statuses": sorted({item["status"] for item in solves}),
        "solver_nonconverged": nonconverged,
        "solve_ns_max": max((item["solve_ns"] for item in solves), default=None),
        "final_z_error_m": z_errors[-1] if z_errors else None,
        "tail_max_abs_z_error_m": max((abs(value) for value in tail), default=None),
        "differential_action_steps": differential,
        "applied_control_min": min(applied_controls, default=None),
        "applied_control_max": max(applied_controls, default=None),
        "nominal_control_bound_violation_steps": nominal_bound_violations,
        "environment_action_space_violation_steps": action_space_violations,
        "synchronizer_exit": sync_rc,
        "timed_out": timed_out,
        "warnings": warnings,
    }


def refresh_summary(run_dir: Path) -> dict[str, Any]:
    """Re-derive a run summary and plot from its immutable trace and logs."""
    config = json.loads((run_dir / "rendered-config.json").read_text(encoding="utf-8"))
    old_path = run_dir / "summary.json"
    old = json.loads(old_path.read_text(encoding="utf-8")) if old_path.is_file() else {}
    summary = summarize(
        run_dir / "trace.jsonl",
        run_dir / "logs" / "guest.log",
        int(config["steps"]),
        float(config["control_period_ms"]),
        old.get("synchronizer_exit", 0),
        bool(old.get("timed_out", False)),
    )
    _json(old_path, summary)
    render_plot(run_dir / "trace.jsonl", summary, run_dir / "plot.svg")
    return summary


def _points(values: Iterable[tuple[float, float]], x0: float, y0: float,
            width: float, height: float, xmax: float, ymin: float, ymax: float) -> str:
    scale_y = ymax - ymin or 1.0
    return " ".join(
        f"{x0 + width * x / (xmax or 1):.1f},{y0 + height * (ymax - y) / scale_y:.1f}"
        for x, y in values
    )


def render_plot(trace: Path, summary: dict[str, Any], output: Path) -> Path:
    physics = [event for event in _events(trace) if event.get("event") == "physics_step"]
    if not physics:
        raise RoseError(f"no physics samples in {trace}")
    times = [float(event["sim_time_s"]) for event in physics]
    z = [float((event.get("pos") or [0, 0, 0])[2]) for event in physics]
    actions = [[float(v) for v in (event.get("action") or [])[:4]] for event in physics]
    xmax = max(times) or 1.0
    zmin, zmax = min(0.0, min(z) - 0.1), max(1.1, max(z) + 0.1)
    action_values = [v for row in actions for v in row] or [-1.0, 1.0]
    amin, amax = min(-0.7, min(action_values)), max(0.5, max(action_values))
    colors = ("#2d6cdf", "#df6c2d", "#3a9d5d", "#9a4fb0")
    title = html.escape(f"RoSE TinyMPC · {summary['verdict']}")
    parts = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="920" height="600" viewBox="0 0 920 600">',
        '<rect width="920" height="600" fill="#fbfbf8"/>',
        f'<text x="54" y="38" font-family="sans-serif" font-size="20" font-weight="600">{title}</text>',
        f'<text x="54" y="60" font-family="sans-serif" font-size="12" fill="#555">{summary["physics_steps"]} physics steps · {summary["control_updates"]} controls · FPGA-free rose-spike executor</text>',
    ]
    for y0, label in ((86, "Altitude (m)"), (340, "Normalized motor control")):
        parts += [
            f'<rect x="54" y="{y0}" width="812" height="190" fill="#fff" stroke="#d9d9d2"/>',
            f'<text x="54" y="{y0 - 8}" font-family="sans-serif" font-size="12" fill="#333">{label}</text>',
        ]
    target_y = 86 + 190 * (zmax - 1.0) / (zmax - zmin)
    parts.append(f'<line x1="54" y1="{target_y:.1f}" x2="866" y2="{target_y:.1f}" stroke="#999" stroke-dasharray="5 4"/>')
    parts.append(f'<polyline fill="none" stroke="#176b87" stroke-width="2.2" points="{_points(zip(times, z), 54, 86, 812, 190, xmax, zmin, zmax)}"/>')
    for motor in range(4):
        series = ((t, row[motor]) for t, row in zip(times, actions) if len(row) == 4)
        parts.append(f'<polyline fill="none" stroke="{colors[motor]}" stroke-width="1.5" points="{_points(series, 54, 340, 812, 190, xmax, amin, amax)}"/>')
        parts.append(f'<text x="{610 + motor * 62}" y="326" font-family="sans-serif" font-size="11" fill="{colors[motor]}">motor {motor}</text>')
    parts += [
        '<text x="460" y="565" text-anchor="middle" font-family="sans-serif" font-size="12">simulated time (s)</text>',
        '<text x="54" y="588" font-family="sans-serif" font-size="10" fill="#666">Solid altitude trace; dashed line is the 1.0 m target. Values come from trace.jsonl.</text>',
        '</svg>',
    ]
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(parts) + "\n", encoding="utf-8")
    return output


def run_mpc_hover(*, guest: Path = DEFAULT_GUEST, clock_mhz: float = 1000.0,
                  period_ms: float = 20.0, steps: int = 300,
                  timeout_s: float = 600.0, output: Path | None = None,
                  python: Path | None = None) -> tuple[Path, dict[str, Any]]:
    if steps < 1:
        raise RoseError("steps must be at least one")
    if not BRIDGE_BINARY.is_file():
        raise RoseError(f"bridge harness is missing: {BRIDGE_BINARY}")
    if not guest.is_file():
        raise RoseError(f"MPC guest is missing: {guest}")
    host_python = select_python(python)
    cycles = step_cycles(clock_mhz, period_ms)
    run_dir = _new_run_dir(output)
    trace = run_dir / "trace.jsonl"
    sync_log = run_dir / "logs" / "synchronizer.log"
    guest_log = run_dir / "logs" / "guest.log"
    config = {
        "schema": "riskybird.rose-mpc-config.v1",
        "scenario": "mpc-hover",
        "executor": "rose-spike",
        "mode": "lockstep",
        "clock_mhz": clock_mhz,
        "control_period_ms": period_ms,
        "step_cycles": cycles,
        "steps": steps,
        "gym_env": "PyBulletDroneMPCEnv-v0",
        "gym_config": str(GYM_CONFIG),
        "sync_host": "127.0.0.1",
        "sync_port": 10001,
    }
    _json(run_dir / "rendered-config.json", config)
    commit, dirty = _repository_state()
    _json(run_dir / "meta.json", {
        "schema": "riskybird.run-meta.v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": [
            "rb", "hil", "replay", "mpc-hover", "--executor", "rose-spike",
            "--clock-mhz", str(clock_mhz), "--period-ms", str(period_ms),
            "--steps", str(steps),
        ],
        "executor": "rose-spike",
        "uses_firesim": False,
        "uses_u250": False,
        "guest": str(guest),
        "guest_sha256": _sha256(guest),
        "bridge": str(BRIDGE_BINARY),
        "bridge_sha256": _sha256(BRIDGE_BINARY),
        "host_python": str(host_python),
        "repository_commit": commit,
        "repository_dirty": dirty,
    })

    sync_process: subprocess.Popen[Any] | None = None
    sim_process: subprocess.Popen[Any] | None = None
    timed_out = False
    sync_rc: int | None = None
    run_failure: RoseError | None = None
    try:
        with sync_log.open("w", encoding="utf-8") as sync_stream, guest_log.open("w", encoding="utf-8") as guest_stream:
            sync_process = subprocess.Popen(
                [str(host_python), "-m", "tools.rb.rose_sync",
                 "--config", str(run_dir / "rendered-config.json"),
                 "--trace", str(trace)],
                cwd=ROOT,
                env=python_environment(host_python),
                stdout=sync_stream,
                stderr=subprocess.STDOUT,
                text=True,
            )
            _wait_for(sync_log, "RB_ROSE_SYNC_READY", sync_process, 30)
            sim_process = subprocess.Popen(
                [str(BRIDGE_BINARY), "-p", "1", "--rose-host=127.0.0.1",
                 "--rose-port=10001", "--rose-base=0x2000", "--rose-irq=3",
                 "--rose-dma-base=0x88000000", "--rose-nreqrsp=2", "--rose-ndma=1",
                 str(guest)],
                cwd=ROOT,
                env=_chipyard_environment(),
                stdout=guest_stream,
                stderr=subprocess.STDOUT,
                text=True,
            )
            deadline = time.monotonic() + timeout_s
            while sync_process.poll() is None:
                if sim_process.poll() is not None:
                    time.sleep(0.5)
                    if sync_process.poll() is None:
                        raise RoseError(f"Spike exited early:\n{_tail(guest_log)}")
                    break
                if time.monotonic() >= deadline:
                    timed_out = True
                    break
                time.sleep(0.1)
            sync_rc = sync_process.poll()
    except RoseError as exc:
        run_failure = exc
    finally:
        _stop(sim_process)
        _stop(sync_process)
        if sync_process is not None:
            sync_rc = sync_process.returncode

    summary = summarize(trace, guest_log, steps, period_ms, sync_rc, timed_out)
    _json(run_dir / "summary.json", summary)
    if _events(trace):
        render_plot(trace, summary, run_dir / "plot.svg")
    print(f"run      {run_dir}")
    print(f"verdict  {summary['verdict']}")
    print(f"steps    {summary['physics_steps']}/{steps}")
    print(f"packets  state={summary['state_requests']} control={summary['control_updates']}")
    print(f"plot     {run_dir / 'plot.svg'}")
    for warning in summary.get("warnings", []):
        print(f"warning  {warning}")
    if timed_out:
        raise RoseError(f"RoSE run exceeded {timeout_s:g}s; partial artifacts are in {run_dir}")
    if run_failure is not None:
        raise RoseError(f"{run_failure}\npartial artifacts are in {run_dir}")
    if not summary["execution_ok"]:
        raise RoseError(f"RoSE execution did not complete; see {run_dir}")
    return run_dir, summary
