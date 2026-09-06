"""Capture an interactive serial session with reproducibility metadata."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import selectors
import subprocess
import sys
import termios
import time
import tty
from datetime import datetime, timezone
from pathlib import Path

import serial


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _git_state(path: Path) -> dict[str, object]:
    def git(*arguments: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            ["git", "-C", str(path), *arguments],
            capture_output=True,
            check=False,
            text=True,
        )

    revision = git("rev-parse", "HEAD")
    status = git("status", "--porcelain", "--untracked-files=no")
    return {
        "path": str(path),
        "revision": revision.stdout.strip() if revision.returncode == 0 else None,
        "tracked_changes": bool(status.stdout.strip()) if status.returncode == 0 else None,
    }


def _write_json(path: Path, value: dict[str, object]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
    temporary.replace(path)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--target", required=True)
    parser.add_argument("--workload", required=True)
    parser.add_argument("--mode", required=True)
    parser.add_argument("--board")
    parser.add_argument("--port", type=Path, required=True)
    parser.add_argument("--baudrate", type=int, required=True)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--elf", type=Path, required=True)
    return parser


def main() -> int:
    args = _parser().parse_args()
    if args.duration < 0:
        raise SystemExit("--duration must be zero or positive")
    if not args.elf.is_file():
        raise SystemExit(f"firmware ELF does not exist: {args.elf}")

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=False)
    serial_log = output_dir / "serial.log"
    input_log = output_dir / "operator-input.bin"
    metadata_path = output_dir / "run.json"

    repository = args.root.resolve()
    metadata: dict[str, object] = {
        "schema_version": 1,
        "status": "capturing",
        "started_utc": _utc_now(),
        "target": args.target,
        "board": args.board,
        "workload": args.workload,
        "mode": args.mode,
        "execution_environment": "physical-serial",
        "serial": {
            "port": str(args.port.resolve()),
            "baudrate": args.baudrate,
            "requested_duration_s": args.duration or None,
        },
        "firmware": {
            "elf": str(args.elf.resolve()),
            "size_bytes": args.elf.stat().st_size,
            "sha256": _sha256(args.elf),
        },
        "revisions": {
            "riskybird": _git_state(repository),
            "chipyard": _git_state(repository / "backends" / "chipyard"),
            "zephyr_chipyard_sw": _git_state(
                repository / "backends" / "zephyr-chipyard-sw"
            ),
        },
    }
    _write_json(metadata_path, metadata)

    print(f"capturing {args.port} at {args.baudrate} baud")
    print(f"results: {output_dir}")
    print("Ctrl-] ends the capture; typed bytes are forwarded to the target.")

    started = time.monotonic()
    received = 0
    transmitted = 0
    original_terminal: list[object] | None = None
    selector = selectors.DefaultSelector()
    stdin_fd = sys.stdin.fileno()
    if sys.stdin.isatty():
        original_terminal = termios.tcgetattr(stdin_fd)
        tty.setcbreak(stdin_fd)
        selector.register(stdin_fd, selectors.EVENT_READ)

    result_status = "completed"
    error: str | None = None
    try:
        with (
            serial.Serial(str(args.port), args.baudrate, timeout=0.05) as device,
            serial_log.open("wb") as output,
            input_log.open("wb") as sent,
        ):
            while args.duration == 0 or time.monotonic() - started < args.duration:
                block = device.read(max(device.in_waiting, 1))
                if block:
                    output.write(block)
                    output.flush()
                    sys.stdout.buffer.write(block)
                    sys.stdout.buffer.flush()
                    received += len(block)

                for key, _ in selector.select(timeout=0):
                    typed = os.read(key.fd, 1024)
                    if b"\x1d" in typed:
                        typed = typed.split(b"\x1d", 1)[0]
                        if typed:
                            device.write(typed)
                            sent.write(typed)
                            transmitted += len(typed)
                        raise KeyboardInterrupt
                    if typed:
                        device.write(typed)
                        sent.write(typed)
                        sent.flush()
                        transmitted += len(typed)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        result_status = "failed"
        error = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        if original_terminal is not None:
            termios.tcsetattr(stdin_fd, termios.TCSADRAIN, original_terminal)
        selector.close()
        elapsed = time.monotonic() - started
        metadata.update(
            {
                "status": result_status,
                "completed_utc": _utc_now(),
                "elapsed_s": elapsed,
                "received_bytes": received,
                "transmitted_bytes": transmitted,
                "serial_log": {
                    "path": str(serial_log),
                    "sha256": _sha256(serial_log) if serial_log.is_file() else None,
                },
                "operator_input": {
                    "path": str(input_log),
                    "sha256": _sha256(input_log) if input_log.is_file() else None,
                },
                "error": error,
            }
        )
        _write_json(metadata_path, metadata)
        print(f"\ncapture {result_status}: {metadata_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
