"""Configured RoSE synchronizer process launched by ``rb hil replay``."""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import threading
from pathlib import Path
from typing import Any


def _plain(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    if hasattr(value, "tolist"):
        return value.tolist()
    if hasattr(value, "item"):
        return value.item()
    return value


class TraceWriter:
    def __init__(self, path: Path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = path.open("w", encoding="utf-8", buffering=1)

    def emit(self, event: str, **values: Any) -> None:
        record = {"event": event, **{key: _plain(value) for key, value in values.items()}}
        self.stream.write(json.dumps(record, sort_keys=True) + "\n")

    def close(self) -> None:
        self.stream.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--trace", type=Path, required=True)
    args = parser.parse_args(argv)
    config = json.loads(args.config.read_text(encoding="utf-8"))

    hephaestus = Path(os.environ["ROSE_DIR"]) / "deploy" / "hephaestus"
    sys.path.insert(0, str(hephaestus))
    import gymnasium as gym  # type: ignore
    import gym_synchronizer  # type: ignore
    from socket_thread import ServerThread  # type: ignore

    trace = TraceWriter(args.trace)
    period_s = float(config["control_period_ms"]) / 1000.0

    class TracedEnvironment(gym.Wrapper):
        def __init__(self, env):
            super().__init__(env)
            self.step_count = 0

        def reset(self, **kwargs):
            result = self.env.reset(**kwargs)
            trace.emit("environment_reset", reset_index=self.step_count)
            return result

        def step(self, action):
            obs, reward, terminated, truncated, info = self.env.step(action)
            self.step_count += 1
            pos = _plain(obs.get("pos", [])) if isinstance(obs, dict) else []
            mpc_state = _plain(obs.get("mpc_state", [])) if isinstance(obs, dict) else []
            trace.emit(
                "physics_step",
                step=self.step_count,
                sim_time_s=self.step_count * period_s,
                pos=pos,
                z_error_m=(float(pos[2]) - 1.0) if len(pos) >= 3 else None,
                mpc_state=mpc_state,
                action=action,
                reward=reward,
                terminated=terminated,
                truncated=truncated,
            )
            return obs, reward, terminated, truncated, info

    class StopSimulation(Exception):
        pass

    class RiskyBirdSynchronizer(gym_synchronizer.Synchronizer):
        def load_config(self):
            self.firesim_step = int(config["step_cycles"])
            self.firesim_freq = float(config["clock_mhz"]) * 1_000_000.0
            self.cycle_limit = int(config["steps"]) * self.firesim_step
            self.render = False
            return str(config["gym_env"])

        def check_task_termination(self):
            if self.cycle_limit is not None and self.count * self.firesim_step >= self.cycle_limit:
                raise StopSimulation
            if self.done:
                raise StopSimulation

        def process_fsim_data_packet(self, target_thread):
            packet = target_thread.data_rxqueue[0]
            cmd = int(packet.cmd)
            words = _plain(packet.data)
            super().process_fsim_data_packet(target_thread)
            trace.emit(
                "bridge_packet",
                step=int(self.count),
                sim_time_s=float(self.count) * period_s,
                cmd=cmd,
                num_bytes=int(packet.num_bytes),
                words=words,
            )

    sync = RiskyBirdSynchronizer(
        host=str(config["sync_host"]),
        sync_port=int(config["sync_port"]),
        yaml_path=str(config["gym_config"]),
    )
    sync.env = TracedEnvironment(sync.env)
    # Fail before advertising readiness when the execution environment forbids
    # localhost sockets or the selected port is already occupied.  The backend
    # ServerThread otherwise reports bind failures only on its worker thread.
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind((sync.sync_host, sync.sync_port))
    finally:
        probe.close()
    condition = threading.Condition()
    server = ServerThread(sync, condition)
    server.daemon = True
    server.start()
    print(
        f"RB_ROSE_SYNC_READY host={sync.sync_host} port={sync.sync_port} "
        f"step_cycles={sync.firesim_step}",
        flush=True,
    )
    try:
        while server.connected_sockets < server.num_sockets:
            if not server.is_alive():
                raise RuntimeError("RoSE socket server stopped before accepting Spike")
            server.join(timeout=0.01)
        sync.socket_threads = sync.nodes
        sync.run()
    except StopSimulation:
        print(f"RB_ROSE_SYNC_COMPLETE steps={sync.count}", flush=True)
    finally:
        if hasattr(sync, "logger"):
            sync.logger.close()
        for node in sync.nodes:
            node.kill()
        try:
            sync.env.close()
        finally:
            trace.close()
        with condition:
            condition.notify_all()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
