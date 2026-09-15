"""Hermetic process environments for the existing backend installations."""

from __future__ import annotations

import os
import platform
import shutil
import subprocess
from pathlib import Path
from typing import Sequence

from .boards import CHIPYARD, ROOT, ZEPHYR_BACKEND


CHIPYARD_PREFIX = CHIPYARD / ".conda-env"
ZEPHYR_MINIFORGE = ZEPHYR_BACKEND / "tools" / "miniforge3"
ZEPHYR_ENVS_DIR = ZEPHYR_MINIFORGE / "envs"
ZEPHYR_ENV_PREFERENCE = ("zephyr-new", "zephyr")


def select_zephyr_prefix(envs_dir: Path = ZEPHYR_ENVS_DIR) -> Path:
    """Select an installed Zephyr environment or the clean-setup default."""
    for name in ZEPHYR_ENV_PREFERENCE:
        candidate = envs_dir / name
        if candidate.is_dir():
            return candidate
    # The backend's committed installer creates an environment named
    # ``zephyr``. Selecting that path before first setup keeps a clean clone
    # consistent without requiring a backend edit.
    return envs_dir / "zephyr"


ZEPHYR_PREFIX = select_zephyr_prefix()
ZEPHYR_BASE = ZEPHYR_BACKEND / "zephyr_ws" / "zephyr"
# The version this project pins. Kept as the preferred name so a machine with
# several SDKs installed picks the intended one.
ZEPHYR_SDK_PINNED = "zephyr-sdk-1.0.0-beta1"


def select_zephyr_sdk(manual: Path = ZEPHYR_BACKEND / "tools-manual") -> Path:
    """Locate an installed Zephyr SDK, preferring the pinned version.

    Hardcoding the version made the SDK path unreachable for anyone who had to
    install a different one, and rb would then report the SDK missing while it
    sat right there.

    This docstring used to claim the pinned v1.0.0-beta1 publishes a ZERO-BYTE
    macos-aarch64 tarball and that Apple Silicon therefore has to move to
    v1.0.0+. That is wrong. The asset is 5,908 bytes -- the real _minimal_
    package, which carries the cmake scaffolding and a setup.sh that downloads
    the toolchain itself -- and it installs correctly on the pinned version.
    Confirmed against the release metadata and by a macOS install on 2026-09-14.
    Staying on the pinned version is what keeps macOS and Linux on one toolchain.

    Prefers the pinned name, else the newest zephyr-sdk-* directory present,
    else returns the pinned path so `rb doctor` still reports what it expected.
    """
    pinned = manual / ZEPHYR_SDK_PINNED
    if pinned.is_dir():
        return pinned
    if manual.is_dir():
        found = sorted(
            (d for d in manual.iterdir() if d.is_dir() and d.name.startswith("zephyr-sdk-")),
            key=lambda d: d.name,
        )
        if found:
            return found[-1]
    return pinned


ZEPHYR_SDK = select_zephyr_sdk()
def select_host_sysroot(sdk: Path = ZEPHYR_SDK) -> Path:
    """Locate the SDK's host-tools sysroot for THIS machine.

    The directory is named for the host the SDK was built for --
    ``x86_64-pokysdk-linux`` on Linux, ``aarch64-apple-darwin`` or
    ``x86_64-apple-darwin`` on macOS -- so hardcoding the Linux name makes
    every SDK-hosted tool (OpenOCD, GDB) unreachable on a Mac.

    Discovered by looking rather than by mapping platform names: the SDK
    ships exactly one sysroot, and a future host triple then needs no change
    here. Falls back to the Linux name when the SDK is not installed yet, so
    importing this module on a fresh clone still works and `rb doctor` can
    report the path it expected.
    """
    sysroots = sdk / "hosttools" / "sysroots"
    if sysroots.is_dir():
        found = sorted(entry for entry in sysroots.iterdir() if entry.is_dir())
        if found:
            for entry in found:
                if platform.system() == "Darwin" and "darwin" in entry.name:
                    return entry
                if platform.system() == "Linux" and "linux" in entry.name:
                    return entry
            return found[0]
    return sysroots / "x86_64-pokysdk-linux"


ZEPHYR_HOST_SYSROOT = select_host_sysroot()
ZEPHYR_HOST_BIN = ZEPHYR_HOST_SYSROOT / "usr" / "bin"
# Only meaningful on Linux, and only for an SDK that was MOVED after install:
# a relocated SDK keeps a stale ELF interpreter path in its host binaries, so
# they have to be launched through the loader explicitly. macOS has no
# equivalent (and needs none), and a cleanly installed Linux SDK runs its own
# binaries directly. openocd_command() below decides at call time.
ZEPHYR_HOST_LOADER = ZEPHYR_HOST_SYSROOT / "lib" / "ld-linux-x86-64.so.2"
ZEPHYR_HOST_LIBRARY_PATH = os.pathsep.join(
    (
        str(ZEPHYR_HOST_SYSROOT / "lib"),
        str(ZEPHYR_HOST_SYSROOT / "usr" / "lib"),
    )
)
ZEPHYR_GNU_BIN = ZEPHYR_SDK / "gnu" / "riscv64-zephyr-elf" / "bin"
# Site-specific and not discoverable, so it is an override rather than a
# constant. Silicon-tier only: nothing in the bench flow touches Vivado.
VIVADO_BIN = Path(os.environ.get("RB_VIVADO_BIN", "/ecad/tools/xilinx/Vivado/2023.1/bin"))
# /opt/homebrew/bin is where Homebrew installs on Apple Silicon; Intel Macs and
# most Linux distributions use /usr/local/bin. Listing both costs nothing and is
# how openFPGALoader, dtc and a Homebrew python get found on a Mac.
SYSTEM_PATHS = (
    Path("/opt/homebrew/bin"),
    Path("/usr/local/bin"),
    Path("/usr/bin"),
    Path("/bin"),
)

_ACTIVATION_KEYS = (
    "CCACHE_DIR",
    "CCACHE_TEMPDIR",
    "CMAKE_PREFIX_PATH",
    "CONDA_DEFAULT_ENV",
    "CONDA_EXE",
    "CONDA_PREFIX",
    "CONDA_PROMPT_MODIFIER",
    "CONDA_PYTHON_EXE",
    "CONDA_SHLVL",
    "CPATH",
    "CPLUS_INCLUDE_PATH",
    "C_INCLUDE_PATH",
    "DYLD_LIBRARY_PATH",
    "JAVA_HOME",
    "JAVA_LD_LIBRARY_PATH",
    "LD_LIBRARY_PATH",
    "LIBRARY_PATH",
    "PKG_CONFIG_PATH",
    "PYTHONHOME",
    "PYTHONPATH",
    "RISCV",
    "VIRTUAL_ENV",
    "WEST_TOPDIR",
    "XDG_RUNTIME_DIR",
    "ZEPHYR_BASE",
    "ZEPHYR_SDK_INSTALL_DIR",
    "ZEPHYR_TOOLCHAIN_VARIANT",
)


def _clean_parent() -> dict[str, str]:
    env = dict(os.environ)
    for key in _ACTIVATION_KEYS:
        env.pop(key, None)
    env["PYTHONNOUSERSITE"] = "1"
    return env


def _path_string(paths: Sequence[Path]) -> str:
    return os.pathsep.join(str(path) for path in paths if path.is_dir())


def zephyr_env() -> dict[str, str]:
    (ROOT / ".rb" / "ccache").mkdir(parents=True, exist_ok=True)
    (ROOT / ".rb" / "ccache-tmp").mkdir(parents=True, exist_ok=True)
    env = _clean_parent()
    env.update(
        {
            "CONDA_DEFAULT_ENV": ZEPHYR_PREFIX.name,
            "CONDA_PREFIX": str(ZEPHYR_PREFIX),
            "RB_ENV": "zephyr",
            "ZEPHYR_BASE": str(ZEPHYR_BASE),
            "ZEPHYR_SDK_INSTALL_DIR": str(ZEPHYR_SDK),
            "ZEPHYR_TOOLCHAIN_VARIANT": "zephyr",
            "OPENOCD_SCRIPTS": str(
                ZEPHYR_HOST_SYSROOT / "usr" / "share" / "openocd" / "scripts"
            ),
            # Backend-local and user-global caches are not assumed writable.
            "CCACHE_DIR": str(ROOT / ".rb" / "ccache"),
            "CCACHE_TEMPDIR": str(ROOT / ".rb" / "ccache-tmp"),
            "PATH": _path_string(
                (
                    ZEPHYR_PREFIX / "bin",
                    ZEPHYR_GNU_BIN,
                    *SYSTEM_PATHS,
                    # This SDK was moved from another checkout, so some of its
                    # host binaries retain a stale ELF interpreter. Prefer
                    # system build tools; rb invokes SDK OpenOCD explicitly.
                    ZEPHYR_HOST_BIN,
                )
            ),
        }
    )
    return env


def openocd_command(*arguments: str | Path) -> list[str]:
    """Build the command that runs OpenOCD, preferring the SDK's own binary.

    The ELF-loader form is a workaround for an SDK that was MOVED after
    installation: its host binaries keep the absolute interpreter path from
    where they were built, so they cannot exec themselves. That is a property
    of this bench's history, not of the SDK -- a clean install runs the binary
    directly, and macOS has no ELF loader at all.

    So: use the loader only when it is actually present, else exec the SDK
    binary, else fall back to an OpenOCD on PATH (Homebrew's, typically).
    Anything else would make a correct clean install fail with a confusing
    "no such file or directory" naming the loader rather than OpenOCD.
    """
    sdk_openocd = ZEPHYR_HOST_BIN / "openocd"
    if ZEPHYR_HOST_LOADER.is_file() and sdk_openocd.is_file():
        return [
            str(ZEPHYR_HOST_LOADER),
            "--library-path",
            ZEPHYR_HOST_LIBRARY_PATH,
            str(sdk_openocd),
            *(str(argument) for argument in arguments),
        ]
    if sdk_openocd.is_file():
        return [str(sdk_openocd), *(str(argument) for argument in arguments)]
    return ["openocd", *(str(argument) for argument in arguments)]


def chipyard_env() -> dict[str, str]:
    runtime_dir = ROOT / ".rb" / "runtime" / "chipyard"
    java_tmp = ROOT / ".rb" / "tmp" / "chipyard"
    runtime_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
    runtime_dir.chmod(0o700)
    java_tmp.mkdir(parents=True, exist_ok=True)
    env = _clean_parent()
    riscv = CHIPYARD_PREFIX / "riscv-tools"
    java_home = CHIPYARD_PREFIX / "lib" / "jvm"
    sbt_options = " ".join(
        (
            f"-Dsbt.ivy.home={CHIPYARD / '.ivy2'}",
            f"-Dsbt.global.base={CHIPYARD / '.sbt'}",
            f"-Dsbt.boot.directory={CHIPYARD / '.sbt' / 'boot'}",
            "-Dsbt.color=always",
            "-Dsbt.supershell=false",
            # If a managed build sandbox rejects SBT's Unix-domain boot
            # socket, continue the one-shot Make flow without that socket.
            "-Dsbt.server.forcestart=true",
        )
    )
    env.update(
        {
            "CONDA_DEFAULT_ENV": str(CHIPYARD_PREFIX),
            "CONDA_PREFIX": str(CHIPYARD_PREFIX),
            "CY_DIR": str(CHIPYARD),
            "RB_ENV": "chipyard",
            # SBT's boot server otherwise follows the host XDG runtime path,
            # which is commonly read-only inside managed build sessions.
            "XDG_RUNTIME_DIR": str(runtime_dir),
            "JAVA_TOOL_OPTIONS": f"-Xmx8G -Xss8M -Djava.io.tmpdir={java_tmp}",
            "SBT_OPTS": sbt_options,
            "RISCV": str(riscv),
            "JAVA_HOME": str(java_home),
            "JAVA_LD_LIBRARY_PATH": str(java_home / "lib" / "server"),
            "LD_LIBRARY_PATH": str(riscv / "lib"),
            "GSETTINGS_SCHEMA_DIR": str(
                CHIPYARD_PREFIX / "share" / "glib-2.0" / "schemas"
            ),
            "CMAKE_PREFIX_PATH": str(CHIPYARD_PREFIX),
            "PKG_CONFIG_PATH": str(CHIPYARD_PREFIX / "lib" / "pkgconfig"),
            "PATH": _path_string(
                (
                    CHIPYARD_PREFIX / "bin",
                    riscv / "bin",
                    VIVADO_BIN,
                    *SYSTEM_PATHS,
                )
            ),
        }
    )
    return env


def host_env() -> dict[str, str]:
    env = _clean_parent()
    env["PATH"] = _path_string((VIVADO_BIN, *SYSTEM_PATHS))
    return env


def executable(environment: str, name: str) -> Path | None:
    env = {
        "chipyard": chipyard_env,
        "zephyr": zephyr_env,
        "host": host_env,
    }[environment]()
    found = shutil.which(name, path=env["PATH"])
    return Path(found) if found else None


def run(
    command: Sequence[str | Path],
    *,
    environment: str,
    cwd: Path,
    check: bool = True,
) -> subprocess.CompletedProcess[str]:
    env_fn = {
        "chipyard": chipyard_env,
        "zephyr": zephyr_env,
        "host": host_env,
    }[environment]
    rendered = [str(part) for part in command]
    print("+", " ".join(rendered), flush=True)
    return subprocess.run(rendered, cwd=cwd, env=env_fn(), check=check, text=True)


def shell(environment: str) -> int:
    env_fn = {"chipyard": chipyard_env, "zephyr": zephyr_env}[environment]
    env = env_fn()
    env["PS1"] = f"(rb:{environment}) \\u@\\h:\\w\\$ "
    return subprocess.call(
        ["/bin/bash", "--noprofile", "--norc"],
        cwd=CHIPYARD if environment == "chipyard" else ZEPHYR_BACKEND,
        env=env,
    )
