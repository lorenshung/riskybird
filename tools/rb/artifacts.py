"""Build artifact and serial-device discovery."""

from __future__ import annotations

import os
from pathlib import Path

from .boards import ROOT, Workload, ZephyrTarget


def zephyr_build_dir(
    target: ZephyrTarget,
    workload: Workload,
    mode: str | None = None,
    ram_console: bool = False,
) -> Path:
    selected_mode = mode or workload.default_mode
    # The RAM-console build is a different image from the UART one, so it gets
    # its own tree. Sharing a directory would make every toggle of the flag a
    # silent stale-artifact hazard.
    if ram_console:
        selected_mode += "-ramconsole"
    return ROOT / "build" / "zephyr" / target.name / workload.name / selected_mode


def zephyr_elf(
    target: ZephyrTarget,
    workload: Workload,
    mode: str | None = None,
    ram_console: bool = False,
) -> Path:
    return (
        zephyr_build_dir(target, workload, mode, ram_console)
        / "zephyr"
        / "zephyr.elf"
    )


def serial_devices() -> list[Path]:
    """Every serial port that could be a board, most stable name first.

    /dev/serial/by-id is a Linux udev construct and the best answer where it
    exists, because the name carries the adapter's identity and survives a
    replug that renumbers ttyUSB.

    macOS has no by-id directory and different device names: a USB serial port
    appears as /dev/cu.usbserial-* or /dev/cu.usbmodem*. The cu.* ("call-out")
    node is the right one -- the matching tty.* node blocks on open waiting for
    carrier detect, which on a board with no modem control lines simply hangs.
    """
    by_id = Path("/dev/serial/by-id")
    if by_id.is_dir():
        return sorted(by_id.iterdir())
    devices: list[Path] = []
    for pattern in (
        "ttyUSB*", "ttyACM*",           # Linux
        "cu.usbserial*", "cu.usbmodem*",  # macOS
    ):
        devices.extend(sorted(Path("/dev").glob(pattern)))
    return devices


def select_serial_port(role: str, explicit: str | None = None) -> Path:
    if explicit:
        port = Path(explicit)
        if not port.exists():
            raise FileNotFoundError(f"serial port does not exist: {port}")
        return port

    env_name = {
        "tsi": "RB_TSI_PORT",
        "esp": "RB_ESP_PORT",
        "console": "RB_CONSOLE_PORT",
    }.get(role, "RB_CONSOLE_PORT")
    if os.environ.get(env_name):
        return select_serial_port(role, os.environ[env_name])

    devices = serial_devices()
    if not devices:
        raise FileNotFoundError(
            f"no serial devices found; pass --port or set {env_name}"
        )
    if len(devices) == 1:
        return devices[0]

    preferred_words = (
        ("FT232R", "UARTTSI", "TSI")
        if role == "tsi"
        else (
            ("USB_JTAG", "ESP32", "Espressif", "CP210", "Silicon_Labs")
            if role == "esp"
            else ("CP210", "Silicon_Labs", "console")
        )
    )
    preferred = [
        device
        for device in devices
        if any(word.lower() in device.name.lower() for word in preferred_words)
    ]
    if len(preferred) == 1:
        return preferred[0]

    choices = "\n  ".join(str(device) for device in devices)
    raise RuntimeError(
        f"multiple serial devices could be the {role} port; pass --port or set "
        f"{env_name}:\n  {choices}"
    )
