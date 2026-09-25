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


ESPRESSIF_VENDOR = 0x303A


def macos_location_prefix(bus: int, ports) -> str:
    """The locationID macOS builds a serial node name from, as that name spells it.

    locationID packs the libusb bus number into the top byte and one port per nibble
    below it; the node name is its hex with the trailing zeros dropped. Measured on the
    drone bench Mac: bus 1 ports 1.3 -> 0x01130000 -> "113" (cu.usbserial-11300/-11301),
    bus 0 port 1 -> 0x00100000 -> "1" (the ESP32-C6's cu.usbmodem101).
    """
    location = bus << 24
    for i, port in enumerate(list(ports)[:6]):
        location |= (port & 0xF) << (20 - 4 * i)
    return f"{location:x}".rstrip("0")


def macos_espressif_ports() -> list[Path]:
    """cu.usbmodem nodes that belong to an attached Espressif USB device (VID 0x303a).

    On macOS the node name carries no vendor string, so "which usbmodem is the ESP" is
    answered from USB instead: the device's bus/port chain gives the name prefix. Empty
    without pyusb or without such a device -- callers fall back to asking for --port.
    """
    try:
        import usb.core  # type: ignore
    except ImportError:
        return []
    found: list[Path] = []
    try:
        devices = list(usb.core.find(find_all=True, idVendor=ESPRESSIF_VENDOR))
    except Exception:
        return []
    for device in devices:
        ports = getattr(device, "port_numbers", None) or ()
        if not ports:
            continue
        prefix = macos_location_prefix(device.bus, ports)
        found.extend(sorted(Path("/dev").glob(f"cu.usbmodem{prefix}[0-9][0-9]")))
    return found


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
    if role == "esp" and not preferred:
        # macOS: no by-id names, so identify the ESP by its USB vendor instead.
        espressif = [p for p in macos_espressif_ports() if p in devices]
        if len(espressif) == 1:
            return espressif[0]

    choices = "\n  ".join(str(device) for device in devices)
    raise RuntimeError(
        f"multiple serial devices could be the {role} port; pass --port or set "
        f"{env_name}:\n  {choices}"
    )
