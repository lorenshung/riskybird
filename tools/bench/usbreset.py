#!/usr/bin/env python3
"""Reset one USB device by its topology location, e.g. 7-1.1.4.1.

Why this exists: only the first OpenOCD run after an FTDI adapter enumerates is
trustworthy on this bench. After that the MPSSE engine can be left in a state
where the JTAG scan returns all zeroes or all ones, which reads exactly like a
broken cable or an unconfigured part. A USB-level reset clears it; nothing
softer reliably does.

Two implementations, because the mechanism is not portable:

  Linux   the USBDEVFS_RESET ioctl on /dev/bus/usb/<bus>/<dev>, addressed
          through /sys/bus/usb/devices/<location>.
  other   libusb's device reset, via pyusb, matching the same bus-and-port
          path. macOS has no usbfs and no sysfs, so this is the only way
          there.

The location argument is identical on both: it is what `rb probe-ftdi` prints
and what OpenOCD's `adapter usb location` matches.
"""

import sys
from pathlib import Path

USBDEVFS_RESET = ord("U") << (4 * 2) | 20


def reset_linux(location: str) -> str:
    import fcntl
    import os

    base = Path("/sys/bus/usb/devices") / location
    if not base.is_dir():
        raise SystemExit(
            f"no USB device at location {location}\n"
            "Locations change when an adapter is replugged -- re-read them with:\n"
            "  rb probe-ftdi"
        )
    busnum = int((base / "busnum").read_text())
    devnum = int((base / "devnum").read_text())
    node = f"/dev/bus/usb/{busnum:03d}/{devnum:03d}"
    fd = os.open(node, os.O_WRONLY)
    try:
        fcntl.ioctl(fd, USBDEVFS_RESET, 0)
    finally:
        os.close(fd)
    return f"reset {node} ({location})"


def reset_libusb(location: str) -> str:
    try:
        import usb.core
    except ImportError:
        raise SystemExit(
            "this host has no /sys/bus/usb, so the reset needs pyusb:\n"
            "  brew install libusb && pip install pyusb"
        )

    # "7-1.1.4.1" is bus 7, then the chain of hub ports. libusb reports both,
    # and reports them the same way OpenOCD's own libusb backend does, so the
    # string that works for --usb-location works here.
    bus_text, _, ports_text = location.partition("-")
    want_bus = int(bus_text)
    want_ports = tuple(int(p) for p in ports_text.split(".")) if ports_text else ()

    for device in usb.core.find(find_all=True):
        if device.bus != want_bus:
            continue
        if tuple(getattr(device, "port_numbers", None) or ()) != want_ports:
            continue
        device.reset()
        return f"reset bus {device.bus} device {device.address} ({location})"
    raise SystemExit(f"no USB device at location {location}")


def main() -> int:
    if len(sys.argv) == 2 and sys.argv[1] in ("-h", "--help"):
        print(__doc__.strip())
        return 0
    if len(sys.argv) != 2:
        print(f"usage: {Path(sys.argv[0]).name} <usb-location>", file=sys.stderr)
        print("  e.g. 7-1.1.4.1   (as printed by: rb probe-ftdi)", file=sys.stderr)
        return 2
    location = sys.argv[1]
    if Path("/sys/bus/usb/devices").is_dir():
        print(reset_linux(location))
    else:
        print(reset_libusb(location))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
