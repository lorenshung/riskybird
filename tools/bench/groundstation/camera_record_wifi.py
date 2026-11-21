#!/usr/bin/env python3
"""
WiFi camera-clip recorder for the ground station — record-on-command over the EspLink loader.

The drone's camera_record workload (zcs_fcdronet/samples/camera_record) runs on the FPGA Rocket,
captures HM01B0 frames to DDR via OSPI DMA, and sits in a record-on-command loop. This host side
drives it over the ESP loader link (socket://192.168.4.1:14552) using RBPOKE1 — a NO-RESET TSI
session (the program keeps running, no SoC reset, no bootrom memtest to clobber frames):

  1. ensure the recorder is loaded (RBPOKE probe; RBLOAD the ELF if not synced), then
  2. RBPOKE-write the command block: cmd[1]=N frames, cmd[0]=CMD_REQ_MAGIC,
  3. poll rec_seq (cmd[2]) until it increments => a new clip is ready,
  4. RBPOKE-read the header (geometry) and the N frame slots straight out of DDR.

Needs pyserial + pyelftools (the same as esp_tsi_load, which it reuses); numpy + pillow only for the
optional de-Bayer/GIF. Over TCP there is no DTR/RTS, so closing a connection does NOT reset the ESP
and tsi_synced persists across RBPOKE sessions — so after one RBLOAD you can record repeatedly.

Standalone (run on a host joined to the drone SoftAP):
    python camera_record_wifi.py --url socket://192.168.4.1:14552 \
        --elf /home/cobble/Tools/rb_relay_build/camera_record/zephyr/zephyr.elf --secs 1 --out clip
"""
import argparse
import os
import struct
import sys
import time
import types

# --- drone DDR layout (must match camera_record/src/main.c) ----------------------------------
HDRBASE = 0xB0000000          # header: [0]magic [1]n_ok [2]w [3]h [4]slot_stride [6]stage [20]N [23]rec_seq
CMDBASE = 0xB0001000          # cmd:    [0]REQ magic [1]N frames [2]rec_seq echo
FRAMEBASE = 0x82000000        # frame f at FRAMEBASE + f*slot_stride, w*h bytes (DMA, no cache flush needed)
HDR_MAGIC = 0xCA3EDA7A
CMD_REQ_MAGIC = 0xC0DE0001
DEFAULT_FPS = 30
MAX_FRAMES = 45               # recorder MAX_REC_FRAMES

# Locate esp_tsi_load.py (rb CLI tree). Override with $ESP_TSI_LOAD.
_ESP_CANDIDATES = [
    os.environ.get("ESP_TSI_LOAD", ""),
    os.path.expanduser("~/Tools/riskybird_pr65/tools/rb/esp_tsi_load.py"),
    os.path.expanduser("~/Tools/riskybird/tools/rb/esp_tsi_load.py"),
]


def _import_esp_tsi():
    for p in _ESP_CANDIDATES:
        if p and os.path.exists(p):
            sys.path.insert(0, os.path.dirname(os.path.abspath(p)))
            import esp_tsi_load as E       # noqa: E402
            return E
    # last resort: maybe already importable
    import esp_tsi_load as E               # noqa: E402
    return E


def _u32(b):
    return struct.unpack("<I", b)[0]


def _open_retry(E, url, timeout, tries=20, delay=0.5, log=lambda m: None):
    """open_link, retrying the flapping ESP TCP :14552 listener (ConnectionRefused)."""
    args = types.SimpleNamespace(url=url, usb=None, timeout=timeout)
    last = None
    for i in range(tries):
        try:
            return E.open_link(args)
        except Exception as exc:
            last = exc
            if "refused" not in str(exc).lower() and "Connection" not in str(exc):
                raise
            if i == 0 or (i + 1) % 5 == 0:
                log(f"  :14552 not up yet (try {i + 1}/{tries}) ...")
            time.sleep(delay)
    raise last


def _poke_session(E, url, timeout, log=lambda m: None):
    """Open a NO-RESET RBPOKE session. Raises if the recorder is not loaded/synced."""
    ser = _open_retry(E, url, timeout, log=log)
    E.handshake(ser, timeout, "RBPOKE1", "RBPOKE")
    return ser, E.Tsi(ser, timeout)


def _rbload(E, url, elf, timeout, log=lambda m: None):
    """Fresh RBLOAD of the recorder ELF (resets the SoC, parks hart0, sets tsi_synced)."""
    from elftools.elf.elffile import ELFFile
    ser = _open_retry(E, url, timeout, log=log)
    try:
        tsi = E.Tsi(ser, timeout)
        E.handshake(ser, timeout, "RBLOAD1", "RBLOAD")
        for addr, data in E.segments(elf):
            for off in range(0, len(data), E.CHUNK):
                tsi.write(addr + off, data[off:off + E.CHUNK])
        with open(elf, "rb") as f:
            entry = ELFFile(f)["e_entry"]
        tsi.write(E.BOOT_ADDR_REG, struct.pack("<Q", entry))
        tsi.write(E.CLINT_MSIP0, struct.pack("<I", 1))
        tsi.end()
    finally:
        ser.close()      # over TCP this does NOT reset the ESP, so tsi_synced persists


def record_clip(url, elf, n_frames, timeout=8.0, settle_s=8.0,
                poll_s=25.0, log=lambda m: None):
    """Record n_frames and return (w, h, [raw frame bytes]). Loads the recorder if needed."""
    E = _import_esp_tsi()
    n_frames = max(1, min(int(n_frames), MAX_FRAMES))
    ser = tsi = None
    try:
        try:
            ser, tsi = _poke_session(E, url, timeout, log=log)
            log("recorder already running (RBPOKE synced)")
        except Exception as exc:
            log(f"RBPOKE not ready ({exc}); loading recorder ELF ...")
            _rbload(E, url, elf, timeout, log=log)
            log(f"RBLOAD done; waiting {settle_s:.0f}s for camera/AE settle")
            time.sleep(settle_s)
            ser, tsi = _poke_session(E, url, timeout, log=log)

        seq0 = _u32(tsi.read(CMDBASE + 8, 4))
        tsi.write(CMDBASE + 4, struct.pack("<I", n_frames))     # N first
        tsi.write(CMDBASE + 0, struct.pack("<I", CMD_REQ_MAGIC))  # then the request
        log(f"requested {n_frames} frames (seq0={seq0}); waiting for capture ...")

        deadline = time.time() + poll_s
        while time.time() < deadline:
            if _u32(tsi.read(CMDBASE + 8, 4)) != seq0:
                break
            time.sleep(0.2)
        else:
            raise TimeoutError("recorder did not complete the clip in time")

        h = struct.unpack("<24I", tsi.read(HDRBASE, 96))
        if h[0] != HDR_MAGIC:
            raise RuntimeError(f"header magic {h[0]:#010x} != {HDR_MAGIC:#010x}")
        w, ht, stride, N = h[2], h[3], h[4], h[20]
        log(f"clip ready: {N} frames {w}x{ht}, stride {stride:#x}, rec_seq={h[23]}")
        frames = []
        for f in range(N):
            frames.append(tsi.read(FRAMEBASE + f * stride, w * ht))
            log(f"  read frame {f + 1}/{N}")
        return w, ht, frames
    finally:
        if tsi is not None:
            try:
                tsi.end()
            except Exception:
                pass
        if ser is not None:
            ser.close()


# --- optional host-side de-Bayer + GIF (needs numpy + pillow) ---------------------------------
def debayer_rgb(frame, w, h, order="GBRG", black=11.0):
    """2x2 GBRG -> RGB (w/2 x h/2), simple grey-world WB + sRGB. Returns a PIL image at (w,h)."""
    import numpy as np
    from PIL import Image
    g = np.frombuffer(frame, np.uint8)[:w * h].reshape(h, w).astype(np.float32)
    # GBRG: r0c0=G r0c1=B r1c0=R r1c1=G
    G = (g[0::2, 0::2] + g[1::2, 1::2]) / 2.0
    B = g[0::2, 1::2]
    R = g[1::2, 0::2]
    R = np.clip(R - black, 0, None); G = np.clip(G - black, 0, None); B = np.clip(B - black, 0, None)
    mg = max(G.mean(), 1.0)
    R *= mg / max(R.mean(), 1.0); B *= mg / max(B.mean(), 1.0)
    rgb = np.stack([R, G, B], -1)
    rgb = np.clip((rgb / max(rgb.max(), 1.0)) ** (1 / 2.2) * 255, 0, 255).astype(np.uint8)
    return Image.fromarray(rgb).resize((w, h), Image.NEAREST)


def make_gif(frames, w, h, path, fps=DEFAULT_FPS, order="GBRG"):
    imgs = [debayer_rgb(fr, w, h, order) for fr in frames if len(fr) >= w * h]
    if not imgs:
        raise RuntimeError("no frames to encode")
    dur = max(1, int(1000 / max(fps, 1)))
    imgs[0].save(path, save_all=True, append_images=imgs[1:], duration=dur, loop=0)
    return len(imgs)


def main():
    ap = argparse.ArgumentParser(description="record a camera clip off the drone over WiFi")
    ap.add_argument("--url", default="socket://192.168.4.1:14552", help="ESP loader link")
    ap.add_argument("--elf", required=True, help="camera_record zephyr.elf (for RBLOAD if needed)")
    ap.add_argument("--secs", type=float, default=1.0, help="clip length in seconds")
    ap.add_argument("--fps", type=int, default=DEFAULT_FPS)
    ap.add_argument("--out", default="clip", help="output prefix (writes <out>.gif + <out>_NN.bin)")
    ap.add_argument("--order", default="GBRG")
    a = ap.parse_args()
    n = max(1, min(int(round(a.secs * a.fps)), MAX_FRAMES))
    print(f"recording {a.secs}s = {n} frames from {a.url}")
    w, h, frames = record_clip(a.url, a.elf, n, log=lambda m: print("  " + m))
    for i, fr in enumerate(frames):
        with open(f"{a.out}_{i:02d}.bin", "wb") as f:
            f.write(fr)
    print(f"saved {len(frames)} raw frames {w}x{h} to {a.out}_NN.bin")
    try:
        k = make_gif(frames, w, h, f"{a.out}.gif", a.fps, a.order)
        print(f"wrote {a.out}.gif ({k} frames)")
    except Exception as exc:
        print(f"(GIF skipped: {exc}; raw frames saved — de-Bayer with frame-to-colour)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
