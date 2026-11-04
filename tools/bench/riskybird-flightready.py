#!/usr/bin/env python3
"""
RiskyBird flight-ready monitor: owns UDP :14550, logs everything, relays to the panel,
and prints a live GO / NO-GO against the pre-flight checklist.

    python3 tools/bench/riskybird-flightready.py --relay 14560
    (normally started by tools/bench/riskybird-ready.sh, which also starts the panel)

WHY THIS EXISTS
---------------
Only one socket can receive the drone's unicast on :14550. With the v3 panel holding it,
RBLINK (the ESP's own health line) was invisible, because the panel does not parse it,
and every check meant stopping the panel to run `verify`. This process takes the port
instead and:

  * writes every datagram, FC lines AND RBLINK, to raw_<stamp>.log (host timestamped),
  * forwards each datagram unchanged to 127.0.0.1:<relay> for the panel
    (run the panel with --telem-port <relay>),
  * prints one status line a second and keeps ready-status.json up to date,
  * tells a STALE LEASE (drone answers ping, zero datagrams) apart from DRONE OFF AIR
    (no ping). A stale lease is what an ESP reboot leaves behind: the Mac keeps its old
    DHCP lease, the ESP's fresh DHCP server has no client to unicast to, and telemetry
    silently goes nowhere while PING still works.

RECEIVE-ONLY TOWARDS THE DRONE. Nothing here ever sends to the drone. The only thing it
transmits is the local relay copy to 127.0.0.1. ICMP ping is used only for the
stale-lease diagnosis. Commands stay on the panel's buttons.

--auto-rejoin (off by default): on a STALE LEASE, run `riskybird-wifi-macos.sh down`,
then `up` if en0 did not rejoin by itself. That is the same radio cycle `up` does, and
`up` sends its usual PING. While the drone is off the air it also retries `up` every 30 s,
and each retry switches the Mac's Wi-Fi, so leave it off unless that is wanted.
"""

import argparse
import atexit
import datetime
import json
import os
import re
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
WIFI = os.path.join(HERE, "riskybird-wifi-macos.sh")
DRONE = "192.168.4.1"

KV_RE = re.compile(r"(\w+)=(-?[0-9.]+)")
TILT_RE = re.compile(r"tilt=est(-?\d+)deg/acc(-?\d+)deg")
DUTY_RE = re.compile(r"duty=\[([^\]]*)\]")
FL_RE = re.compile(r"fl=0x([0-9a-fA-F]+)\[([^\]]*)\]")
ML_RE = re.compile(r"ml=(\w+)")

# st= bits (main.cpp TELEM_FLAG_*)
ARMED, ESTOP, ARMING, CALDONE = 1, 2, 4, 8

# Checklist thresholds. Change them here, not at the call sites.
MAX_TILT_DEG = 3
MIN_FC_HZ = 5.0
MIN_RB_HZ = 3.0
MAX_GAP_MS = 90
MAX_Z_REST = 0.05
MIN_VBAT_WARN = 3.8        # telemetry vbat is unmetered; this is a hint, not a gate
STALE_S = 3.0              # older than this, last-seen values are dropped from the status


def parse_fc(line):
    d = {k: float(v) for k, v in KV_RE.findall(line)}
    m = TILT_RE.search(line)
    if m:
        d["tilt_est"], d["tilt_acc"] = int(m.group(1)), int(m.group(2))
    m = DUTY_RE.search(line)
    if m:
        d["duty"] = [float(x) for x in m.group(1).split()]
    return d


def parse_rb(line):
    d = {k: float(v) for k, v in KV_RE.findall(line)}
    m = FL_RE.search(line)
    if m:
        d["fl"], d["fl_txt"] = int(m.group(1), 16), m.group(2).strip()
    m = ML_RE.search(line)
    if m:
        d["ml"] = m.group(1)
    m = DUTY_RE.search(line)
    if m:
        d["duty"] = [int(x) for x in m.group(1).split()]
    return d


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.fc = None
        self.rb = None
        self.t_fc = deque(maxlen=200)
        self.t_rb = deque(maxlen=60)
        self.gaps = deque(maxlen=25)     # RBLINK gap= over the last ~5 s
        self.crc = deque(maxlen=25)      # (t, crc) to spot a climbing counter
        self.last_any = None
        self.other = ""
        self.n = 0


S = State()


def udp_source(port):
    """Datagrams from a normal socket bound to :port."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", port))
    while True:
        data, _ = s.recvfrom(4096)
        yield time.time(), data


def pcap_source(iface, port):
    """Datagrams to :port captured on iface by tcpdump, without a socket.

    For a host whose firewall rejects the drone's unicast (it answers ICMP
    "admin prohibited" and no socket ever sees the datagram): libpcap sees a
    packet at the interface, before the firewall. Nothing on the host changes.
    tcpdump needs root; it is run through `sudo -n`, so sudo must not prompt.
    Timestamps are the kernel's receive times from the capture.
    """
    cmd = ["tcpdump", "-i", iface, "-n", "-U", "-w", "-", "udp", "dst", "port", str(port)]
    if os.geteuid() != 0:
        cmd = ["sudo", "-n"] + cmd
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    f = proc.stdout

    def stop_capture(*_):
        # sudo relays SIGTERM to tcpdump; without this a killed monitor leaves a
        # root tcpdump running until its next write hits the closed pipe.
        if proc.poll() is None:
            proc.terminate()

    atexit.register(stop_capture)

    def take(n):
        buf = b""
        while len(buf) < n:
            chunk = f.read(n - len(buf))
            if not chunk:
                raise SystemExit(f"[ready] tcpdump on {iface} stopped (exit {proc.poll()}) -- "
                                 "is sudo passwordless and the interface up?")
            buf += chunk
        return buf

    gh = take(24)
    magic = struct.unpack("<I", gh[:4])[0]
    if magic in (0xA1B2C3D4, 0xA1B23C4D):
        end = "<"
    elif magic in (0xD4C3B2A1, 0x4D3CB2A1):
        end = ">"
    else:
        raise SystemExit(f"[ready] not a pcap stream from tcpdump (magic {magic:#x})")
    nano = magic in (0xA1B23C4D, 0x4D3CB2A1)
    link = struct.unpack(end + "I", gh[20:24])[0]
    l2 = {1: 14, 113: 16, 276: 20}.get(link)   # Ethernet, Linux cooked, Linux cooked v2
    if l2 is None:
        raise SystemExit(f"[ready] unsupported capture link type {link}")
    while True:
        sec, frac, incl, _orig = struct.unpack(end + "IIII", take(16))
        pkt = take(incl)
        ts = sec + frac / (1e9 if nano else 1e6)
        off = l2
        if link == 1 and pkt[12:14] == b"\x81\x00":   # 802.1Q tag
            off += 4
        if len(pkt) < off + 28 or (pkt[off] >> 4) != 4:
            continue
        ihl = (pkt[off] & 0x0F) * 4
        if pkt[off + 9] != 17:                      # not UDP
            continue
        udp = off + ihl
        ulen = struct.unpack(">H", pkt[udp + 4:udp + 6])[0]
        yield ts, pkt[udp + 8:udp + ulen]


def rx(source, relay, logf):
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if relay else None
    for now, data in source:
        if out:
            out.sendto(data, ("127.0.0.1", relay))
        text = data.decode("utf-8", "replace")
        for line in text.splitlines():
            logf.write("%.3f %s\n" % (now, line))
            with S.lock:
                S.last_any = now
                S.n += 1
                if "RBLINK" in line:
                    d = parse_rb(line)
                    S.rb = d
                    S.t_rb.append(now)
                    if "gap" in d:
                        S.gaps.append(d["gap"])
                    if "crc" in d:
                        S.crc.append((now, d["crc"]))
                elif "it=" in line and "roll=" in line:
                    S.fc = parse_fc(line)
                    S.t_fc.append(now)
                elif line.strip():
                    S.other = line.strip()[:80]
        # Every datagram: a brownout takes the drone down mid-line, and the lines just
        # before it are the ones that matter. Batching lost ~2 s of them once.
        logf.flush()


def rate(ts, now, window=3.0):
    return sum(1 for t in ts if now - t <= window) / window


def ping_ok():
    return subprocess.call(["ping", "-c", "1", "-t", "1", DRONE],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0


def evaluate(now):
    """Return (verdict, reasons, warnings, summary dict)."""
    no, warn = [], []
    with S.lock:
        fc, rb = S.fc, S.rb
        fc_hz, rb_hz = rate(S.t_fc, now), rate(S.t_rb, now)
        fc_age = now - S.t_fc[-1] if S.t_fc else None
        rb_age = now - S.t_rb[-1] if S.t_rb else None
        gmax = max(S.gaps) if S.gaps else None
        crc_rise = (S.crc[-1][1] - S.crc[0][1]) if len(S.crc) > 1 else 0
        other = S.other

    if fc is None or fc_age > 1.0 or fc_hz < MIN_FC_HZ:
        no.append("FC lines %.1f Hz" % fc_hz)
    if rb is None or rb_age > 1.5 or rb_hz < MIN_RB_HZ:
        no.append("RBLINK %.1f Hz" % rb_hz)
    # Past STALE_S the last values describe a drone that may since have been moved,
    # reset or reflashed. Drop them rather than print them as current: a stale
    # "tilt 75" after a link loss read like a live reading.
    if fc_age is not None and fc_age > STALE_S:
        fc = None
    if rb_age is not None and rb_age > STALE_S:
        rb, gmax, crc_rise = None, None, 0

    if fc:
        st = int(fc.get("st", 0))
        if int(fc.get("tofv", 0)) != 1:
            no.append("tofv=%d" % fc.get("tofv", -1))
        if abs(fc.get("z", 0)) > MAX_Z_REST:
            no.append("z=%.3f (drift?)" % fc["z"])
        t = fc.get("tilt_est")
        if t is None or abs(t) > MAX_TILT_DEG:
            no.append("tilt %s deg > %d" % (t, MAX_TILT_DEG))
        if not st & CALDONE:
            no.append("gyro cal not done")
        if st & ESTOP:
            no.append("FC ESTOP latched")
        if st & (ARMED | ARMING):
            warn.append("FC ARMED/ARMING")
        if not (0.02 <= fc.get("tofh", 0) <= 0.04):
            warn.append("tofh=%.3f" % fc.get("tofh", 0))
        if fc.get("vbat", 9) < MIN_VBAT_WARN:
            warn.append("vbat=%.2f" % fc["vbat"])
    if rb:
        if rb.get("ml") != "up":
            no.append("ml=%s" % rb.get("ml"))
        if int(rb.get("kill", 1)) != 0:
            no.append("ESP kill=1")
        if "COMMITTED" not in rb.get("fl_txt", ""):
            no.append("not COMMITTED")
        # The commit budget is spent: the FC still arms on RESET but holds zero
        # thrust (T07/T08). "COMMITTED" is still in the text, so check it on its own.
        if "WINDOW-EXPIRED" in rb.get("fl_txt", ""):
            no.append("commit WINDOW-EXPIRED (re-run flight.sh commit)")
        if int(rb.get("cl", 0)) < 1:
            no.append("cl=0")
        if any(rb.get("duty", [0])):
            warn.append("ESP duty %s" % rb.get("duty"))
        if gmax is not None and gmax >= MAX_GAP_MS:
            no.append("gap %d ms" % gmax)
        if crc_rise > 0:
            warn.append("crc +%d/5s" % crc_rise)

    summary = {
        "t": datetime.datetime.now().isoformat(timespec="seconds"),
        "fc_hz": round(fc_hz, 1), "rb_hz": round(rb_hz, 1),
        "tilt": fc.get("tilt_est") if fc else None,
        "tilt_acc": fc.get("tilt_acc") if fc else None,
        "tofv": fc.get("tofv") if fc else None, "tofh": fc.get("tofh") if fc else None,
        "z": fc.get("z") if fc else None, "st": fc.get("st") if fc else None,
        "vbat": fc.get("vbat") if fc else None,
        "ml": rb.get("ml") if rb else None, "kill": rb.get("kill") if rb else None,
        "fl": rb.get("fl_txt") if rb else None, "cl": rb.get("cl") if rb else None,
        "gap_max": gmax, "cuts": rb.get("cuts") if rb else None,
        "crc": rb.get("crc") if rb else None, "other": other,
    }
    return ("GO" if not no else "NO-GO"), no, warn, summary


def en0_ip():
    return subprocess.run(["ipconfig", "getifaddr", "en0"], capture_output=True,
                          text=True).stdout.strip()


def rejoin():
    print("[ready] auto-rejoin: down (radio cycle)", flush=True)
    subprocess.call([WIFI, "down"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(6)
    if not en0_ip().startswith("192.168.4."):
        up_only()


def up_only():
    print("[ready] auto-rejoin: up", flush=True)
    subprocess.call([WIFI, "up"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=14550)
    ap.add_argument("--relay", type=int, default=14560,
                    help="forward every datagram to 127.0.0.1:RELAY (0 = off)")
    ap.add_argument("--log-dir", default=os.path.join(os.getcwd(), "riskybird-logs"))
    ap.add_argument("--silence", type=float, default=3.0,
                    help="seconds with no datagram before diagnosing the link")
    ap.add_argument("--auto-rejoin", action="store_true",
                    help="on a stale lease, run the wifi script's down/up")
    ap.add_argument("--pcap", metavar="IFACE",
                    help="capture the datagrams on IFACE with tcpdump (sudo -n) instead of "
                    "binding the port -- for a host whose firewall rejects them")
    args = ap.parse_args()

    os.makedirs(args.log_dir, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    raw = os.path.join(args.log_dir, f"raw_{stamp}.log")
    status_path = os.path.join(args.log_dir, "ready-status.json")
    logf = open(raw, "a")
    print(f"[ready] raw log:   {raw}", flush=True)
    print(f"[ready] status:    {status_path}", flush=True)
    print(f"[ready] relay ->   127.0.0.1:{args.relay}" if args.relay else "[ready] relay off",
          flush=True)
    # SIGTERM -> normal exit, so atexit handlers (the capture's tcpdump) run.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    source = pcap_source(args.pcap, args.port) if args.pcap else udp_source(args.port)
    if args.pcap:
        print(f"[ready] capture:   tcpdump on {args.pcap}, udp dst port {args.port}", flush=True)
    threading.Thread(target=rx, args=(source, args.relay, logf), daemon=True).start()

    t_start = time.time()
    last_diag = 0.0
    last_verdict = None
    rejoined_at = 0.0
    rejoiner = None
    while True:
        time.sleep(1.0)
        now = time.time()
        with S.lock:
            last_any = S.last_any
        silent = (now - (last_any or t_start)) > args.silence
        link = "ok"
        if silent:
            if now - last_diag >= 5:
                last_diag = now
                link = "STALE LEASE" if ping_ok() else "DRONE OFF AIR"
                if (args.auto_rejoin and now - rejoined_at > 30
                        and not (rejoiner and rejoiner.is_alive())):
                    # In a thread: `up` takes 30-40 s, and running it inline froze
                    # the status line and ready-status.json for that long.
                    if link == "STALE LEASE":
                        rejoined_at = now
                        rejoiner = threading.Thread(target=rejoin, daemon=True)
                        rejoiner.start()
                    elif not en0_ip().startswith("192.168.4."):
                        # SoftAP not joined: keep trying `up` so the Mac joins
                        # as soon as the drone comes back on the air.
                        rejoined_at = now
                        rejoiner = threading.Thread(target=up_only, daemon=True)
                        rejoiner.start()
            else:
                link = "silent"
        verdict, no, warn, summ = evaluate(now)
        summ["link"] = link
        summ["verdict"] = verdict
        summ["reasons"] = no
        summ["warnings"] = warn
        try:
            with open(status_path + ".tmp", "w") as f:
                json.dump(summ, f)
            os.replace(status_path + ".tmp", status_path)
        except OSError:
            pass
        if silent:
            hint = {"STALE LEASE": " -> run: riskybird-wifi-macos.sh down; then up",
                    "DRONE OFF AIR": " -> drone unpowered / SoftAP down"}.get(link, "")
            line = "[%s] NO-GO  link %s%s" % (summ["t"][11:], link, hint)
        else:
            line = ("[%s] %-5s fc %4.1fHz rb %3.1fHz tilt %s/%s tofv %s tofh %s z %s st %s "
                    "| ml %s kill %s %s cl %s gap %s cuts %s | vbat %s"
                    % (summ["t"][11:], verdict, summ["fc_hz"], summ["rb_hz"], summ["tilt"],
                       summ["tilt_acc"], _i(summ["tofv"]), summ["tofh"], summ["z"],
                       _i(summ["st"]), summ["ml"], _i(summ["kill"]), summ["fl"],
                       _i(summ["cl"]), _i(summ["gap_max"]), _i(summ["cuts"]), summ["vbat"]))
            if no:
                line += "\n        NO-GO: " + "; ".join(no)
            if warn:
                line += "\n        warn:  " + "; ".join(warn)
        # Every second while not GO, and on each change; once every 10 s while GO.
        if verdict != "GO" or verdict != last_verdict or int(now) % 10 == 0:
            print(line, flush=True)
        last_verdict = verdict


def _i(v):
    return None if v is None else int(v)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
