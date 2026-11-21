#!/usr/bin/env python3
"""deck_pilot.py - read the Steam Deck gamepad via evdev and drive the riskybird
panel's /pilot + /cmd endpoints directly, bypassing the browser Gamepad API.

Why this exists: in Firefox kiosk on the Deck the browser Gamepad API often never
sees the controller (flatpak sandbox / needs a user-gesture), so the panel sends no
throttle and the drone never spins. Steam Input's virtual Xbox pad IS live at the
evdev layer (/dev/input/event5, vendor 0x28de), so we read it here and POST the same
commands the browser would - reusing ALL the panel-side clamping, wire format and
safety. Mapping is copied 1:1 from panel.html pollPad() so behaviour is identical.

Runs ON THE DECK and POSTs to the local tunnelled panel (http://localhost:8080).

SAFE BY DEFAULT: --dry-run (the default) computes + prints but sends NOTHING.
Pass --live to actually transmit. Even in --live, nothing is sent unless you
toggle engage (A) AND hold the deadman (RB) - identical to the panel.

  python3 deck_pilot.py                 # dry-run: watch the mapping, no commands
  python3 deck_pilot.py --live          # transmit (A=engage, hold RB, right-stick up=throttle)
  python3 deck_pilot.py --live --no-deadman   # don't require RB (NOT recommended)
"""
import argparse, json, select, sys, time, urllib.request
import evdev
from evdev import ecodes as E

# ---- mapping constants, copied from panel.html PCFG / pollPad ----
DEADZONE = 0.10
EXPO     = 0.35
SIGN_VX, SIGN_VY, SIGN_YAW = -1, -1, -1   # yaw: FLU +yaw = CCW, so stick-right (CW) is -1
# evdev axis codes for the logical sticks (xpad layout):
AX_LX, AX_LY = E.ABS_X,  E.ABS_Y     # left stick  (codes 0,1)
AX_RX, AX_RY = E.ABS_RX, E.ABS_RY    # right stick (codes 3,4; code 2=ABS_Z=LT)
# evdev button codes <- panel button indices (standard gamepad order):
BTN_ENGAGE = E.BTN_A       # idx0 A  -> toggle engage
BTN_ESTOP  = E.BTN_B       # idx1 B  -> ESTOP
BTN_DISARM = E.BTN_X       # idx2 X  -> DISARM
BTN_ARM    = E.BTN_Y       # idx3 Y  -> RESET (clear estop)
BTN_DEADMAN= E.BTN_TR      # idx5 RB -> deadman (hold)
BTN_TRACE  = E.BTN_START   # idx9 Start -> toggle trace

def dz(v, d=DEADZONE):
    if abs(v) < d: return 0.0
    s = 1.0 if v > 0 else -1.0
    m = (abs(v) - d) / (1 - d)
    m = (1 - EXPO) * m + EXPO * m * m * m
    return s * m

def find_pad():
    for p in evdev.list_devices():
        d = evdev.InputDevice(p)
        if "x-box" in d.name.lower() or "xbox" in d.name.lower() or "360" in d.name:
            return d
    return None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://localhost:8080")
    ap.add_argument("--device", default=None, help="evdev path (default: auto-find Xbox pad)")
    ap.add_argument("--live", action="store_true", help="actually transmit (default is dry-run)")
    ap.add_argument("--no-deadman", action="store_true", help="do not require RB held")
    ap.add_argument("--hz", type=float, default=30.0)
    # Yaw shares the right stick with throttle, and the FC INTEGRATES yaw rate into a heading target,
    # so a slightly off-vertical throttle push keeps winding the heading. Scale it down (0 = yaw off;
    # the FC still HOLDS the takeoff heading) and give it its own, wider deadzone.
    ap.add_argument("--yaw-scale", type=float, default=1.0, help="yaw stick gain (0 = disabled)")
    # Left-stick gain. In the ANGLE-mode FC build full stick (gain 1.0) = PILOT_ANGLE_MAX_RAD (40 deg),
    # so 0.75 = 30 deg; the FC clamps |vx|,|vy| at 0.6 m/s, so gains >1 just saturate earlier.
    ap.add_argument("--stick-gain", type=float, default=1.0, help="left-stick gain (angle mode: x 40 deg)")
    ap.add_argument("--yaw-deadzone", type=float, default=0.35, help="yaw-only stick deadzone [0..1)")
    args = ap.parse_args()
    live = args.live
    require_deadman = not args.no_deadman

    dev = evdev.InputDevice(args.device) if args.device else find_pad()
    if not dev:
        print("no Xbox pad found in", evdev.list_devices()); sys.exit(1)
    print("pad:", dev.path, dev.name, "| mode:", "LIVE" if live else "DRY-RUN",
          "| deadman:", "required (hold RB)" if require_deadman else "OFF")

    # per-axis normalization from absinfo
    norm = {}
    caps = dict(dev.capabilities().get(E.EV_ABS, []))
    for code in (AX_LX, AX_LY, AX_RX, AX_RY):
        ai = caps.get(code)
        if ai:
            center = (ai.max + ai.min) / 2.0
            half = (ai.max - ai.min) / 2.0 or 1.0
            norm[code] = (center, half)
        else:
            norm[code] = (0.0, 32767.0)

    # config from the panel
    vmax, yawmax = 0.5, 0.6
    try:
        c = json.load(urllib.request.urlopen(args.url + "/config", timeout=3))
        vmax = float(c.get("pilot_vmax", vmax)); yawmax = float(c.get("pilot_yawmax", yawmax))
        if not c.get("pilot_dry_run", True) and live:
            print("panel is LIVE-transmit; pilot commands WILL reach the drone.")
    except Exception as e:
        print("warn: /config unreachable (%s); using defaults" % e)

    axis = {AX_LX: 0, AX_LY: 0, AX_RX: 0, AX_RY: 0}
    btn = {}
    prev = {}
    engaged = False; trace = False; was_active = False; thr_kill = False
    last_tx = 0.0; last_status = 0.0; txn = 0; dt_send = 1.0 / args.hz

    def naxis(code):
        center, half = norm[code]
        return max(-1.0, min(1.0, (axis[code] - center) / half))

    def post_cmd(text):
        if not live:
            print("  [dry] CMD", text); return
        try:
            urllib.request.urlopen(urllib.request.Request(args.url + "/cmd", data=text.encode()), timeout=1.5).read()
            print("  CMD ->", text)
        except Exception as e:
            print("  CMD fail", text, e)

    def post_pilot(vx, vy, yawrate, thr, eng):
        nonlocal txn
        if not live: return
        body = json.dumps({"vx": vx, "vy": vy, "yawrate": yawrate, "alt": thr,
                           "engaged": eng, "trace": trace,
                           "armed": True}).encode()
        try:
            urllib.request.urlopen(urllib.request.Request(args.url + "/pilot", data=body,
                                   headers={"Content-Type": "application/json"}), timeout=1.0).read()
            txn += 1
        except Exception:
            pass

    print("reading pad... (Ctrl-C to stop)")
    while True:
        r, _, _ = select.select([dev.fd], [], [], 0.005)
        if r:
            try:
                for e in dev.read():
                    if e.type == E.EV_ABS and e.code in axis: axis[e.code] = e.value
                    elif e.type == E.EV_KEY: btn[e.code] = e.value
            except OSError:
                time.sleep(0.5); continue
        now = time.time()
        if now - last_tx < dt_send: continue
        last_tx = now

        def pressed(code): return bool(btn.get(code, 0))
        def edge(code):
            n = pressed(code); was = prev.get(code, False); prev[code] = n
            return n and not was
        if edge(BTN_ESTOP):  post_cmd("ESTOP")
        if edge(BTN_DISARM): post_cmd("DISARM")
        if edge(BTN_ARM):    post_cmd("RESET")
        if edge(BTN_ENGAGE): engaged = not engaged; print("  engage ->", engaged)
        if edge(BTN_TRACE):  trace = not trace; print("  trace ->", trace)

        deadman = (not require_deadman) or pressed(BTN_DEADMAN)
        lx, ly = dz(naxis(AX_LX)), dz(naxis(AX_LY))
        rx, ry = dz(naxis(AX_RX), args.yaw_deadzone), dz(naxis(AX_RY))
        vx = SIGN_VX * ly * vmax * args.stick_gain
        vy = SIGN_VY * lx * vmax * args.stick_gain
        yawrate = SIGN_YAW * rx * yawmax * args.yaw_scale
        thr = max(0.0, -ry)
        # THROTTLE-DOWN KILL: right stick pushed DOWN past 50% = ESTOP (same as B), once per push; raw
        # (no deadzone) axis so 50% is the physical half-way; re-arms once back above 30%.
        ry_raw = naxis(AX_RY)
        if ry_raw > 0.5 and not thr_kill:
            thr_kill = True; post_cmd("ESTOP"); print("  throttle-down KILL -> ESTOP")
        elif ry_raw < 0.3:
            thr_kill = False
        active = engaged and deadman

        if active:
            post_pilot(round(vx, 3), round(vy, 3), round(yawrate, 3), round(thr, 3), True)
            was_active = True
        elif was_active:
            post_pilot(0, 0, 0, 0, False); was_active = False

        if now - last_status >= 0.25:
            last_status = now
            st = "ENGAGED" if engaged else "off"
            dm = "RB" if deadman else "--"
            act = "SEND" if active else "idle"
            print("\r eng:%-7s dead:%s  vx%+.2f vy%+.2f yaw%+.2f thr%3d%%  [%s%s] tx%d   "
                  % (st, dm, vx, vy, yawrate, int(thr*100), act,
                     "" if live else "/dry", txn), end="", flush=True)

if __name__ == "__main__":
    try: main()
    except KeyboardInterrupt: print("\nstopped.")
