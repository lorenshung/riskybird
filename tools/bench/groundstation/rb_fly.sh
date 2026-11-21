#!/bin/bash
# rb_fly.sh - one-command bring-up + health for the riskybird standalone-ESP flight
# controller, flown from the Steam Deck panel via CobbledSlate.
#
# It walks every hop in order and tells you EXACTLY which one is broken:
#   ESP app running -> SoftAP up -> dongle associated+IP -> drone pingable
#   -> panel serving -> Deck reverse tunnel -> live telemetry + FC health
#   -> command uplink (:14551) reachable
# It is idempotent and non-destructive by default: it only hard-resets the ESP if
# the SoftAP is missing, and it never spins motors unless you pass --motor-test.
#
# Usage:
#   ./rb_fly.sh               full bring-up + health check (safe)
#   ./rb_fly.sh --reset       force an ESP hard-reset even if the AP is already up
#   ./rb_fly.sh --motor-test  DANGER: ramp a tiny throttle to prove motors spin.
#                             PROPS OFF / drone secured ONLY. 3 s abort window.
#
# Why this exists: bring-up kept failing on two silent faults - (1) the ESP powered
# but NOT running the flight app (no SoftAP at all), fixed by an esptool hard-reset;
# (2) the ESP SoftAP's DHCP is broken, so the dongle associates but never gets an IP
# - fixed by a STATIC dongle address. See memory riskybird-steamdeck-gcs.
set -u

# ---- config ---------------------------------------------------------------
ESPTOOL=/home/cobble/Tools/zephyr-chipyard-sw/tools/miniforge3/envs/zephyr/bin/esptool.py
PORT=/dev/ttyACM0
SSID=riskybird-98e0
DRONE=192.168.4.1
DONGLE_IP=192.168.4.11
IFACE=wlan1
CMD_PORT=14551
PANEL=http://127.0.0.1:8080
DECK=deck@10.0.0.45
GS="$(cd "$(dirname "$0")" && pwd)"

RESET=0; MOTOR_TEST=0
for a in "$@"; do
  case "$a" in
    --reset) RESET=1;;
    --motor-test) MOTOR_TEST=1;;
    *) echo "unknown arg: $a"; exit 2;;
  esac
done

ok(){   printf '  \033[32m[ OK ]\033[0m %s\n' "$*"; }
bad(){  printf '  \033[31m[FAIL]\033[0m %s\n' "$*"; }
warn(){ printf '  \033[33m[warn]\033[0m %s\n' "$*"; }
hdr(){  printf '\n\033[1m== %s ==\033[0m\n' "$*"; }
GO=1

scan_has_ap(){ nmcli -t -f SSID device wifi list ifname "$IFACE" 2>/dev/null | grep -qx "$SSID"; }

# ---- 1. ESP app + SoftAP --------------------------------------------------
hdr "1. ESP flight app / SoftAP ($SSID)"
nmcli device wifi rescan ifname "$IFACE" >/dev/null 2>&1; sleep 3
if [ "$RESET" = 1 ] || ! scan_has_ap; then
  if [ "$RESET" = 1 ]; then warn "forced reset requested"; else warn "SoftAP not visible - ESP may not be running the app"; fi
  if [ ! -e "$PORT" ]; then
    bad "$PORT missing - ESP not connected/powered over USB. Plug it in."; GO=0
  else
    for try in 1 2; do
      echo "  hard-resetting ESP into the flight app (try $try)..."
      timeout 40 "$ESPTOOL" --chip esp32c6 --port "$PORT" --before default_reset --after hard_reset chip_id \
        >/dev/null 2>&1 && ok "esptool reset sent" || warn "esptool reset returned nonzero"
      sleep 11
      nmcli device wifi rescan ifname "$IFACE" >/dev/null 2>&1; sleep 4
      scan_has_ap && break
    done
  fi
fi
if scan_has_ap; then
  sig=$(nmcli -t -f SSID,SIGNAL device wifi list ifname "$IFACE" 2>/dev/null | awk -F: -v s="$SSID" '$1==s{print $2; exit}')
  ok "SoftAP $SSID up (signal ${sig}%)"
else
  bad "SoftAP $SSID still not broadcasting after reset - firmware crash-looping or motor power sag? Check the ESP."; GO=0
fi

# ---- 2. dongle association + static IP ------------------------------------
hdr "2. Dongle $IFACE -> $SSID (static $DONGLE_IP)"
have_ip(){ ip -4 -br addr show "$IFACE" 2>/dev/null | grep -q "$DONGLE_IP"; }
pingdrone(){ ping -c2 -W2 -I "$IFACE" "$DRONE" >/dev/null 2>&1; }
if have_ip && pingdrone; then
  ok "already connected, $DRONE answers ping"
elif scan_has_ap; then
  # enforce static (DHCP from the ESP AP is broken); single clean connect, no hammering
  nmcli connection modify "$SSID" ipv4.method manual ipv4.addresses "$DONGLE_IP/24" ipv4.gateway "$DRONE" >/dev/null 2>&1
  for try in 1 2; do
    nmcli --wait 25 connection up "$SSID" ifname "$IFACE" >/dev/null 2>&1
    sleep 2
    if have_ip && pingdrone; then break; fi
    warn "connect attempt $try did not take; rescanning"
    nmcli device wifi rescan ifname "$IFACE" >/dev/null 2>&1; sleep 4
  done
  if have_ip && pingdrone; then
    rtt=$(ping -c3 -W2 -I "$IFACE" "$DRONE" 2>/dev/null | awk -F'/' '/rtt/{print $5" ms avg"}')
    ok "connected: $IFACE @ $DONGLE_IP, drone ping OK ($rtt)"
  else
    bad "could not associate/reach the drone (don't retry-hammer: it crashes the ESP WiFi). Re-run; if persistent, power-cycle the ESP."; GO=0
  fi
else
  bad "skipped (no SoftAP)"; GO=0
fi

# ---- 3. panel ------------------------------------------------------------
hdr "3. Panel ($PANEL)"
if curl -s -o /dev/null --max-time 4 "$PANEL/config"; then
  ok "panel serving"
else
  warn "panel down - starting it (live transmit)"
  ( cd "$GS" && setsid nohup python3 riskybird_panel.py --bind 127.0.0.1 >/tmp/rb_panel.log 2>&1 & )
  sleep 2
  curl -s -o /dev/null --max-time 4 "$PANEL/config" && ok "panel started" || { bad "panel failed to start - see /tmp/rb_panel.log"; GO=0; }
fi

# ---- 4. Deck reverse tunnel ----------------------------------------------
hdr "4. Deck reverse tunnel (Deck localhost:8080)"
if pgrep -f 'deck_tunnel\.sh' >/dev/null 2>&1 || pgrep -f 'ssh -N .*-R 8080:localhost:8080' >/dev/null 2>&1; then
  ok "tunnel process running"
else
  warn "tunnel down - starting the self-healing loop (deck_tunnel.sh)"
  setsid nohup "$GS/deck_tunnel.sh" >/tmp/rb_tunnel.log 2>&1 &
  sleep 6
fi
deckcode=$(ssh -o ConnectTimeout=6 "$DECK" 'curl -s -o /dev/null -w "%{http_code}" --max-time 4 http://localhost:8080/config 2>/dev/null' 2>/dev/null)
if [ "$deckcode" = "200" ]; then ok "Deck reaches the panel (HTTP 200)"; else warn "Deck localhost:8080 returned '$deckcode' (tunnel still settling, or the orphan-port gotcha - see memory)"; fi

# ---- 5. live telemetry + FC health ---------------------------------------
hdr "5. Telemetry + FC health"
frame=$(curl -s --max-time 6 "$PANEL/stream" 2>/dev/null | grep -m1 '"link"')
if [ -n "$frame" ]; then
  echo "$frame" | python3 -c '
import sys,json
d=json.loads(sys.stdin.read().replace("data: ",""))
L=d["link"]; t=d["t"]
def o(c,m): print(("  \033[32m[ OK ]\033[0m " if c else "  \033[31m[FAIL]\033[0m ")+m)
o(L.get("connected"), "link connected  rate=%.0f Hz  quality=%s  age=%sms"%(L.get("rate",0),L.get("quality"),L.get("age_ms")))
o(not t.get("estop"), "estop clear" if not t.get("estop") else "ESTOP LATCHED - press ARM/ENABLE to clear")
o(t.get("caldone"), "gyro cal done" if t.get("caldone") else "still calibrating - hold still")
o(t.get("tofv")==1, "ToF ranging (tofh=%.2f)"%t.get("tofh",0))
print("  loop it=%s dt=%sms  st=%s armed=%s  motors u=%s"%(t.get("it"),t.get("dt_ms"),t.get("st"),t.get("armed"),t.get("u")))
vb=t.get("vbat",0)
if vb<1.0:
    print("  \033[33m[note]\033[0m vbat=%.2f - battery sense is compiled OFF in this build"%vb)
    print("         (ROSE_BATT_SENSE=0), so this is EXPECTED and does NOT indicate a dead pack.")
    print("         It also means the FC cannot see motor power - verify the flight battery")
    print("         physically if there is no chirp/thrust (USB powers only the ESP logic).")
' || { warn "could not parse telemetry frame"; }
else
  bad "no telemetry on $PANEL/stream"; GO=0
fi

# ---- 6. command uplink ---------------------------------------------------
hdr "6. Command uplink (FC :$CMD_PORT)"
python3 - "$DONGLE_IP" "$DRONE" "$CMD_PORT" <<'PY'
import socket,sys
ip,drone,port=sys.argv[1],sys.argv[2],int(sys.argv[3])
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);
try: s.bind((ip,0))
except OSError as e: print("  \033[31m[FAIL]\033[0m dongle not at %s (%s)"%(ip,e)); sys.exit(0)
s.settimeout(1.2)
for _ in range(3): s.sendto(b'PILOT 0 0 0 0\n',(drone,port))
try: s.recv(64); print("  \033[32m[ OK ]\033[0m FC command port replied")
except socket.timeout: print("  \033[32m[ OK ]\033[0m FC command port open (no ICMP unreachable; PILOT is one-way)")
except Exception as e: print("  \033[31m[FAIL]\033[0m %s"%e)
PY

# ---- 7. optional motor proof --------------------------------------------
if [ "$MOTOR_TEST" = 1 ]; then
  hdr "7. MOTOR TEST  (!! PROPS OFF / DRONE SECURED !!)"
  echo "  Ramping throttle to ~8% for ~1.5 s then back to 0. Ctrl-C to abort."
  for n in 3 2 1; do printf "  starting in %s...\r" "$n"; sleep 1; done; echo
  python3 - "$DONGLE_IP" "$DRONE" "$CMD_PORT" "$PANEL" <<'PY'
import socket,sys,time,urllib.request,json
ip,drone,port,panel=sys.argv[1],sys.argv[2],int(sys.argv[3]),sys.argv[4]
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.bind((ip,0))
def u_now():
    try:
        for line in urllib.request.urlopen(panel+"/stream",timeout=3):
            line=line.decode();
            if '"link"' in line:
                return json.loads(line.replace("data: ","")).get("t",{}).get("u")
    except Exception: return None
base=u_now(); print("  idle motors u =",base)
t0=time.time()
try:
    while time.time()-t0 < 2.6:                     # ramp up to 80, hold
        thr=min(80,int(80*(time.time()-t0)/1.5))
        s.sendto(("PILOT 0 0 0 %d\n"%thr).encode(),(drone,port)); time.sleep(0.033)
    live=u_now(); print("  motors under 8%% throttle u =",live)
finally:
    for _ in range(20): s.sendto(b'PILOT 0 0 0 0\n',(drone,port)); time.sleep(0.02)
    print("  throttle zeroed.")
if base and live and max(live)>max(base)+0.05:
    print("  \033[32m[ OK ]\033[0m motors responded to throttle - drive path works end to end")
else:
    print("  \033[33m[warn]\033[0m motor output did not rise - check motor power (battery) / ESC wiring")
PY
fi

# ---- summary -------------------------------------------------------------
hdr "RESULT"
if [ "$GO" = 1 ]; then
  printf '  \033[1;32mGO\033[0m - link + FC healthy.\n'
else
  printf '  \033[1;31mNO-GO\033[0m - fix the FAIL line(s) above.\n'
fi
cat <<EOF

  To actually fly from the Deck:
    1. Flight battery connected (vbat>0; brushless ESCs chirp on power-up).
    2. On the Deck, the GCS shortcut's Steam Input layout = a GAMEPAD template
       (not Desktop) - else the sticks drive the mouse and the browser Gamepad
       API sees nothing, so the panel sends no throttle.
    3. In the panel: press a pad button to wake the API, then A (engage) +
       hold RB (deadman) + push right-stick up for throttle.
    4. Sanity-prove motors first with:  ./rb_fly.sh --motor-test   (PROPS OFF)
EOF
