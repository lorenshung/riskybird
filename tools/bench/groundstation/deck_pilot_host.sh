#!/bin/bash
# deck_pilot_host.sh - keep deck_pilot.py --live running ON THE DECK, from CobbledSlate,
# with auto-reconnect. The bridge reads the Deck gamepad via evdev and POSTs the panel's
# /pilot + /cmd over the reverse tunnel (Deck localhost:8080), so it needs neither the
# browser Gamepad API nor a Steam Input template.
#
# Why host-side: SteamOS systemd-logind kills Deck-side session processes when the SSH
# connection closes, so the pilot bridge is maintained FROM here (like deck_tunnel.sh).
# ssh -tt forces a PTY so the remote python dies cleanly on a dropped link (no orphans).
#
# SAFETY: deck_pilot.py is --live here, but it transmits NOTHING until you toggle engage
# (A) AND hold the deadman (RB); right-stick-up is throttle, B=ESTOP, X=DISARM, Y=RESET.
# Requires: the reverse tunnel up (deck_tunnel.sh / rb_fly.sh) and the panel serving.
DECK=${DECK:-deck@10.0.0.45}
ARGS=${PILOT_ARGS:---live --yaw-scale 0 --stick-gain 0.75}   # yaw stick OFF (FC holds takeoff heading);
                                                           # full left stick = 0.75 x 40deg = 30deg bank
echo "[pilot] launching deck_pilot.py $ARGS on $DECK (Ctrl-C here to stop)"
while true; do
  ssh -tt -o ServerAliveInterval=15 -o ServerAliveCountMax=3 \
      -o StrictHostKeyChecking=accept-new -o ConnectTimeout=8 \
      "$DECK" "cd ~/riskybird-gcs && pkill -u deck -f '^python3 -u deck_pilot.py'; exec python3 -u deck_pilot.py $ARGS"
  # ^ a dropped link can leave the previous bridge orphaned on the Deck (half-open PTY) -- two bridges
  #   would both read the pad and both toggle engage. Clear it first; the anchored pattern can't match
  #   the remote shell (cmdline starts "bash -c") or pkill itself.
  echo "[pilot] dropped rc=$? $(date +%T); relaunching in 3s"
  sleep 3
done
