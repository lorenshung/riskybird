#!/bin/bash
# gcs_host.sh -- run on CobbledSlate (the machine with the WiFi dongle -> drone SoftAP).
# Hosts the riskybird ground station for the Steam Deck:
#   1. serves the panel locally
#   2. maintains an auto-reconnecting SSH reverse tunnel so the panel shows up on the Deck's
#      localhost:8080 (secure context -> browser Gamepad API works; rides SSH :22, no firewall change).
# The Deck's "Riskybird GCS" Game-Mode shortcut just opens a kiosk browser at localhost:8080.
#
# Usage:
#   ./gcs_host.sh          # DRY-RUN (default, safe): panel does NOT transmit to the drone
#   ./gcs_host.sh --fly    # LIVE: panel transmits PILOT commands to the drone (dongle must be on the SoftAP)
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
DECK=deck@10.0.0.45
MODE="--pilot-dry-run"; LABEL="DRY-RUN (no transmit)"
[ "${1:-}" = "--fly" ] && { MODE=""; LABEL="LIVE (transmitting to drone)"; }

echo "[gcs_host] mode: $LABEL"
# 1) panel (reachable on localhost:8080; the reverse tunnel exposes it to the Deck)
if pgrep -f "[r]iskybird_panel.py" >/dev/null; then
  echo "[gcs_host] panel already running (leaving it; restart it yourself to change mode)"
else
  ( cd "$HERE" && nohup python3 riskybird_panel.py $MODE --bind 127.0.0.1 >/tmp/rb_panel.log 2>&1 & )
  echo "[gcs_host] panel started -> /tmp/rb_panel.log"
fi

# 2) auto-reconnecting reverse tunnel: CobbledSlate panel :8080 -> Deck localhost:8080
echo "[gcs_host] maintaining reverse tunnel to $DECK  (Ctrl-C to stop)"
trap 'echo; echo "[gcs_host] stopping tunnel"; exit 0' INT TERM
while true; do
  ssh -N -o BatchMode=yes -o ExitOnForwardFailure=yes \
      -o ServerAliveInterval=20 -o ServerAliveCountMax=3 \
      -R 8080:localhost:8080 "$DECK" || true
  echo "[gcs_host] tunnel dropped; reconnecting in 3s..."
  sleep 3
done
