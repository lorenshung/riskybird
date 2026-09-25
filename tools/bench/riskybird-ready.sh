#!/usr/bin/env bash
#
# One command to get this Mac flight-ready, and keep it that way between flights:
#
#   tools/bench/riskybird-ready.sh                  # join once; no auto-rejoin (default)
#   tools/bench/riskybird-ready.sh --no-join        # start everything, do not touch Wi-Fi
#   tools/bench/riskybird-ready.sh --auto-rejoin    # also recover stale leases / rejoin by itself
#
# What it does:
#   1. caffeinate for as long as this script runs;
#   2. `riskybird-wifi-macos.sh up` once if en0 is not already on the drone network
#      (this needs internet on a non-Wi-Fi service, iPhone USB, and `up` still refuses without it);
#      skipped with --no-join;
#   3. the v3 panel on http://127.0.0.1:8080, fed from a LOCAL relay port (14560);
#   4. riskybird-flightready.py in the foreground. It owns :14550, logs every raw line
#      (FC + RBLINK), relays to the panel, and prints a live GO / NO-GO. With
#      --auto-rejoin it also recovers a stale lease after an ESP reboot, and joins the
#      SoftAP by itself when the drone comes back on the air. Off by default: each
#      retry switches the Mac's Wi-Fi, and after an ESP reset you run `up` once instead.
#
# Nothing here sends to the drone's :14551 except `up`'s built-in PING. Commands
# (ESTOP / RESET / ...) are only ever sent from the panel's buttons.
# Ctrl-C stops the monitor, the panel and caffeinate together.

set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
WIFI="$HERE/riskybird-wifi-macos.sh"
PANEL="$ROOT/backends/zephyr-chipyard-sw/samples/rose_flight_controller/tools/groundstation/riskybird_panel.py"
LOGDIR="$ROOT/results/riskybird-logs"
RELAY=14560
AUTO=""
JOIN=1
for a in "$@"; do
	case "$a" in
		--auto-rejoin)    AUTO="--auto-rejoin" ;;
		--no-auto-rejoin) AUTO="" ;;
		--no-join)        JOIN=0 ;;
		*) echo "unknown argument: $a" >&2; exit 1 ;;
	esac
done

[ -f "$PANEL" ] || { echo "panel not found: $PANEL" >&2; exit 1; }
mkdir -p "$LOGDIR"

# Replace any earlier copies. Two receivers on :14550 would split the stream.
pkill -f riskybird_panel.py 2>/dev/null
pkill -f riskybird-flightready.py 2>/dev/null
pkill -f riskybird-dash.py 2>/dev/null
sleep 0.5

caffeinate -dimsu -w $$ &

case "$(ipconfig getifaddr en0 2>/dev/null)" in
	192.168.4.*) echo "[ready] already on the drone network" ;;
	*)
		if [ "$JOIN" -eq 1 ]; then
			"$WIFI" up || echo "[ready] 'up' did not complete; run '$WIFI up' when the drone is on the air"
		else
			echo "[ready] --no-join: not joining; run '$WIFI up' when ready"
		fi ;;
esac

python3 -u "$PANEL" --drone 192.168.4.1 --telem-port "$RELAY" --cmd-port 14551 \
	--http 8080 --log-dir "$LOGDIR" > "$LOGDIR/panel-stdout.txt" 2>&1 &
PANEL_PID=$!
trap 'kill $PANEL_PID 2>/dev/null' EXIT INT TERM
sleep 1
open http://127.0.0.1:8080 2>/dev/null

python3 -u "$HERE/riskybird-flightready.py" --relay "$RELAY" --log-dir "$LOGDIR" $AUTO
