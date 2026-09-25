#!/usr/bin/env bash
#
# The standard bench session. One command, from anywhere, including over ssh:
#
#     tools/bench/rb-watch.sh
#
# Creates (or re-attaches to) a persistent tmux session laid out so that a
# person who is NOT in the room can see what the drone is doing. Detaching with
# ctrl-b d leaves everything running and still logging; closing the ssh
# connection does the same. Re-run it to get back in.
#
#   +---------------------------+------------------------+
#   |                           |  STATUS  (refreshing)  |
#   |                           +------------------------+
#   |   work pane               |  FPGA console          |
#   |   (shell / claude)        +------------------------+
#   |                           |  ESP link / gate probe |
#   +---------------------------+------------------------+
#
# The two console panes write into $RB_LOGDIR so the status pane -- and anyone
# reading later -- can see them without fighting over the serial port. Only one
# reader per tty works, so ALWAYS read the log file rather than opening the
# device a second time. That is the single most common way to get a console
# that silently drops half its bytes.
#
# LOG LAYOUT. Everything a console pane captures goes under a per-session
# directory, and the two paths every other tool already knows are symlinks into
# it:
#
#   $RB_LOGDIR/fpga-console.log          -> the CURRENT part, always
#   $RB_LOGDIR/esp-live.log              -> the CURRENT part, always
#   $RB_LOGDIR/sessions/current          -> the CURRENT session directory
#   $RB_LOGDIR/sessions/<stamp>/         one directory per rb-watch session
#       session.txt                      what was plugged in, and when
#       fpga-console.<stamp>.NNN.log     parts, capped at RB_LOG_MAX_MB
#       esp-live.<stamp>.NNN.log
#
# That is what keeps $RB_LOGDIR readable after a week of running: yesterday is a
# directory you can ignore rather than the first 400 MB of the file you are
# trying to tail. Each attach also writes a `==== ... ====` marker line into the
# log, so `grep '^===='` over a session is its table of contents.
#
# Sessions older than RB_LOG_KEEP_DAYS (default 14) are removed at startup.
set -u

SESSION="${RB_SESSION:-riskybird}"
# The checkout this script lives in, two levels up from tools/bench.
REPO="${RB_REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
LOGDIR="${RB_LOGDIR:-$HOME/riskybird-logs}"
KEEP_DAYS="${RB_LOG_KEEP_DAYS:-14}"
MAX_MB="${RB_LOG_MAX_MB:-32}"
mkdir -p "$LOGDIR/sessions"

STATUS="$REPO/tools/bench/rb-status.sh"

# `rb bench` shifts an IDCODE, which means it OPENS the FTDI adapter. That is
# fine once, at session creation, but it would collide with a live OpenOCD, so
# skip it when a debug session is already attached and let the console pane wait
# for a device rather than stealing the adapter to go looking for one.
openocd_running() { pgrep -x openocd >/dev/null 2>&1; }

# Resolve the consoles fresh every time: ttyUSB numbers and USB locations
# renumber on every replug, so anything hardcoded is wrong by tomorrow.
#
# `< /dev/null` is load-bearing. `timeout` runs its child in a new process
# group; OpenOCD, underneath `rb bench`, reads whatever stdin it inherits; and a
# background process group that reads the terminal gets SIGTTIN and STOPS. That
# is not hypothetical -- it hung the status pane on its first render, in state T,
# forever, on the word ADAPTERS. Take the terminal away and it cannot happen.
bench_probe() {
    (cd "$REPO" && timeout 25 ./rb bench 2>/dev/null </dev/null)
}

resolve_fpga_console() {
    # The role column reads "debug" when the FPGA is configured and "debug?"
    # when the debug TAP is shifting all ones, which is what an UNCONFIGURED
    # part does -- exactly the state you are in before you program it. Matching
    # only "debug" pointed the console pane at a device that does not exist for
    # the whole window in which you most want the boot banner. `rb` itself
    # matches on the prefix; so do we.
    printf '%s\n' "$BENCH_OUT" | awk '$1 ~ /^debug/ {print $5; exit}'
}
resolve_esp() {
    local d; d=$(ls /dev/serial/by-id/*Espressif* 2>/dev/null | head -1)
    [ -n "$d" ] && readlink -f "$d"
}

# A console pane that survives the device being absent or unplugged: it retries
# rather than dying, so a drone plugged in later needs no action from the user.
#
# The capture goes through awk rather than `tee` so one stream can be size-capped
# without anybody having to remember to truncate it. awk writes each line to the
# pane and to the current part; when a part reaches the cap it opens the next one
# and re-points the stable symlink. Nothing that reads $RB_LOGDIR/<name>.log has
# to know any of that happened.
#
# The trade is that capture is line-buffered rather than byte-transparent: a
# console that emits a prompt with no trailing newline shows it late. Every
# workload on this bench prints whole lines through printk, so that is a fair
# trade for a log that cannot eat the disk.
#
# Parts are numbered per SESSION, not per attach, and a re-attach APPENDS to the
# part it left off in. Naming them per attach instead cost a whole status screen
# in testing: a console flap -- a USB blip, a board reset -- moved the stable
# symlink to a brand new empty file, and the status pane instantly forgot the
# battery reading, the commit gate and every recent event, none of which had
# stopped being true. A remote operator must not lose the picture because a
# cable twitched.
console_cmd() {
    local dev="$1" name="$2" link="$3" label="$4" sessdir="$5"
    cat <<EOF
clear
while true; do
  if [ -e "$dev" ]; then
    echo "=== $label : $dev @115200 ==="
    stty -F "$dev" 115200 raw -echo 2>/dev/null
    rbpart=\$(ls '$sessdir'/$name.[0-9][0-9][0-9].log 2>/dev/null \
              | sed -n 's/.*\.\([0-9]\{3\}\)\.log\$/\1/p' | sort -n | tail -1)
    rbpart=\$(( 10#\${rbpart:-0} )); [ "\$rbpart" -eq 0 ] && rbpart=1
    rbbytes=\$(stat -c %s "\$(printf '%s/%s.%03d.log' '$sessdir' '$name' "\$rbpart")" 2>/dev/null || echo 0)
    { echo "==== \$(date '+%Y-%m-%d %H:%M:%S')  $label ATTACH  dev=$dev ===="; \
      cat "$dev" 2>/dev/null; } \
    | awk -v dir='$sessdir' -v name='$name' -v link='$link' \
          -v part="\$rbpart" -v bytes="\$rbbytes" -v max=\$(( $MAX_MB * 1024 * 1024 )) '
        function open_part() {
            file = sprintf("%s/%s.%03d.log", dir, name, part)
            system("ln -sfn \"" file "\" \"" link "\"")
        }
        BEGIN { open_part() }
        {
            print
            print >> file
            fflush(file)
            bytes += length(\$0) + 1
            if (bytes >= max) { close(file); part++; bytes = 0; open_part() }
        }
      '
    echo "--- $label detached, retrying in 5s ---"
  else
    echo "=== $label : $dev not present, waiting ==="
  fi
  sleep 5
done
EOF
}

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "session '$SESSION' already exists (nothing restarted, nothing re-probed)"
    [ -t 0 ] || { echo "no terminal to attach to; it is still running and capturing."; exit 0; }
    exec tmux attach -t "$SESSION"
fi

# One directory per session. The stamp is this session's identity everywhere.
STAMP="$(date +%Y-%m-%dT%H-%M-%S)"
SESSDIR="$LOGDIR/sessions/$STAMP"
mkdir -p "$SESSDIR"
ln -sfn "$SESSDIR" "$LOGDIR/sessions/current"

# A log written by an older version of this script is a plain file where a
# symlink now goes. Keep it -- it is somebody's evidence -- but move it aside
# rather than letting `ln -sfn` clobber it.
for legacy in fpga-console.log esp-live.log; do
    if [ -f "$LOGDIR/$legacy" ] && [ ! -L "$LOGDIR/$legacy" ]; then
        mkdir -p "$LOGDIR/archive"
        mv "$LOGDIR/$legacy" "$LOGDIR/archive/$legacy.$STAMP"
        echo "moved pre-session $legacy to archive/$legacy.$STAMP"
    fi
done

# Prune whole sessions, never individual parts: a half-deleted session reads as
# a gap in the evidence, which is worse than a big directory.
if [ "$KEEP_DAYS" -gt 0 ] 2>/dev/null; then
    find "$LOGDIR/sessions" -mindepth 1 -maxdepth 1 -type d -mtime "+$KEEP_DAYS" \
        -exec rm -rf {} + 2>/dev/null
fi

# Probe ONCE, here, and hand the result to everything downstream -- including
# rb-status.sh, which reads this cache and never probes on its own. One open of
# the adapter per session is the budget; the status screen refreshing every five
# seconds is not allowed to spend any of it.
if openocd_running; then
    BENCH_OUT=""
    BENCH_NOTE="(skipped: OpenOCD is already running, and probing would fight it for the adapter)"
else
    BENCH_OUT="$(bench_probe)"
    BENCH_NOTE=""
    [ -n "$BENCH_OUT" ] && printf '%s\n' "$BENCH_OUT" > "$LOGDIR/.rb-bench-cache"
fi

# RB_FPGA_DEV / RB_ESP_DEV override the probe. Two real uses: a bench where
# `rb bench` cannot resolve a role (an unconfigured part, or OpenOCD already
# holding the adapter), and testing this script with no hardware attached.
FPGA_DEV="${RB_FPGA_DEV:-$(resolve_fpga_console)}"; FPGA_DEV="${FPGA_DEV:-/dev/ttyUSB_absent}"
ESP_DEV="${RB_ESP_DEV:-$(resolve_esp)}";            ESP_DEV="${ESP_DEV:-/dev/ttyACM_absent}"
FPGA_LOG="$LOGDIR/fpga-console.log"
ESP_LOG="$LOGDIR/esp-live.log"

{
    echo "session   $STAMP"
    echo "host      $(hostname -s)"
    echo "repo      $REPO"
    echo "rev       $(cd "$REPO" && git rev-parse --short HEAD 2>/dev/null || echo unknown)"
    echo "branch    $(cd "$REPO" && git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
    echo "fpga dev  $FPGA_DEV"
    echo "esp dev   $ESP_DEV"
    echo "log cap   ${MAX_MB} MB per part, sessions kept ${KEEP_DAYS} days"
    echo
    echo "--- rb bench at session start ---"
    printf '%s\n' "${BENCH_OUT:-$BENCH_NOTE}"
} > "$SESSDIR/session.txt"

echo "creating session '$SESSION'"
echo "  logs         : $SESSDIR"
echo "  FPGA console : $FPGA_DEV -> $FPGA_LOG"
echo "  ESP          : $ESP_DEV -> $ESP_LOG"

# -x/-y are load-bearing when this is started without a controlling terminal
# (a script, a CI step, `ssh host tools/bench/rb-watch.sh </dev/null`): tmux has
# no client to take a size from, reports "size missing" and every split-window
# then fails with "can't find pane". The window resizes to whatever client
# attaches later, so these are only a starting size.
tmux new-session  -d -s "$SESSION" -c "$REPO" -n bench -x 200 -y 50

# tmux 3.4 REMOVED `split-window -p <pct>`. It does not warn: it fails with
# "size missing", the split never happens, and every send-keys that follows
# fails with "can't find pane" -- so the session comes up as one bare shell with
# no status pane and no consoles, quietly capturing nothing. Measured on tmux
# 3.4 here. `-l <pct>%` is the replacement; `-p` is the fallback for tmux < 3.1,
# which does not accept a percentage in -l.
split() {   # direction  target  percent
    tmux split-window "$1" -t "$2" -c "$REPO" -l "$3%" 2>/dev/null \
        || tmux split-window "$1" -t "$2" -c "$REPO" -p "$3"
}
split -h "$SESSION:bench"   48
split -v "$SESSION:bench.1" 66
split -v "$SESSION:bench.2" 50

tmux send-keys -t "$SESSION:bench.1" "RB_REPO='$REPO' RB_LOGDIR='$LOGDIR' '$STATUS' -w 5" C-m
tmux send-keys -t "$SESSION:bench.2" "$(console_cmd "$FPGA_DEV" 'fpga-console' "$FPGA_LOG" 'FPGA console' "$SESSDIR")" C-m
tmux send-keys -t "$SESSION:bench.3" "$(console_cmd "$ESP_DEV"  'esp-live'     "$ESP_LOG"  'ESP'          "$SESSDIR")" C-m

tmux select-pane -t "$SESSION:bench.0"
tmux send-keys -t "$SESSION:bench.0" "clear; echo 'work pane. ctrl-b d detaches and leaves everything running.'; echo 'logs: $SESSDIR'" C-m

# Started with no terminal (a script, or `ssh host tools/bench/rb-watch.sh`):
# the session is up and capturing, there is just nothing to attach it to. Say so
# and exit 0 rather than failing after having done the whole job.
if [ ! -t 0 ]; then
    echo "no terminal to attach to; session '$SESSION' is running and capturing."
    echo "attach with: tmux attach -t $SESSION"
    exit 0
fi

exec tmux attach -t "$SESSION"
