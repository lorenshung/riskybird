#!/usr/bin/env bash
#
# One screen of "what is the drone doing right now", for someone who is not in
# the room.
#
#   rb-status.sh          one shot
#   rb-status.sh -w [N]   refresh every N seconds (default 5)
#   rb-status.sh -s       list past rb-watch sessions and what they logged
#   rb-status.sh -p       re-probe the adapters ONCE, then show the screen
#
# SAFE TO RUN AT ANY TIME, AND THAT IS THE WHOLE POINT. It programs nothing,
# writes nothing to the board, opens no serial device and opens no JTAG adapter.
# Everything it knows about the drone it reads out of the LOG FILES the console
# panes are writing. Only one reader per tty works, so a status tool that opened
# /dev/ttyUSB* would be stealing half the bytes from the console pane and
# corrupting both.
#
# THE ADAPTER TABLE IS CACHED, NEVER PROBED IN THE LOOP. `rb bench` shifts an
# IDCODE, which is a real open of a real FTDI adapter. Doing that on a 5-second
# refresh was wrong twice over:
#
#   * it fights a live OpenOCD for the adapter, and re-opens an MPSSE dozens of
#     times an hour on a bench where dirty MPSSE cycling has dropped USB buses;
#   * it HUNG. `timeout` runs its child in a new process group, OpenOCD reads
#     the inherited tty stdin, and a background process group that reads the
#     terminal gets SIGTTIN and stops. Measured in a tmux pane: `python3 ./rb
#     bench` parked in state T and the status pane froze forever on the word
#     ADAPTERS, which is the single worst failure mode a remote status screen
#     can have -- a frozen screen looks like a calm bench.
#
# So `rb-watch.sh` probes once at session start and writes the cache; this shows
# it with its age; and `-p` is the ONE operator-initiated action that re-probes.
# Every `rb` call here redirects stdin from /dev/null so the SIGTTIN path cannot
# come back.
#
# Every line degrades to a clear "ABSENT" rather than an error when the bench is
# unplugged, because a remote operator needs "the drone is not connected" to look
# different from "the script is broken".
#
# The single most important line is BATTERY. It is the only thing that decides
# whether a motor can physically turn, and it is the thing a remote operator
# cannot see. Read it before you believe anything else on this screen -- and read
# what it actually says: "no vbat in telemetry" means UNKNOWN, not "no battery".
set -u

# The checkout this script lives in, two levels up from tools/bench.
REPO="${RB_REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
LOGDIR="${RB_LOGDIR:-$HOME/riskybird-logs}"
ESP_LOG="$LOGDIR/esp-live.log"
FPGA_LOG="$LOGDIR/fpga-console.log"
CACHE="$LOGDIR/.rb-bench-cache"

c()  { printf '\033[%sm%s\033[0m' "$1" "$2"; }
ok() { c '32' "$1"; }; warn(){ c '33' "$1"; }; bad(){ c '31' "$1"; }; dim(){ c '90' "$1"; }
hdr(){ printf '\n\033[1;36m%s\033[0m\n' "$1"; }

# Age of a path in seconds, following symlinks. -L is load-bearing: the two log
# paths are symlinks into the current session directory, and a plain `stat` on a
# symlink reports when the LINK was made, not when data last arrived. Without it
# every freshness check below silently reports "no recent output" forever.
age_of() {
    local f="$1" m
    m=$(stat -Lc %Y "$f" 2>/dev/null) || return 1
    echo $(( $(date +%s) - m ))
}

# Most recent line matching $2 in file $1, within the last $3 seconds, else "".
fresh_line() {
    local f="$1" pat="$2" maxage="${3:-30}" age
    [ -f "$f" ] || return 0
    age=$(age_of "$f") || return 0
    [ "$age" -gt "$maxage" ] && return 0
    tail -c 20000 "$f" 2>/dev/null | tr -d '\r\000' | grep -a "$pat" | tail -1
}

# Same, with no freshness bound: for state that LATCHES and stays true until the
# board is reset, where "it scrolled off 40 seconds ago" does not make it false.
last_line() {
    local f="$1" pat="$2"
    [ -f "$f" ] || return 0
    tail -c 40000 "$f" 2>/dev/null | tr -d '\r\000' | grep -aE "$pat" | tail -1
}

# The cached adapter table. Rendering NEVER probes; it only reads this file.
bench_table() { [ -f "$CACHE" ] && cat "$CACHE"; }

# The one thing in this script that touches the bench, and only when the
# operator asks for it with -p. Refuses while OpenOCD is running, because
# shifting an IDCODE would fight a live debug session for the adapter.
#
# `< /dev/null` is load-bearing, not tidiness: without it OpenOCD inherits the
# terminal's stdin, `timeout` has already put it in a background process group,
# reading the tty raises SIGTTIN and the probe stops dead instead of finishing.
bench_probe() {
    if pgrep -x openocd >/dev/null 2>&1; then
        echo "openocd is running -- not probing; a debug session owns the adapter" >&2
        return 1
    fi
    local out
    out=$(cd "$REPO" && timeout 25 ./rb bench 2>/dev/null </dev/null)
    if [ -n "$out" ]; then printf '%s\n' "$out" > "$CACHE"; return 0; fi
    echo "rb bench produced nothing -- toolchain or permissions, not the drone" >&2
    return 1
}

render() {
    printf '\033[H\033[2J'
    printf '\033[1m RiskyBird bench \033[0m  %s   host %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$(hostname -s)"
    local sess; sess=$(readlink "$LOGDIR/sessions/current" 2>/dev/null)
    if [ -n "$sess" ]; then
        printf ' session %s\n' "$(dim "$(basename "$sess")")"
    else
        printf ' session %s\n' "$(dim 'none -- run tools/bench/rb-watch.sh')"
    fi
    printf '%s\n' "$(dim '─────────────────────────────────────────────────────────────────')"

    # ---- 1. adapters -----------------------------------------------------
    hdr "ADAPTERS"
    local bench bage; bench=$(bench_table); bage=$(age_of "$CACHE" 2>/dev/null || echo '?')
    if pgrep -x openocd >/dev/null 2>&1; then
        echo "  $(warn 'OpenOCD is running') -- a debug session holds an adapter; not probing"
    fi
    if [ -z "$bench" ]; then
        echo "  $(warn 'never probed') -- no cached rb bench result"
        echo "                   $(dim 'run rb-status.sh -p once, or start tools/bench/rb-watch.sh')"
    else
        local bit dbg
        bit=$(printf '%s\n' "$bench" | awk '$1=="bitstream"{print $2" ("$3")"; exit}')
        # "debug" when the part is configured, "debug?" when the debug TAP is
        # shifting all ones -- which is what an UNCONFIGURED part does. Matching
        # only "debug" reported ABSENT for an adapter that was plugged in and
        # working, which is the exact failure this screen exists to avoid.
        dbg=$(printf '%s\n' "$bench" | awk '$1 ~ /^debug/{print $2" console="$5; exit}')
        [ -n "$bit" ] && echo "  bitstream link   $(ok "$bit")" || echo "  bitstream link   $(bad 'ABSENT') -- cannot program the FPGA"
        [ -n "$dbg" ] && echo "  debug link       $(ok "$dbg")" || echo "  debug link       $(bad 'ABSENT') -- cannot load or halt Zephyr"
        # Age matters: USB locations and ttyUSB numbers renumber on every
        # replug, so a table from before the last replug is fiction. Say how
        # old it is rather than presenting it as now.
        printf '  %s\n' "$(dim "cached, probed ${bage}s ago -- rb-status.sh -p to re-probe (locations renumber on replug)")"
    fi

    # ---- 2. FPGA ---------------------------------------------------------
    hdr "FPGA"
    if printf '%s\n' "$bench" | grep -q "0x00000001"; then
        echo "  configuration    $(ok 'CONFIGURED') -- Rocket debug TAP answering"
    elif printf '%s\n' "$bench" | grep -q "0x13636093"; then
        echo "  configuration    $(warn 'UNCONFIGURED') -- config TAP only, no design loaded"
    elif printf '%s\n' "$bench" | grep -q "all ones"; then
        echo "  configuration    $(warn 'UNCONFIGURED') -- debug TAP shifting all ones, nothing driving TDO"
    else
        echo "  configuration    $(dim 'unknown') -- no arty200t adapter visible"
    fi

    # ---- 3. ESP ----------------------------------------------------------
    hdr "ESP32-C6"
    local espdev; espdev=$(ls /dev/serial/by-id/*Espressif* 2>/dev/null | head -1)
    if [ -z "$espdev" ]; then
        echo "  usb              $(bad 'ABSENT')"
    else
        echo "  usb              $(ok "$(basename "$(readlink -f "$espdev")")")"
        local gate cmd link
        gate=$(fresh_line "$ESP_LOG" "MOTOR GATE" 30)
        cmd=$(fresh_line  "$ESP_LOG" "MOTOR CMD"  30)
        link=$(fresh_line "$ESP_LOG" "link="      30)
        if   [ -n "$gate" ]; then echo "  firmware         $(ok 'motor_safe') -- gate probe, motors CANNOT be driven by the ESP"
        elif [ -n "$cmd"  ]; then echo "  firmware         $(warn 'esp_motors') -- the ESP OWNS all four motor gates"
        else                      echo "  firmware         $(dim 'no recent output') -- is a console pane attached?"; fi
        [ -n "$gate" ] && echo "  gates            $gate"
        [ -n "$cmd"  ] && echo "  commanded        $cmd"
        [ -n "$link" ] && echo "  link             $link"
    fi

    # ---- 4. the line that matters ---------------------------------------
    #
    # READ THIS BEFORE TRUSTING THE LINE BELOW. Battery PRESENCE IS NOT
    # OBSERVABLE on this board. U4 is an MCP73831 charger whose VDD is USB VBUS
    # and whose output is wired straight to +BATT, so with USB connected and NO
    # pack fitted it charges the ~200 uF of bulk capacitance to its 4.20 V float
    # and terminates. An empty board therefore reads ~4.2 V -- indistinguishable
    # from a fully charged pack. The only discriminator is load response, and
    # deliberately loading the motors to find out is exactly what a remote
    # operator must not do.
    #
    # So this section reports whether the +BATT RAIL IS ENERGISED. It can tell
    # you motors MIGHT turn. It can never tell you they cannot.
    # ---- ARE THE MOTORS TURNING *RIGHT NOW*? ----------------------------
    #
    # Read from the ESP's log, not the FPGA's. The ESP is the last component
    # before the gates and prints what it APPLIED at 4 Hz; the FPGA prints what
    # it COMMANDED, interleaved with flow/telemetry lines and only every
    # ROSE_TELEM_DIV iterations. Measured: with motors genuinely at 200 per
    # mille the FPGA-derived line still read duty=[0.000 ...] because the
    # nearest telemetry line predated the window. A 5 s refresh against a 5 s
    # motor window misses the spin more often than it catches it, and a status
    # screen that says "idle" while props are turning is the single most
    # dangerous thing this tool could print.
    hdr "ARE THE MOTORS TURNING?"
    local espcmd espflags
    espcmd=$(fresh_line "$ESP_LOG" "MOTOR CMD" 15)
    espflags=$(fresh_line "$ESP_LOG" "link=" 15)
    if [ -z "$espcmd" ]; then
        echo "  $(warn 'NO RECENT ESP OUTPUT') -- cannot tell. Assume they might be."
    elif printf '%s\n' "$espcmd" | grep -q "M1=0 M2=0 M3=0 M4=0"; then
        echo "  $(ok 'all four at zero') -- $(printf '%s' "$espcmd" | sed 's/.*mille: //')"
    else
        echo "  $(bad '*** MOTORS ARE TURNING ***')  $(printf '%s' "$espcmd" | sed 's/.*mille: //')"
    fi
    [ -n "$espflags" ] && printf '  %s\n' "$(printf '%s' "$espflags" | sed 's/^ *//' | cut -c1-96)"

    hdr "CAN A MOTOR TURN RIGHT NOW?"
    local telem vbat bstate
    telem=$(fresh_line "$FPGA_LOG" "it=" 30)
    vbat=$(printf '%s\n' "$telem" | grep -ao "vbat=[0-9][0-9.]*" | head -1 | cut -d= -f2)
    bstate=$(printf '%s\n' "$telem" | grep -ao "batt=[a-z]*" | head -1 | cut -d= -f2)

    case "${bstate:-}" in
      off)    echo "  +BATT rail       $(warn 'NOT MEASURED') -- this build has battery sense off"
              echo "                   $(dim 'assume a motor CAN turn; --mode esp-motors enables the sense')" ;;
      nodata) echo "  +BATT rail       $(warn 'NO DATA') -- sense is on but the ADS7128 has never answered"
              echo "                   $(dim 'assume a motor CAN turn')" ;;
      stale)  echo "  +BATT rail       $(bad "STALE ${vbat:-?} V") -- polls are failing, this number is frozen"
              echo "                   $(dim 'a power fault can present as a healthy unmoving voltage')" ;;
      meas)   if [ -n "${vbat:-}" ] && awk "BEGIN{exit !($vbat > 1.0)}" 2>/dev/null; then
                  echo "  +BATT rail       $(bad "ENERGISED  ${vbat} V") -- A MOTOR CAN TURN. Not for unattended use."
                  echo "                   $(dim 'NOTE: ~4.2 V on USB with no pack fitted looks identical to a full pack.')"
              else
                  echo "  +BATT rail       $(warn "${vbat:-?} V") -- below the energised threshold, but see the note above"
              fi ;;
      *)      echo "  +BATT rail       $(warn 'UNKNOWN') -- no battery field in the captured telemetry"
              echo "                   $(dim 'no telemetry, or a build predating the batt= token. Assume a motor CAN turn.')" ;;
    esac

    # The commitment window is what actually bounds an UNATTENDED bench, and it
    # is the one bound that does not depend on the rail telling the truth:
    # uncommitted, all four motors latch off 5 s after they first spin. Latched
    # state does not expire, so this is deliberately not freshness-bounded --
    # "it scrolled past 40 seconds ago" does not make it false.
    local gate_ev
    gate_ev=$(last_line "$FPGA_LOG" "COMMIT GATE|MOTOR GATE: (WINDOW|MOTORS LIVE|\*\*\* COMMITTED)")
    if [ -n "$gate_ev" ]; then
        # UNCOMMITTED must be tested BEFORE COMMITTED: a `*COMMITTED*` glob
        # matches "UNCOMMITTED BUILD" too, and read a 5 s safety default as a
        # 60 s flight-test commitment -- the wrong way round, on the one line
        # that bounds an unattended bench. Caught in testing; kept in this order.
        case "$gate_ev" in
            *UNCOMMITTED*)
                echo "  commit gate      $(ok 'UNCOMMITTED') -- 5 s motor budget, then latched off" ;;
            *EXPIRED*)
                echo "  commit gate      $(ok 'WINDOW EXPIRED') -- motors latched off until board reset" ;;
            *"*** COMMITTED"*)
                echo "  commit gate      $(bad 'COMMITTED') -- 60 s motor budget. PROPS ARE LIVE." ;;
            *"WINDOW OPEN"*|*"MOTORS LIVE"*)
                echo "  commit gate      $(bad 'WINDOW OPEN') -- motors are live right now. $gate_ev" ;;
            *)  echo "  commit gate      $gate_ev" ;;
        esac
    fi

    local duty; duty=$(printf '%s\n' "$telem" | grep -ao "duty=\[[^]]*\]" | head -1)
    [ -n "$duty" ] && echo "  commanded duty   $duty"
    local tilt; tilt=$(printf '%s\n' "$telem" | grep -ao "tilt=[^ ]*" | head -1)
    [ -n "$tilt" ] && echo "  guard            $tilt $(printf '%s' "$telem" | grep -ao '|a|=[0-9.]*' | head -1)"

    # ---- 5. recent events ------------------------------------------------
    hdr "RECENT"
    local ev
    ev=$(tail -c 40000 "$FPGA_LOG" 2>/dev/null | tr -d '\r\000' \
         | grep -aiE "EMERGENCY|CUTOFF|WINDOW EXPIRED|WINDOW OPEN|GUARD|BOARD (READY|NOT READY)" | tail -4)
    [ -n "$ev" ] && printf '  %s\n' "$ev" || echo "  $(dim 'nothing notable in the FPGA console log')"

    hdr "LOGS"
    for f in "$FPGA_LOG" "$ESP_LOG"; do
        if [ -f "$f" ]; then
            # -L everywhere: these are symlinks into the session directory, and
            # a bare `du` on a symlink reports 0 while the target holds 40 MB.
            printf '  %-22s %8s  last write %s  %s\n' "$(basename "$f")" \
                "$(du -hL "$f" 2>/dev/null | cut -f1)" \
                "$(date -r "$f" '+%H:%M:%S' 2>/dev/null)" \
                "$(dim "$(basename "$(readlink -f "$f" 2>/dev/null)")")"
        else
            printf '  %-22s %s\n' "$(basename "$f")" "$(dim 'not being captured')"
        fi
    done
    printf '\n%s\n' "$(dim 'reads log files only -- never a serial device; programs nothing, writes nothing to the board')"
}

# Past sessions, for "what happened while I was away". Reads only the log tree.
sessions() {
    local d
    printf '%-22s %10s  %s\n' "SESSION" "SIZE" "FIRST / LAST CONSOLE ACTIVITY"
    for d in "$LOGDIR"/sessions/*/; do
        [ -d "$d" ] || continue
        case "$(basename "$d")" in current) continue ;; esac
        local first last
        first=$(head -1 "$d"/fpga-console.*.log 2>/dev/null | grep -a '^====' | head -1)
        last=$(cat "$d"/fpga-console.*.log 2>/dev/null | grep -a '^====' | tail -1)
        printf '%-22s %10s  %s\n' "$(basename "$d")" "$(du -sh "$d" 2>/dev/null | cut -f1)" \
            "${first:-no console capture}"
        [ -n "$last" ] && [ "$last" != "$first" ] && printf '%-22s %10s  %s\n' "" "" "$last"
    done
    echo
    echo "grep a session:  grep -a 'CUTOFF\|WINDOW\|EMERGENCY' $LOGDIR/sessions/<id>/fpga-console.*.log"
    echo "read it in order: cat $LOGDIR/sessions/<id>/fpga-console.*.log | less"
}

case "${1:-}" in
    -w) while true; do render; sleep "${2:-5}"; done ;;
    -s|--sessions) sessions ;;
    -p|--probe) bench_probe; render ;;
    *)  render ;;
esac
