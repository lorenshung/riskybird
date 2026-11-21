#!/bin/bash
# deck_tunnel.sh - SELF-HEALING reverse tunnel: Deck localhost:8080 -> this host's panel 127.0.0.1:8080.
#
# The failure this exists to fix (hit 3x): when the link drops, the local `ssh -R` dies but the Deck's
# sshd never notices (no ClientAliveInterval on SteamOS, rootfs read-only), so its old session stays
# bound to :8080 forever. Every reconnect then fails "remote port forwarding failed" and the Deck's
# kiosk shows a white page. So before EVERY (re)connect we kill stale tunnel sessions on the Deck.
#
# Only processes whose command line is EXACTLY "sshd: deck" are touched: that is what an `ssh -N`
# session's proctitle looks like. Command sessions are "sshd: deck@notty" and the pilot bridge's are
# "sshd: deck@pts/N", so neither can match. At cleanup time our own previous ssh has already exited,
# so the only thing it can hit is a dead forward. (pgrep's own cmdline can't match the anchored -x.)
DECK=${DECK:-deck@10.0.0.45}
SSH_OPTS=(-o ServerAliveInterval=15 -o ServerAliveCountMax=3 -o StrictHostKeyChecking=accept-new -o ConnectTimeout=8)
echo "[tunnel] self-healing reverse tunnel to $DECK (Ctrl-C to stop)"
while true; do
  stale=$(ssh "${SSH_OPTS[@]}" "$DECK" 'pgrep -u deck -x -f "sshd: deck" | tr "\n" " "' 2>/dev/null)
  if [ -n "${stale// /}" ]; then
    echo "[tunnel] $(date +%T) clearing stale Deck tunnel session(s): $stale"
    ssh "${SSH_OPTS[@]}" "$DECK" "kill $stale" 2>/dev/null
    sleep 1
  fi
  ssh -N -o ExitOnForwardFailure=yes "${SSH_OPTS[@]}" -R 8080:localhost:8080 "$DECK"
  echo "[tunnel] dropped rc=$? $(date +%T); reconnecting in 4s"
  sleep 4
done
