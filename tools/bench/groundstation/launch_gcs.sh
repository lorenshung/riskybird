#!/bin/bash
# riskybird ground station -- Steam Deck Game Mode launcher (deploy to the Deck as
# ~/riskybird-gcs/launch_gcs.sh; registered with steamos-add-to-steam).
# The panel + reverse tunnel are hosted on CobbledSlate (keeps the WiFi dongle -> drone); this just
# opens the kiosk browser at the tunnelled localhost:8080.
#
# WHITE-SCREEN FIX: a kiosk that loads a dead URL can't reload itself, so never launch until the
# panel actually ANSWERS. A dead tunnel leaves the Deck's sshd holding :8080 (port listening, nothing
# behind it): if that's what we see, kill the stale session ourselves -- CobbledSlate's self-healing
# deck_tunnel.sh rebinds within ~4 s. Only exact "sshd: deck" (ssh -N tunnel sessions) is touched.
URL="http://localhost:8080"
for i in $(seq 1 120); do                         # up to ~60 s
  curl -sf --max-time 1 "$URL/config" >/dev/null 2>&1 && break
  if [ $((i % 6)) -eq 0 ] && ss -ltn 2>/dev/null | grep -q '127.0.0.1:8080'; then
    pkill -u "$USER" -x -f "sshd: deck" 2>/dev/null   # port held but dead -> clear the zombie
  fi
  sleep 0.5
done
# Dedicated profile so there is no first-run onboarding tab over the dashboard.
PROF="$HOME/.riskybird-ff"
mkdir -p "$PROF"
# Keep the Deck AWAKE while the GCS is open. A suspend (Steam idle timer / power button) takes WiFi
# down: the tunnel and the pilot bridge drop, stick commands stop, and the drone's link-loss failsafe
# lands it; after wake the bridge restarts with engage OFF (seen 2026-11-17: 22:22-22:29 and 20:27-21:01).
# A logind block-inhibitor on sleep+idle for the browser's lifetime. If Steam overrides it, also set
# Settings > Power > sleep delay to Never for flight sessions.
# Probe first: polkit only grants block-inhibitors to the ACTIVE local session (denied over SSH), and an
# exec that fails would mean no browser at all -- the white screen again. So fall back to a plain launch.
FF=(flatpak run org.mozilla.firefox --kiosk --profile "$PROF" "$URL")
if systemd-inhibit --what=sleep:idle --mode=block --who="riskybird GCS" --why="probe" true 2>/dev/null; then
  exec systemd-inhibit --what=sleep:idle --mode=block --who="riskybird GCS" --why="flying the drone" "${FF[@]}"
fi
echo "launch_gcs: sleep inhibitor DENIED -- the Deck can still suspend; set Steam's sleep delay to Never" >&2
exec "${FF[@]}"
