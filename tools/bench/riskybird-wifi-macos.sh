#!/usr/bin/env bash
#
# Join a riskybird drone SoftAP from a Mac WITHOUT losing the host's internet.
#
# This is the macOS counterpart of
#   backends/zephyr-chipyard-sw/samples/rose_flight_controller/tools/groundstation/riskybird_wifi_setup.sh
# which is Linux/NetworkManager only. Same goal, different mechanism, and the
# difference is not cosmetic -- it is forced by what macOS can and cannot do.
#
# WHY THIS IS NOT A PORT OF THE LINUX SCRIPT
#
# The Linux script's whole premise is a SECOND WiFi adapter: it pins a profile
# to the secondary interface and sets `ipv4.never-default yes` so the primary
# keeps the default route. That works because Linux has cfg80211/mac80211, a
# public in-tree 802.11 framework -- an out-of-tree USB WiFi chip registers with
# it and becomes an ordinary wlanN netdev, and nothing cares how many exist.
# The bench's ALFA AWUS036ACS (Realtek RTL8811AU) binds rtw88 there.
#
# macOS has no public 802.11 driver family. WiFi lives behind IO80211Family,
# which is private, kext-only and Intel-era; DriverKit -- the only supported
# way to ship a third-party driver on Apple Silicon -- publishes families for
# USB, serial, HID and Ethernet-class networking, but NONE for 802.11 clients.
# So the ALFA enumerates as a USB device with no driver and never becomes a
# network interface. Confirmed on this bench: it appears in `ioreg` as
# Realtek 0bda:0811 "802.11ac WLAN Adapter" and is absent from `ifconfig` and
# `networksetup -listallhardwareports`. That is not a missing package; there is
# no interface to write the driver against.
#
# Consequence: a Mac has ONE WiFi radio, so "drone AP + internet" cannot be two
# WiFi networks. The second path has to be a medium macOS can actually drive --
# iPhone Personal Hotspot over USB, or a USB-C/Thunderbolt Ethernet adapter.
# Given that, macOS's Service Order is a faithful stand-in for
# `ipv4.never-default`: the service ranked highest that has a router supplies
# the default route, so ranking the internet service above WiFi stops the drone
# (which advertises itself as the gateway) from stealing it.
#
#   Linux                             macOS
#   --------------------------------  ------------------------------------------
#   secondary WiFi interface          non-WiFi internet service (USB/Ethernet)
#   ipv4.never-default yes            networksetup -ordernetworkservices
#   connection.zone trusted           socketfilterfw state (checked, not changed)
#   802-11-wireless.powersave 2       no equivalent -- measured, not disabled
#   connection.autoconnect yes        macOS preferred-networks auto-join
#
# THE SAFETY PROPERTY: `up` refuses to touch the radio unless a working non-WiFi
# internet service already exists. Joining the drone AP with no second path is
# exactly how this bench lost its network mid-session, so it is a hard refusal
# rather than a warning, and `up` restores the previous state on any failure.
#
# usage:
#   riskybird-wifi-macos.sh status
#   riskybird-wifi-macos.sh up      [--ssid S] [--iface en0] [--internet-service NAME]
#   riskybird-wifi-macos.sh down
#   riskybird-wifi-macos.sh verify  [--seconds N]
#
# `up` may need sudo, but ONLY when the service order actually has to change;
# it checks first and tells you rather than prompting from inside a loop.

set -u

SSID="riskybird-5674"          # this board: ESP32-C6 MAC tail 56:74
WIFI_IFACE="en0"
INTERNET_SERVICE=""            # empty = auto-detect
DRONE_IP="192.168.4.1"
DRONE_NET="192.168.4."
TELEM_PORT=14550
CMD_PORT=14551
LEASE_TIMEOUT=25
REACH_TIMEOUT=15
VERIFY_SECONDS=10

STATE_DIR="${TMPDIR:-/tmp}/riskybird-wifi-macos"
ORDER_FILE="${STATE_DIR}/saved-service-order"
SSID_FILE="${STATE_DIR}/saved-ssid"

NETWORKSETUP=/usr/sbin/networksetup
SOCKETFILTERFW=/usr/libexec/ApplicationFirewall/socketfilterfw

# ---------------------------------------------------------------- reporting --
red()  { printf '\033[31m%s\033[0m\n' "$*"; }
grn()  { printf '\033[32m%s\033[0m\n' "$*"; }
ylw()  { printf '\033[33m%s\033[0m\n' "$*"; }
info() { printf '%s\n' "$*"; }
die()  { red "ERROR: $*" >&2; exit 1; }

# --------------------------------------------------------- service plumbing --
#
# `networksetup -listnetworkserviceorder` prints, in priority order:
#
#   (1) Wi-Fi
#   (Hardware Port: Wi-Fi, Device: en0)
#
# A leading '*' on the name means the service is disabled. Emit one
# "name<TAB>device" record per service, in order, names de-asterisked --
# -ordernetworkservices wants the plain name back. Keep services with no Device
# line too: they still belong in the saved order, with an empty device field.
service_table() {
	local out
	if ! out="$("$NETWORKSETUP" -listnetworkserviceorder 2>/dev/null)"; then
		return 1
	fi
	printf '%s\n' "$out" | awk '
		function emit() {
			if (name != "") printf "%s\t%s\n", name, dev
			name = ""
			dev = ""
		}
		/^\([0-9]+\)/ {
			emit()
			name = $0
			sub(/^\([0-9]+\) /, "", name)
			sub(/^\*/, "", name)
			next
		}
		/^\(Hardware Port:/ {
			dev = $0
			if (sub(/^.*Device: /, "", dev)) sub(/\).*$/, "", dev)
			else dev = ""
			emit()
		}
		END { emit() }
	'
}

service_for_device() { service_table | awk -F'\t' -v d="$1" '$2==d {print $1; exit}'; }
device_for_service() { service_table | awk -F'\t' -v s="$1" '$1==s {print $2; exit}'; }
ordered_service_names() { service_table | cut -f1; }

# Which interface currently carries the default route.
default_route_device() { route -n get default 2>/dev/null | awk '/interface:/{print $2; exit}'; }

iface_is_up()  { ifconfig "$1" 2>/dev/null | grep -q "status: active"; }

# ipconfig only answers for interfaces it configured by DHCP. An iPhone USB
# tether is NOT one: it comes up as `inet 192.0.0.2 netmask 0xffffffff` -- a /32
# with no lease and no router option -- so `ipconfig getifaddr` and
# `getoption router` are both EMPTY on a link that is up and carrying the
# default route. Measured on this bench; relying on ipconfig alone made the
# script refuse to run with a perfectly good internet path attached. Fall back
# to ifconfig, which reports the address however it was configured.
iface_ipv4() {
	local ip
	ip="$(ipconfig getifaddr "$1" 2>/dev/null)"
	[ -n "$ip" ] && { printf '%s' "$ip"; return 0; }
	ifconfig "$1" 2>/dev/null | awk '/^[[:space:]]*inet /{print $2; exit}'
}

# "Can this interface actually send packets off-box?" A DHCP router option
# proves it; so does already owning the default route, which is the only
# evidence a /32 tether offers.
iface_has_route() {
	[ -n "$(ipconfig getoption "$1" router 2>/dev/null | grep -v '^$')" ] && return 0
	[ "$(default_route_device)" = "$1" ]
}

# A usable internet path: not the WiFi radio, link up, a real (non self-assigned)
# IPv4, and a router to send packets to. The drone's own subnet is excluded so a
# half-finished run cannot nominate the drone as the internet.
find_internet_service() {
	local wanted="${1-}"
	local name dev ip
	while IFS=$'\t' read -r name dev; do
		[ -z "$dev" ] && continue
		if [ -n "$wanted" ] && [ "$name" != "$wanted" ]; then continue; fi
		[ "$dev" = "$WIFI_IFACE" ] && continue
		iface_is_up "$dev" || continue
		ip="$(iface_ipv4 "$dev")"
		[ -z "$ip" ] && continue
		case "$ip" in 169.254.*|"$DRONE_NET"*) continue ;; esac
		iface_has_route "$dev" || continue
		printf '%s\t%s\t%s\n' "$name" "$dev" "$ip"
		return 0
	done < <(service_table)
	return 1
}

current_ssid() {
	# Unreliable on recent macOS: without Location Services authorisation this
	# prints "You are not associated with an AirPort network." even while
	# associated. Observed on this bench. Callers must tolerate an empty result.
	local out
	out="$("$NETWORKSETUP" -getairportnetwork "$WIFI_IFACE" 2>/dev/null)"
	case "$out" in
		"Current Wi-Fi Network: "*) printf '%s' "${out#Current Wi-Fi Network: }" ;;
		*) printf '' ;;
	esac
}

alfa_present() { ioreg -p IOUSB -l -w 0 2>/dev/null | grep -q "802.11ac WLAN Adapter"; }

# ------------------------------------------------------------- UDP helpers --
# stdlib-only python3, matching riskybird_panel.py's no-pip constraint.

# PING -> :14551, expect the bridge's own "ACK PING". This is the liveness proof
# that matters: esp_bridge ACKs from its uplink thread WITHOUT involving the
# FPGA, so it works before any flight-controller image is loaded, and it
# exercises association + DHCP + UDP in BOTH directions. A DHCP lease alone
# proves none of that -- TELEMETRY_BRINGUP.md records stale associations where
# ping succeeded and zero telemetry ever arrived.
udp_ping() {
	python3 - "$DRONE_IP" "$CMD_PORT" <<'PY'
import socket, sys
host, port = sys.argv[1], int(sys.argv[2])
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(3.0)
try:
    s.sendto(b"PING\n", (host, port))
    data, src = s.recvfrom(256)
    print(data.decode("utf-8", "replace").strip())
    sys.exit(0)
except socket.timeout:
    print("(no reply)")
    sys.exit(1)
except OSError as exc:
    print(f"(socket error: {exc})")
    sys.exit(1)
finally:
    s.close()
PY
}

# Listen on :14550 and report the same figures the Linux soak reports, so the
# two benches are comparable: rate, loss proxy, and worst inter-arrival gap.
udp_listen() {
	python3 - "$TELEM_PORT" "$1" <<'PY'
import socket, statistics, sys, time
port, secs = int(sys.argv[1]), float(sys.argv[2])
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("0.0.0.0", port))
s.settimeout(0.5)
t0 = time.time(); last = None; n = 0; gaps = []; first = None
while time.time() - t0 < secs:
    try:
        data, src = s.recvfrom(2048)
    except socket.timeout:
        continue
    now = time.time()
    n += 1
    if first is None:
        first = now
        print(f"  first datagram from {src[0]}: {data[:80].decode('utf-8','replace').strip()}")
    if last is not None:
        gaps.append(now - last)
    last = now
s.close()
if n == 0:
    print(f"  0 datagrams in {secs:.0f}s on :{port}")
    sys.exit(1)
span = (last - first) if (first and last and last > first) else secs
rate = (n - 1) / span if span > 0 else 0.0
worst = max(gaps) * 1000 if gaps else 0.0
if gaps:
    median = statistics.median(gaps) * 1000
    gap_threshold = 3.0 * median
    over = sum(1 for g in gaps if g * 1000 > gap_threshold)
    print(f"  {n} datagrams, {rate:.1f} Hz, median gap {median:.1f} ms, "
          f"max gap {worst:.1f} ms, {over} gap(s) > {gap_threshold:.1f} ms (3x median)")
else:
    print(f"  {n} datagram, {rate:.1f} Hz, median gap n/a, max gap {worst:.1f} ms "
          "(need at least 2 datagrams for adaptive gap reporting)")
PY
}

# ------------------------------------------------------------------ status --
cmd_status() {
	info "== riskybird wifi (macOS) =="
	info "drone SSID     : ${SSID}"
	info "wifi interface : ${WIFI_IFACE}  ip=$(iface_ipv4 "$WIFI_IFACE" || true)  ssid=$(current_ssid || true)"
	info ""
	info "service order (highest first):"
	service_table | awk -F'\t' '{printf "  %-28s %s\n", $1, $2}'
	info ""
	local drdev drsvc
	drdev="$(default_route_device)"
	drsvc="$(service_for_device "$drdev")"
	info "default route  : ${drdev:-none} (${drsvc:-unknown service})"

	local found
	if found="$(find_internet_service)"; then
		grn "internet path  : $(printf '%s' "$found" | awk -F'\t' '{print $1" ("$2", "$3")"}')"
	else
		ylw "internet path  : NONE besides ${WIFI_IFACE}."
		ylw "                 'up' will refuse. Plug in an iPhone (Personal Hotspot,"
		ylw "                 USB) or a USB-C Ethernet adapter."
	fi

	info ""
	if alfa_present; then
		ylw "ALFA AWUS036ACS: present on USB, NO driver, not a network interface."
		ylw "                 Realtek RTL8811AU. macOS has no DriverKit 802.11 family"
		ylw "                 and IO80211Family is private/Intel-kext-only, so this"
		ylw "                 cannot be made to work here. Use it on Linux with rtw88"
		ylw "                 and the original riskybird_wifi_setup.sh."
	else
		info "ALFA           : not attached"
	fi

	local fw
	fw="$("$SOCKETFILTERFW" --getglobalstate 2>/dev/null | tr -d '\n')"
	info "app firewall   : ${fw:-unknown}"
	case "$fw" in
		*enabled*) ylw "                 If telemetry does not arrive, allow python3 inbound." ;;
	esac

	info ""
	info "note: macOS cannot disable STA power-save per interface (the Linux"
	info "      profile sets 802-11-wireless.powersave 2). If 'verify' reports"
	info "      100-400 ms gaps, that is the likely cause and it is not fixable"
	info "      from here."
}

# ---------------------------------------------------------------------- up --
restore_on_failure() {
	local status="${1:-1}"
	trap - EXIT INT TERM
	ylw "restoring previous network state..."
	cmd_down || true
	exit "$status"
}

cmd_up() {
	# 1. The safety gate. Nothing is touched before this passes.
	local found isvc idev iip requested
	requested="$INTERNET_SERVICE"
	if ! found="$(find_internet_service "$requested")"; then
		if [ -n "$requested" ]; then
			red "The requested internet service '${requested}' is not an active,"
			red "routed non-WiFi path. Refusing to join the drone AP."
		else
			red "No non-WiFi internet service is active, so joining the drone AP"
			red "would take this Mac off the network entirely. Refusing."
		fi
		info ""
		info "Interfaces considered (WiFi ${WIFI_IFACE} excluded by design):"
		service_table | awk -F'\t' -v w="$WIFI_IFACE" '$2!=w {printf "  %-28s %s\n", $1, $2}'
		info ""
		info "Provide one of:"
		info "  - iPhone plugged in over USB with Personal Hotspot enabled"
		info "  - a USB-C / Thunderbolt Ethernet adapter on a live jack"
		info ""
		info "Then re-run. If you deliberately want the offline-window workflow"
		info "instead, that is a different tool -- this one exists to keep you online."
		exit 2
	fi
	isvc="$(printf '%s' "$found" | cut -f1)"
	idev="$(printf '%s' "$found" | cut -f2)"
	iip="$(printf '%s' "$found" | cut -f3)"
	[ -z "$idev" ] && die "no device for internet service '${isvc}'"
	grn "internet path: ${isvc} (${idev}, ${iip})"

	# 2. Save what we are about to change.
	local order_before wifi_svc first_svc wifi_ip_before saved_ssid
	if ! order_before="$(ordered_service_names)" || [ -z "$order_before" ]; then
		die "could not read the current network service order"
	fi
	if ! wifi_svc="$(service_for_device "$WIFI_IFACE")" || [ -z "$wifi_svc" ]; then
		die "no network service found for WiFi interface '${WIFI_IFACE}'"
	fi
	if ! printf '%s\n' "$order_before" | grep -qxF -- "$isvc"; then
		die "internet service '${isvc}' is missing from the service order"
	fi
	first_svc="$(printf '%s\n' "$order_before" | head -1)"

	mkdir -p "$STATE_DIR" || die "could not create state directory '${STATE_DIR}'"
	wifi_ip_before="$(iface_ipv4 "$WIFI_IFACE")"
	case "$wifi_ip_before" in
		"$DRONE_NET"*)
			# A repeated `up` must not overwrite the pre-drone state saved by the
			# first run; without it, `down` could not get the bench back online.
			[ -s "$ORDER_FILE" ] && [ -f "$SSID_FILE" ] \
				|| die "already on the drone network, but no complete saved state exists"
			info "already on the drone network; preserving the original saved state"
			;;
		*)
			saved_ssid="$(current_ssid)"
			printf '%s\n' "$order_before" > "$ORDER_FILE" \
				|| die "could not save the current service order"
			printf '%s' "$saved_ssid" > "$SSID_FILE" \
				|| die "could not save the current SSID"
			info "saved service order and current SSID (${saved_ssid:-unknown})"
			;;
	esac

	# EXIT is deliberate: unlike ERR without `set -e`, it also catches die()'s
	# bare `exit`. Arm it before the first network mutation, including reordering.
	trap 'restore_on_failure $?' EXIT
	trap 'restore_on_failure 130' INT
	trap 'restore_on_failure 143' TERM

	# 3. Rank the internet service above WiFi -- the never-default equivalent.
	#    Only invoke sudo if the order is actually wrong; a correct order needs
	#    no privilege at all, and most re-runs are already correct.
	if [ "$first_svc" != "$isvc" ]; then
		info "raising '${isvc}' above '${wifi_svc}' in the service order (needs sudo)"
		local reordered service
		reordered="$( { printf '%s\n' "$isvc"; printf '%s\n' "$order_before" | grep -vxF -- "$isvc"; } )"
		# Service names contain spaces ("iPhone USB", "Thunderbolt Bridge"), so
		# append one complete line at a time as a separate argv entry. This is
		# Bash 3.2-safe and also avoids pathname expansion on service names.
		set --
		while IFS= read -r service; do
			[ -n "$service" ] && set -- "$@" "$service"
		done < <(printf '%s\n' "$reordered")
		[ "$#" -gt 0 ] || die "refusing to apply an empty service order"
		sudo "$NETWORKSETUP" -ordernetworkservices "$@" \
			|| die "could not reorder network services (sudo required)"
		grn "service order updated"
	else
		info "service order already correct ('${isvc}' first) -- no sudo needed"
	fi

	# 4. Join the drone.
	#
	# Always associate fresh. A cached DHCP lease is worse than no lease here:
	# after a drone reboot, ping can succeed with the host's old address while
	# the bridge's fresh DHCP server has no lease to enumerate, so unicast
	# telemetry is silently sent to zero clients. `ipconfig set en0 DHCP` is the
	# direct way to force a renew but needs root; a radio cycle does not.
	info "cycling ${WIFI_IFACE} radio to force a fresh association and DHCP lease"
	"$NETWORKSETUP" -setairportpower "$WIFI_IFACE" off >/dev/null 2>&1 \
		|| die "could not power off ${WIFI_IFACE}"
	sleep 2
	"$NETWORKSETUP" -setairportpower "$WIFI_IFACE" on >/dev/null 2>&1 \
		|| die "could not power on ${WIFI_IFACE}"
	sleep 3

	info "joining ${SSID} on ${WIFI_IFACE}..."
	"$NETWORKSETUP" -setairportnetwork "$WIFI_IFACE" "$SSID" \
		|| die "could not associate with ${SSID} (is the drone powered?)"

	# 5. Wait for the drone's DHCP lease.
	local waited=0 ip=""
	while [ "$waited" -lt "$LEASE_TIMEOUT" ]; do
		ip="$(iface_ipv4 "$WIFI_IFACE")"
		case "$ip" in "$DRONE_NET"*) break ;; esac
		sleep 1; waited=$((waited + 1)); ip=""
	done
	[ -z "$ip" ] && die "no ${DRONE_NET}x lease after ${LEASE_TIMEOUT}s (got '$(iface_ipv4 "$WIFI_IFACE")')"
	grn "lease: ${ip}"

	# 6. The drone advertises itself as gateway. Confirm it did not win.
	local drdev
	if ! find_internet_service "$isvc" >/dev/null; then
		die "internet service '${isvc}' stopped being usable after association"
	fi
	drdev="$(default_route_device)"
	if [ -z "$drdev" ]; then
		red "there is no default route after joining the drone"
		die "the non-WiFi internet path did not hold"
	elif [ "$drdev" = "$WIFI_IFACE" ]; then
		red "the drone took the default route (${drdev}) -- internet would be dead"
		die "service order did not hold; run 'status' and re-check '${isvc}'"
	fi
	grn "default route still on ${drdev} ($(service_for_device "$drdev"))"

	# 7. A lease can appear before macOS installs the scoped interface route or
	# resolves ARP. Wait for both the route and an ICMP reply before probing UDP.
	local reach_deadline route_dev="" reachable=0
	reach_deadline=$((SECONDS + REACH_TIMEOUT))
	info "waiting up to ${REACH_TIMEOUT}s for ${DRONE_IP} via ${WIFI_IFACE}..."
	while [ "$SECONDS" -lt "$reach_deadline" ]; do
		route_dev="$(route -n get "$DRONE_IP" 2>/dev/null | awk '/interface:/{print $2; exit}')"
		if [ "$route_dev" = "$WIFI_IFACE" ] \
			&& ping -c 1 -t 1 "$DRONE_IP" >/dev/null 2>&1; then
			reachable=1
			break
		fi
		[ "$SECONDS" -lt "$reach_deadline" ] && sleep 1
	done
	if [ "$reachable" -ne 1 ]; then
		red "${DRONE_IP} did not become reachable via ${WIFI_IFACE} (route: ${route_dev:-none})"
		die "associated and leased, but the drone did not become reachable within ${REACH_TIMEOUT}s"
	fi
	grn "drone reachable via ${WIFI_IFACE}"

	# Prove the link carries UDP both ways. Retry because one lost datagram on a
	# fresh association is not enough evidence that the bridge is down.
	info "probing the bridge (PING -> :${CMD_PORT})..."
	local ack="" udp_attempt udp_ok=0
	for udp_attempt in 1 2 3; do
		if ack="$(udp_ping)"; then
			udp_ok=1
			grn "bridge replied: ${ack}"
			break
		fi
		if [ "$udp_attempt" -lt 3 ]; then
			ylw "no ACK (attempt ${udp_attempt}/3); retrying..."
			sleep 1
		fi
	done
	if [ "$udp_ok" -ne 1 ]; then
		red "no ACK from ${DRONE_IP}:${CMD_PORT}"
		die "associated and leased, but the bridge is not answering"
	fi

	trap - EXIT INT TERM
	info ""
	grn "UP. drone ${DRONE_IP} reachable on ${WIFI_IFACE}; internet on ${idev}."
	info "    telemetry: $0 verify"
	info "    panel:     python3 backends/zephyr-chipyard-sw/samples/rose_flight_controller/tools/groundstation/riskybird_panel.py --drone ${DRONE_IP}"
	info "    when done: $0 down"
}

# -------------------------------------------------------------------- down --
cmd_down() {
	local saved_ssid="" ip="" waited=0 drdev="" route_waited=0
	[ -f "$SSID_FILE" ] && saved_ssid="$(cat "$SSID_FILE")"

	ip="$(iface_ipv4 "$WIFI_IFACE")"
	case "$ip" in
		""|169.254.*|"$DRONE_NET"*)
			if [ -n "$saved_ssid" ]; then
				info "rejoining ${saved_ssid}"
				"$NETWORKSETUP" -setairportnetwork "$WIFI_IFACE" "$saved_ssid" >/dev/null 2>&1 || true
			else
				# The SSID readback is unreliable without Location Services, so fall back
				# to a power cycle, which makes macOS auto-join the highest-priority
				# preferred network on its own.
				ylw "previous SSID unknown -- cycling WiFi so macOS auto-joins a preferred network"
				"$NETWORKSETUP" -setairportpower "$WIFI_IFACE" off >/dev/null 2>&1 || true
				sleep 2
				"$NETWORKSETUP" -setairportpower "$WIFI_IFACE" on >/dev/null 2>&1 || true
			fi

			while [ "$waited" -lt 20 ]; do
				ip="$(iface_ipv4 "$WIFI_IFACE")"
				case "$ip" in ""|169.254.*|"$DRONE_NET"*) ;; *) break ;; esac
				sleep 1; waited=$((waited + 1))
			done
			case "$ip" in
				""|169.254.*|"$DRONE_NET"*)
					ylw "DOWN incomplete: ${WIFI_IFACE} has no non-drone address yet."
					ylw "Leaving the internet service first; check Wi-Fi in the menu bar, then run 'down' again."
					return 1
					;;
			esac
			;;
		*) info "${WIFI_IFACE} is already off the drone network (${ip})" ;;
	esac

	# Restore WiFi's old priority only after it has a non-drone address. Doing
	# this first would briefly make the drone the default route, and could leave
	# the bench offline if reassociation failed.
	if [ -f "$ORDER_FILE" ]; then
		local saved_order current_order service
		saved_order="$(cat "$ORDER_FILE")"
		[ -n "$saved_order" ] || { ylw "saved service order is empty; not applying it"; return 1; }
		current_order="$(ordered_service_names)" || current_order=""
		if [ "$current_order" != "$saved_order" ]; then
			set --
			while IFS= read -r service; do
				[ -n "$service" ] && set -- "$@" "$service"
			done < "$ORDER_FILE"
			[ "$#" -gt 0 ] || { ylw "saved service order is empty; not applying it"; return 1; }
			info "restoring saved service order"
			if ! sudo "$NETWORKSETUP" -ordernetworkservices "$@" >/dev/null 2>&1; then
				ylw "could not restore service order (needs sudo); run 'down' again"
				return 1
			fi
		else
			info "saved service order is already restored"
		fi
	fi

	# An address alone is not enough to call the rollback complete. Give macOS a
	# few seconds to reinstall a default route after the association/order changes.
	while [ "$route_waited" -lt 5 ]; do
		drdev="$(default_route_device)"
		[ -n "$drdev" ] && break
		sleep 1; route_waited=$((route_waited + 1))
	done
	if [ -z "$drdev" ]; then
		ylw "DOWN incomplete: no default route is installed yet; run 'down' again"
		return 1
	fi

	if ! rm -f "$ORDER_FILE" "$SSID_FILE"; then
		ylw "network restored, but saved state could not be cleared from ${STATE_DIR}"
		return 1
	fi
	grn "DOWN. ${WIFI_IFACE} is ${ip}, default route ${drdev}"
}

# ------------------------------------------------------------------ verify --
cmd_verify() {
	local ip; ip="$(iface_ipv4 "$WIFI_IFACE")"
	case "$ip" in
		"$DRONE_NET"*) info "on the drone network as ${ip}" ;;
		*) die "not on the drone network (${WIFI_IFACE} is '${ip:-down}') -- run 'up' first" ;;
	esac

	info "uplink  PING -> ${DRONE_IP}:${CMD_PORT}"
	local ack
	if ack="$(udp_ping)"; then grn "  ${ack}"; else red "  no ACK"; fi

	info "downlink listening ${VERIFY_SECONDS}s on :${TELEM_PORT}"
	if udp_listen "$VERIFY_SECONDS"; then
		grn "  downlink alive"
	else
		ylw "  nothing on :${TELEM_PORT}."
		ylw "  Expected when the FPGA is not transmitting: esp_bridge only forwards"
		ylw "  what arrives on its UART. Load esp_uart or a telemetry-emitting"
		ylw "  flight_controller build on the FPGA first. The PING above already"
		ylw "  proves the radio, the lease and both UDP directions."
	fi
}

# -------------------------------------------------------------------- main --
CMD=""
while [ "$#" -gt 0 ]; do
	case "$1" in
		status|up|down|verify) CMD="$1"; shift ;;
		--ssid)             [ "$#" -ge 2 ] || die "--ssid requires a value"; SSID="$2"; shift 2 ;;
		--iface)            [ "$#" -ge 2 ] || die "--iface requires a value"; WIFI_IFACE="$2"; shift 2 ;;
		--internet-service) [ "$#" -ge 2 ] || die "--internet-service requires a value"; INTERNET_SERVICE="$2"; shift 2 ;;
		--seconds)          [ "$#" -ge 2 ] || die "--seconds requires a value"; VERIFY_SECONDS="$2"; shift 2 ;;
		-h|--help) sed -n '2,/^set -u$/p' "$0" | sed '/^set -u$/d; s/^# \{0,1\}//'; exit 0 ;;
		*) die "unknown argument: $1" ;;
	esac
done
[ -z "$CMD" ] && { sed -n '/^# usage:/,/^$/p' "$0" | sed 's/^# \{0,1\}//'; exit 1; }

case "$CMD" in
	status) cmd_status ;;
	up)     cmd_up ;;
	down)   cmd_down ;;
	verify) cmd_verify ;;
esac
