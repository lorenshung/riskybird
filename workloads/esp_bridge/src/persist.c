/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Surviving a power cut: what the ESP does when the whole drone powers up.
 *
 * At power-on the FPGA configures itself from address 0 of its flash -- the
 * GOLDEN bitstream, written once over JTAG -- and its SoC sits in the boot ROM
 * with no program, because programs live in DDR. This file stores, in the
 * ESP's own flash, the two things needed to get from there back to a working
 * drone without anyone at the bench:
 *
 *   policy   boot golden or the UPDATE bitstream (flash 0x800000, written by
 *            tools/rb/esp_flash.py), and whether to reload the stashed program
 *   stash    the last program the host asked to keep (rb: esp_tsi_load.py
 *            --stash), as a flat list of segments plus its entry point
 *
 * Both live in the image-1 partition (slot1_partition, 1.75 MiB), which this
 * image does not otherwise use: it boots with ESP_SIMPLE_BOOT, not MCUboot.
 * (If MCUboot OTA is ever enabled, move this.)
 *
 * THE POWER-ON SEQUENCE (only when the ESP's reset cause is power-on or
 * brownout: an ESP crash, watchdog or reflash must never reset a SoC that may
 * be flying):
 *
 *   1. If uart1 has already delivered traffic, a program is running: stop.
 *   2. PING until a loader answers (<= 15 s): that id is the golden image's.
 *   3. If policy says UPDATE and it has not failed before: BOOT 0x800000, then
 *      PING again. No answer, or the golden id again (configuration fallback
 *      after a bad image), marks UPDATE bad, so the next power-up stays on
 *      golden -- one power cycle at worst, never a loop.
 *   4. If policy says autoload and a valid stash exists: LOAD it over TSI,
 *      verify every byte against the stash, start hart 0. The program starts
 *      as it would from rb: a flight controller comes up disarmed and
 *      UNCOMMITTED -- a commit is never stored or replayed.
 *
 * Host commands (load_link.c dispatches them here):
 *   "RBSTASH1 <hex len>"  then len bytes: the blob esp_tsi_load.py builds
 *   "RBSTASH1 CLEAR"      forget the stash
 *   "RBPOLICY1 [boot=golden|update] [autoload=0|1]"   set and/or report
 *   "RBAUTO1"             run the power-on sequence now (from step 2)
 */

#include "load_link.h"
#include "load_link_internal.h"
#include "esp_safety.h"

#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/printk.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define PART_ID       FIXED_PARTITION_ID(slot1_partition)
#define POLICY_OFF    0x0000
#define STASH_OFF     0x1000
#define ERASE_UNIT    0x1000
#define UPDATE_ADDR   0x800000u
#define MAX_SEGS      16
#define HDR_LEN       512

#define TSI_CMD_READ  0u
#define TSI_CMD_WRITE 1u
#define BOOT_ADDR_REG 0x1000u
#define CLINT_MSIP0   0x2000000u
#define CHUNK         1024

struct policy {
	char magic[8];          /* "RBPOL001" */
	uint8_t boot_update;
	uint8_t autoload;
	uint8_t update_bad;
	uint8_t rsv;
	uint32_t golden_id;
	uint32_t update_id;
	uint32_t crc;
} __packed;

struct stash_seg {
	uint64_t paddr;
	uint32_t off;           /* from the start of the data, i.e. HDR_LEN */
	uint32_t len;
} __packed;

struct stash_hdr {
	char magic[8];          /* "RBSTASH1" */
	uint32_t version;       /* 1 */
	uint32_t nseg;
	uint64_t entry;
	uint32_t data_len;
	uint32_t data_crc;      /* crc32_ieee over the data */
	struct stash_seg seg[MAX_SEGS];
} __packed;

BUILD_ASSERT(sizeof(struct stash_hdr) <= HDR_LEN);

static struct policy pol;
static const struct flash_area *fa;

/* ---- output: to the host of the current session, and/or the console ---- */

struct out {
	int *sock;
	bool is_usb;
	bool to_host;
};

static void say(struct out *o, const char *fmt, ...)
{
	char buf[200];
	va_list ap;

	va_start(ap, fmt);
	vsnprintk(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	if (o != NULL && o->to_host) {
		(void)ll_send(o->sock, o->is_usb, buf);
	} else {
		printk("persist: %s", buf);
	}
}

/* ---- flash ---- */

static bool part_open(void)
{
	return fa != NULL || flash_area_open(PART_ID, &fa) == 0;
}

/* Every erase goes through here. esp_flash erases a whole sector or 64 KiB
 * block per op with interrupts and the cache off, which can outlast the motors
 * build's 250 ms watchdog and stalls the motor failsafe. So an erase runs ONLY
 * under a granted hold (esp_safety.c): no duty on any gate for 1 s, the dog
 * paused (or kept from starting), and the gates held at 0 until the release.
 * Otherwise it is refused with -EBUSY -- a session can last seconds after
 * load_link's motors-running check, and the FC may have spun up since. The
 * default build has no dog or gates and always erases. */
static int erase_guarded(off_t off, size_t len)
{
	int rc = -EBUSY;

	if (!esp_bridge_motors_recently_on() && esp_safety_wdt_hold(true)) {
		rc = flash_area_erase(fa, off, len);
	}
	/* Always: a refused request must not stay pending and be granted later. */
	(void)esp_safety_wdt_hold(false);
	return rc;
}

/* Page programs hold IRQs off for ~1 ms at a time, no hold needed, but none
 * while a gate is driven either (same 1 s rule). */
static int write_guarded(off_t off, const void *data, size_t len)
{
	if (esp_bridge_motors_recently_on()) {
		return -EBUSY;
	}
	return flash_area_write(fa, off, data, len);
}

static uint32_t pol_crc(const struct policy *p)
{
	return crc32_ieee((const uint8_t *)p, offsetof(struct policy, crc));
}

static void policy_load(void)
{
	memset(&pol, 0, sizeof(pol));
	if (part_open() && flash_area_read(fa, POLICY_OFF, &pol, sizeof(pol)) == 0 &&
	    memcmp(pol.magic, "RBPOL001", 8) == 0 && pol.crc == pol_crc(&pol)) {
		return;
	}
	memset(&pol, 0, sizeof(pol));   /* default: golden, no autoload */
	memcpy(pol.magic, "RBPOL001", 8);
}

static int policy_save(void)
{
	pol.crc = pol_crc(&pol);
	if (!part_open()) {
		return -ENODEV;
	}
	int rc = erase_guarded(POLICY_OFF, ERASE_UNIT);

	return rc ? rc : write_guarded(POLICY_OFF, &pol, sizeof(pol));
}

static bool stash_header(struct stash_hdr *h)
{
	if (!part_open() || flash_area_read(fa, STASH_OFF, h, sizeof(*h)) != 0) {
		return false;
	}
	if (memcmp(h->magic, "RBSTASH1", 8) != 0 || h->version != 1 ||
	    h->nseg == 0 || h->nseg > MAX_SEGS ||
	    h->data_len > fa->fa_size - STASH_OFF - HDR_LEN) {
		return false;
	}
	for (uint32_t i = 0; i < h->nseg; i++) {
		if ((uint64_t)h->seg[i].off + h->seg[i].len > h->data_len ||
		    (h->seg[i].paddr & 3) != 0) {
			return false;
		}
	}
	return true;
}

static bool stash_crc_ok(const struct stash_hdr *h)
{
	static uint8_t buf[CHUNK];
	uint32_t crc = 0;

	for (uint32_t off = 0; off < h->data_len; off += CHUNK) {
		uint32_t n = MIN(CHUNK, h->data_len - off);

		if (flash_area_read(fa, STASH_OFF + HDR_LEN + off, buf, n) != 0) {
			return false;
		}
		crc = crc32_ieee_update(crc, buf, n);
	}
	return crc == h->data_crc;
}

/* ---- TSI over the claimed wire ---- */

static void tsi_hdr(uint32_t cmd, uint64_t addr, uint32_t nbytes)
{
	uint8_t h[20];

	sys_put_le32(cmd, &h[0]);
	sys_put_le64(addr, &h[4]);
	sys_put_le64(((nbytes + 3u) / 4u) - 1u, &h[12]);
	ll_write(h, sizeof(h));
}

static void tsi_write(uint64_t addr, const uint8_t *d, uint32_t n)
{
	static const uint8_t pad[3] = { 0xff, 0xff, 0xff };

	tsi_hdr(TSI_CMD_WRITE, addr, n);
	ll_write(d, n);
	ll_write(pad, (4u - (n & 3u)) & 3u);
}

static bool tsi_read(uint64_t addr, uint8_t *d, uint32_t n)
{
	static uint8_t tmp[CHUNK + 4];
	uint32_t want = (n + 3u) & ~3u;

	tsi_hdr(TSI_CMD_READ, addr, n);
	/* 115200 8N1 = 11.5 B/ms; 4x margin and 200 ms for whatever is queued. */
	if (ll_read(tmp, want, 200 + (int)(want / 3u)) != want) {
		return false;
	}
	memcpy(d, tmp, n);
	return true;
}

/* LOAD the stash: write, verify against the flash copy, start. */
static bool load_stash(struct out *o, const struct stash_hdr *h)
{
	static uint8_t a[CHUNK], b[CHUNK];
	uint32_t id;
	bool idv, ok = false;
	int64_t t0 = k_uptime_get();

	ll_claim();
	if (!ll_handshake(K_LOAD, 0, &id, &idv)) {
		say(o, "autoload: no loader answered the LOAD key\n");
		ll_release(false);
		return false;
	}
	for (uint32_t s = 0; s < h->nseg; s++) {
		for (uint32_t off = 0; off < h->seg[s].len; off += CHUNK) {
			uint32_t n = MIN(CHUNK, h->seg[s].len - off);

			(void)flash_area_read(fa, STASH_OFF + HDR_LEN + h->seg[s].off + off, a, n);
			tsi_write(h->seg[s].paddr + off, a, n);
			if ((off / CHUNK) % 16 == 15 && !tsi_read(h->seg[s].paddr + off, b, 4)) {
				say(o, "autoload: TSI stopped answering while writing\n");
				goto out;
			}
		}
	}
	for (uint32_t s = 0; s < h->nseg; s++) {
		for (uint32_t off = 0; off < h->seg[s].len; off += CHUNK) {
			uint32_t n = MIN(CHUNK, h->seg[s].len - off);

			(void)flash_area_read(fa, STASH_OFF + HDR_LEN + h->seg[s].off + off, a, n);
			if (!tsi_read(h->seg[s].paddr + off, b, n) || memcmp(a, b, n) != 0) {
				say(o, "autoload: VERIFY FAILED at 0x%llx; hart 0 not started\n",
				    (unsigned long long)(h->seg[s].paddr + off));
				goto out;
			}
		}
	}
	{
		uint8_t e[8], one[4] = { 1, 0, 0, 0 };

		sys_put_le64(h->entry, e);
		tsi_write(BOOT_ADDR_REG, e, 8);
		tsi_write(CLINT_MSIP0, one, 4);
	}
	ok = true;
	say(o, "autoload: %u B verified and started at 0x%llx in %lld ms\n",
	    h->data_len, (unsigned long long)h->entry, k_uptime_get() - t0);
out:
	ll_release(true);
	return ok;
}

static bool ping(uint32_t *id, bool *idv, int tries_ms)
{
	int64_t end = k_uptime_get() + tries_ms;

	do {
		ll_claim();
		bool ok = ll_handshake(K_PING, 0, id, idv);

		ll_release(ok);
		if (ok) {
			return true;
		}
		k_msleep(700);
	} while (k_uptime_get() < end);
	return false;
}

/* Steps 2-4 of the power-on sequence. */
static void autoboot(struct out *o)
{
	uint32_t id = 0;
	bool idv = false;
	struct stash_hdr h;

	policy_load();
	if (esp_bridge_motors_recently_on()) {
		say(o, "motors are running: not touching the FPGA\n");
		return;
	}
	if (!ping(&id, &idv, 15000)) {
		say(o, "no ESP loader answered for 15 s: nothing to do (is the FPGA "
		    "configured with a loader bitstream?)\n");
		return;
	}
	say(o, "FPGA answers, image id %08x\n", idv ? id : 0);
	if (idv && pol.golden_id != id && !pol.boot_update) {
		pol.golden_id = id;
		(void)policy_save();
	}
	if (pol.boot_update && !pol.update_bad) {
		if (idv && pol.golden_id == 0) {
			pol.golden_id = id;
		}
		say(o, "policy: boot UPDATE at 0x%06x\n", UPDATE_ADDR);
		ll_claim();
		bool ok = ll_handshake(K_BOOT, UPDATE_ADDR, &id, &idv);

		k_msleep(500);
		ll_release(false);
		uint32_t nid = 0;
		bool nidv = false;

		if (!ok || !ping(&nid, &nidv, 20000)) {
			pol.update_bad = 1;
			say(o, "UPDATE image never answered: marked bad; the next power-up "
			    "stays on golden (power-cycle now to get golden back)\n");
			(void)policy_save();
			return;
		}
		if (nidv && nid == pol.golden_id) {
			pol.update_bad = 1;
			say(o, "UPDATE failed to configure (the FPGA fell back to golden "
			    "%08x): marked bad\n", nid);
			(void)policy_save();
		} else {
			say(o, "now running UPDATE, image id %08x\n", nidv ? nid : 0);
			if (nidv && pol.update_id != nid) {
				pol.update_id = nid;
				(void)policy_save();
			}
		}
	}
	if (pol.autoload) {
		if (!stash_header(&h) || !stash_crc_ok(&h)) {
			say(o, "autoload is on but there is no valid stashed program\n");
			return;
		}
		(void)load_stash(o, &h);
	}
}

/* ---- host sessions ---- */

static void report(struct out *o)
{
	struct stash_hdr h;
	char st[80];

	if (stash_header(&h)) {
		snprintf(st, sizeof(st), "%u B in %u seg, entry 0x%llx%s", h.data_len, h.nseg,
			 (unsigned long long)h.entry, stash_crc_ok(&h) ? "" : " (CRC BAD)");
	} else {
		strcpy(st, "none");
	}
	say(o, "RBPOLICY OK boot=%s autoload=%u update_bad=%u golden=%08x update=%08x "
	    "stash=%s\n", pol.boot_update ? "update" : "golden", pol.autoload,
	    pol.update_bad, pol.golden_id, pol.update_id, st);
}

static void stash_receive(struct out *o, const char *args)
{
	static uint8_t buf[CHUNK];
	struct stash_hdr h;

	if (!part_open()) {
		say(o, "RBSTASH ERR no image-1 partition\n");
		return;
	}
	if (strcmp(args, " CLEAR") == 0) {
		(void)erase_guarded(STASH_OFF, ERASE_UNIT);
		say(o, "RBSTASH OK cleared\n");
		return;
	}
	char *endp;
	unsigned long len = strtoul(args, &endp, 16);

	if (endp == args || *endp != '\0' || len < HDR_LEN ||
	    len > fa->fa_size - STASH_OFF) {
		say(o, "RBSTASH ERR bad length (max 0x%lx)\n",
		    (unsigned long)(fa->fa_size - STASH_OFF));
		return;
	}
	uint32_t span = ROUND_UP(len, ERASE_UNIT);

	int erc = erase_guarded(STASH_OFF, span);

	if (erc != 0) {
		say(o, erc == -EBUSY ? "RBSTASH ERR motors-running: erase refused\n" :
		    "RBSTASH ERR erase failed\n");
		return;
	}
	say(o, "RBSTASH READY\n");
	uint32_t got = 0;
	int64_t last = k_uptime_get();

	while (got < len) {
		uint32_t want = MIN(sizeof(buf), len - got);
		uint32_t have = 0;

		while (have < want) {
			int n = ll_recv(o->sock, o->is_usb, &buf[have], want - have);

			if (n < 0 || k_uptime_get() - last > 5000) {
				(void)erase_guarded(STASH_OFF, ERASE_UNIT);
				say(o, "RBSTASH ERR host stopped at %u/%lu B\n", got + have, len);
				return;
			}
			if (n == 0) {
				k_msleep(1);
				continue;
			}
			have += (uint32_t)n;
			last = k_uptime_get();
		}
		uint32_t wlen = ROUND_UP(have, 4);

		memset(&buf[have], 0xff, wlen - have);
		int wrc = write_guarded(STASH_OFF + got, buf, wlen);

		if (wrc != 0) {
			/* Refused (motors started mid-upload), the clearing erase
			 * is refused too; the partial blob then fails stash_crc_ok()
			 * and is never loaded. */
			(void)erase_guarded(STASH_OFF, ERASE_UNIT);
			say(o, wrc == -EBUSY ? "RBSTASH ERR motors-running: write refused\n" :
			    "RBSTASH ERR flash write failed\n");
			return;
		}
		got += have;
	}
	if (!stash_header(&h) || HDR_LEN + h.data_len != len || !stash_crc_ok(&h)) {
		(void)erase_guarded(STASH_OFF, ERASE_UNIT);
		say(o, "RBSTASH ERR the blob failed its header/CRC check; stash cleared\n");
		return;
	}
	say(o, "RBSTASH OK %u B in %u segment(s), entry 0x%llx\n", h.data_len, h.nseg,
	    (unsigned long long)h.entry);
}

void persist_session(int *sock, bool is_usb, const struct request *rq)
{
	struct out o = { .sock = sock, .is_usb = is_usb, .to_host = true };

	policy_load();
	switch (rq->kind) {
	case K_STASH:
		stash_receive(&o, rq->args);
		break;
	case K_POLICY: {
		char a[48];
		bool changed = false;

		strncpy(a, rq->args, sizeof(a) - 1);
		a[sizeof(a) - 1] = '\0';
		for (char *tok = strtok(a, " "); tok != NULL; tok = strtok(NULL, " ")) {
			if (strcmp(tok, "boot=golden") == 0) {
				pol.boot_update = 0; changed = true;
			} else if (strcmp(tok, "boot=update") == 0) {
				pol.boot_update = 1; pol.update_bad = 0; changed = true;
			} else if (strcmp(tok, "autoload=0") == 0) {
				pol.autoload = 0; changed = true;
			} else if (strcmp(tok, "autoload=1") == 0) {
				pol.autoload = 1; changed = true;
			} else {
				say(&o, "RBPOLICY ERR unknown setting '%s'\n", tok);
				return;
			}
		}
		if (changed) {
			int prc = policy_save();

			if (prc != 0) {
				say(&o, prc == -EBUSY ? "RBPOLICY ERR motors-running: flash "
				    "write refused\n" : "RBPOLICY ERR flash write failed\n");
				return;
			}
		}
		report(&o);
		break;
	}
	case K_AUTO:
		say(&o, "RBAUTO OK running the power-on sequence\n");
		autoboot(&o);
		say(&o, "RBAUTO DONE\n");
		break;
	default:
		break;
	}
}

/* Called once from the load thread at startup, before it serves sessions. */
void persist_start(void)
{
	uint32_t cause = 0;
	uint32_t raw = esp_safety_reset_raw();

	(void)hwinfo_get_reset_cause(&cause);
	(void)hwinfo_clear_reset_cause();
	policy_load();
	/*
	 * A brownout is a power event too: the FPGA on the same rail may have
	 * reconfigured to golden with an empty SoC, exactly as at power-on. On the
	 * C6 the chip-level brownout already reads as RESET_POR (ROM 0x01 is both);
	 * RESET_BROWNOUT is the digital-core brownout (ROM 0x0F). The 2 s uart1
	 * guard below still leaves a running FPGA alone.
	 *
	 * The raw ROM reason is logged as well because hwinfo maps the USB-serial
	 * (0x15) and JTAG (0x16/0x18) resets to 0, and RBLINK carries it as rst=.
	 */
	bool power_event = (cause & (RESET_POR | RESET_BROWNOUT)) != 0u;

	printk("persist: reset cause 0x%x%s, rom rst=0x%02x (%s); policy boot=%s "
	       "autoload=%u update_bad=%u\n",
	       cause, (cause & RESET_BROWNOUT) ? " (brownout)" :
		      (cause & RESET_POR) ? " (power-on or chip brownout)" : "",
	       (unsigned)raw, esp_safety_reset_name(raw),
	       pol.boot_update ? "update" : "golden", pol.autoload, pol.update_bad);
	if (!power_event) {
		printk("persist: not a power-on reset: the FPGA may be mid-flight; "
		       "leaving it alone\n");
		return;
	}
	if (!pol.boot_update && !pol.autoload) {
		return;
	}
	/* The FPGA configures from flash in well under a second, and a program
	 * that is already running talks on uart1 at once (telemetry, motor
	 * frames). Watch for 2 s before touching anything. */
	k_msleep(2000);
	if (esp_bridge_uart1_rx_bytes() > 16) {
		printk("persist: uart1 is already busy: a program is running; leaving it alone\n");
		return;
	}
	autoboot(NULL);
}
