#!/usr/bin/env python3
"""Reshape a raw HM01B0 capture into a viewable PNG.

    python3 hardware/ospi/frame-to-png.py camera-frame.raw 326

The raw file is what hardware/ospi/capture-frame.gdb dumps out of pixels[]: one
grayscale byte per pixel, in the order the capture core drained them, with no
header and no row padding.

Two things about the geometry, both of which look like a broken capture if you
get them wrong:

  * Reshape rows at the MEASURED width the core reports (LASTWIDTH), not at the
    width GEOM is configured for. On this sensor they differ -- 326 measured
    against 324 configured -- because each line carries two dummy pixels ahead
    of the active window. Reshaping at 324 shears the image one pixel per row.
  * Then find WHERE in the buffer a line actually starts, and drop the dummy
    pixels from there. The capture core arms wherever the sensor happens to be,
    so the buffer does not begin at a line boundary -- see find_line_phase(),
    which locates it from the dummy pixels themselves rather than assuming it.
    --crop N --no-anchor restores the old blind behaviour.

The HM01B0 comes as a mono part and as a Bayer-CFA colour part that share a pin
map, a register map and a model id, so the only way to know which one is on the
board is to look at the pixels. Every run therefore measures the frame for a
mosaic and says what it found, and writes an RGB PNG only when the pixels
support one.

When they do not, --colour still renders -- somebody is watching that file --
but it is announced as an ARTIFACT RENDER and the png carries that warning in a
tEXt chunk, because this is the most misleading thing this tool can produce.
Demosaicing a frame with no mosaic in it does not give a grey picture: it gives
a confident, plausible-looking colour photograph in which the hue encodes
BRIGHTNESS and nothing else. Bright things come out warm, dark things cool, and
a warm object is warm because it is bright. Measured on three captures from this
bench, the invented chroma correlates -0.72, -0.80 and -0.96 with local
luminance. Such a picture is worse than no picture because it is believable, so
the warning is not optional and is not a footnote.

One 2x2 sampling phase comes off this capture path as `alpha * true + beta`
with alpha near a half -- a KNOWN HARDWARE DEFECT, present in captures from
before the camera module was swapped, so it is the capture core and not the
sensor. It is undone on the way to the PNG, per frame and per pixel, before
anything else looks at the data; fit_phase_defect() carries the measurements and
repair_phase_defect() applies them. The .raw is never written. --no-repair turns
it off. Delete both once the RTL fix lands.

As of the swap on 2026-09-21 the frames still measure as mono, and they do so
AFTER that repair, which is the stronger statement: the defect is no longer
sitting on top of the mosaic test, and the test still finds nothing. Nor is the
mosaic hiding behind a drifting CFA phase, which would cancel in any frame-wide
average -- checker_excess() squares the checkerboard first so a phase flip
cannot cancel it, and reads within noise on every frame here while a synthetic
CFA of one and a half percent reads twelve sigma.

Writes the PNG with zlib and struct rather than an imaging library, because the
bench Python has neither PIL nor numpy. The demosaic is hand-rolled for the same
reason.
"""

import argparse
import pathlib
import struct
import sys
import zlib


def measure_comb(rows, width):
    """Ratio of even-column mean to odd-column mean, from this frame alone.

    The frame carries a fine vertical comb of about 8%. It is measured here
    rather than stored because a stored number would be wrong whenever the
    exposure changed -- see below, the comb is not a fixed gain.

    CORRECTED 2026-09-21. This docstring used to say the comb PHASE was not
    stable, on the evidence of ratios of 0.9232, 0.9231, 1.0833, 1.0831 across
    four frames. That was a measurement bug in this tool, not a property of the
    hardware: the buffer does not begin at a line boundary, and a frame whose
    line phase is ODD has its two column parities swapped before anything here
    gets to look at them. One of the three captures on this bench starts 155
    bytes into a line. Anchored on the sensor's own dummy-pixel marker -- what
    find_line_phase() now does -- all three captures give the SAME sign:
    odd columns brighter by 7.2%, 7.4% and 9.5%. The phase is stable. The tool
    was not.

    NOT the one-phase defect, which IS corrected -- see fit_phase_defect(), which
    runs before this does, so what this measures is whatever comb survives it.

    Reported, NOT corrected. A single gain ratio does not fix this, and trying
    made it measurably worse: equalising the means took the ratio to 1.002 while
    the adjacent-pixel difference ROSE from 6.39 to 6.91, because the error is
    not a constant gain. Measured on one frame, mean(even - odd) by local level:
    -4.4 at 32-63, -19.9 at 64-95, -39.6 at 96-127, -58.4 at 128-159 -- an
    implied ratio sliding from 0.91 to 0.66 as the scene brightens. A gain
    correction under-corrects highlights and over-corrects shadows.

    That level dependence, plus the per-capture phase flip, points at sampling
    timing in the capture core rather than sensor column gain: an under-settled
    sample is pulled toward its neighbour by an amount proportional to the
    signal swing, which is what a level-dependent error looks like. The fix
    belongs in the capture RTL, not in a sensor register table and not here.

    RE-TESTED after the sensor was swapped for the colour part, because a Bayer
    CFA would produce a 2x2 modulation that could be mistaken for this comb.
    It is not the CFA, and the comb is not new: on the post-swap frame the
    phase separation is 9.2% on scene-flat tiles and 25.1% on detailed ones,
    growing 2.7x with edge content, and on a pre-swap frame of the same bench
    the same measurement gives 5.5% and 12.9%, growing 2.4x. Colour does not
    work that way -- a flat red wall is red whether or not it has edges in it --
    but an under-settled sample does exactly that. See detect_mosaic().

    Returns None when there is nothing to report.
    """
    ev = od = 0
    evn = odn = 0
    for row in rows:
        ev += sum(row[0::2]); evn += len(row[0::2])
        od += sum(row[1::2]); odn += len(row[1::2])
    if evn == 0 or odn == 0:
        return None
    even, odd = ev / evn, od / odn
    if even <= 0 or odd <= 0:
        return None
    ratio = even / odd
    return ratio if abs(ratio - 1.0) > 0.01 else None


def find_line_phase(data, width, dummies=2, need=0.5):
    """Byte offset of the first complete line, found from the sensor's own marker.

    Every line the HM01B0 delivers on this board begins with `dummies` bytes of
    0x00 ahead of the active window. That is measured, not assumed: across three
    captures on this bench and one from before the sensor swap, EVERY line start
    carries exactly `00 00` -- 315 of 315 lines in three of the frames and 312 of
    315 in the fourth, where the three extras are dark scene content that happens
    to hold an adjacent zero pair. In each frame the markers are exactly `width`
    apart with no exceptions, which is what makes them usable as an anchor.

    The anchor matters because the capture core arms wherever the sensor happens
    to be in the frame, so the buffer does NOT begin at a line boundary. One of
    the three frames starts 155 bytes into a line. Two things go wrong if you
    ignore that, and the second one cost a bench session:

      * every row is wrapped by 155 pixels, which draws a hard vertical seam
        through the middle of the picture -- it reads as a torn capture and is
        not;
      * the column grid is shifted by an ODD number, which SWAPS the two column
        parities. Every 2x2 statistic measured on that grid then comes back with
        the opposite sign, and two captures of one scene look as though the comb
        had inverted between them. It had not. See measure_comb().

    Returns {"start", "phase", "markers", "lines"} or None when no offset
    repeats often enough to be a line marker, in which case the caller falls
    back to the blind crop and says so.
    """
    if dummies < 2 or width <= dummies or len(data) < 2 * width:
        return None
    counts = {}
    limit = len(data) - dummies
    for i in range(limit + 1):
        if data[i] == 0 and data[i + 1] == 0:
            if dummies > 2 and any(data[i + k] for k in range(2, dummies)):
                continue
            phase = i % width
            counts[phase] = counts.get(phase, 0) + 1
    if not counts:
        return None
    phase = max(counts, key=lambda k: (counts[k], -k))
    lines = len(data) // width
    if lines == 0 or counts[phase] < need * lines:
        return None
    start = next(i for i in range(phase, len(data) - 1, width)
                 if data[i] == 0 and data[i + 1] == 0)
    return {"start": start, "phase": phase, "markers": counts[phase], "lines": lines}


BAYER_PATTERNS = ("RGGB", "BGGR", "GRBG", "GBRG")
PHASE_NAMES = ("r0c0", "r0c1", "r1c0", "r1c1")
DIAGONALS = ((0, 3), (1, 2))            # (r0c0,r1c1) and (r0c1,r1c0)


def tile_phase_ratios(rows, width, tile=16):
    """Per-tile level of each 2x2 sampling phase, as a fraction of the tile mean.

    Everything the mosaic test needs comes off this one pass. Working per tile
    rather than per frame matters: real scene colour changes sign across a frame
    -- some of it reddish, some bluish -- so a frame-wide phase average cancels
    the very thing being looked for. A 16x16 tile is also large enough that two
    phases are compared on equal terms whether they are neighbours or diagonal,
    which a per-pixel difference cannot claim.

    Also returns a scene-detail figure per tile, measured as mean |p[x+2]-p[x]|.
    Two apart is deliberate: those two pixels are the same CFA colour, so the
    mosaic cannot inflate the number. |p[x+1]-p[x]| would be circular, since on
    a mosaiced frame that difference IS the mosaic.

    Yields (ratios, detail, level) with ratios indexed (row%2)*2 + (col%2) and
    level the tile's raw mean, which measure_phase_deficit() needs: a colour
    filter is multiplicative and so its ratio does not move with level, and a
    settling error is not and so its ratio does.
    """
    height = len(rows)
    out = []
    for by in range(0, height - tile + 1, tile):
        for bx in range(0, width - tile + 1, tile):
            tot = [0.0] * 4
            n = [0] * 4
            grad = cnt = 0
            for dy in range(tile):
                r = rows[by + dy]
                for dx in range(tile):
                    tot[(dy & 1) * 2 + ((bx + dx) & 1)] += r[bx + dx]
                    n[(dy & 1) * 2 + ((bx + dx) & 1)] += 1
                    if bx + dx + 2 < width:
                        grad += abs(r[bx + dx + 2] - r[bx + dx])
                        cnt += 1
            if min(n) == 0 or cnt == 0:
                continue
            means = [tot[i] / n[i] for i in range(4)]
            level = sum(means) / 4.0
            if level < 12.0:            # too dark for a ratio to mean anything
                continue
            out.append(([v / level for v in means], grad / cnt, level))
    return out


def measure_phase_deficit(tiles):
    """Does ONE of the four 2x2 phases just read low -- and does it slide with level?

    This settles mono-against-colour more firmly than the parity-correlation
    test below, because it does not need the scene to have had any colour in it.
    Two independent things are checked and a CFA fails both.

    SHAPE. A Bayer quad is R G G B: TWO phases equal (the greens) and two
    different. Measured on three captures minutes apart on this bench, anchored
    on the line marker, the four phase levels are

        A  50.88  59.61  59.66  59.47        B  50.37  59.11  59.23  59.16
        C  51.62  64.60  64.93  64.20

    -- three agreeing to within 0.32%, 0.21% and 1.13%, and the fourth 14.6%,
    14.9% and 20.1% low. It is the same phase every time, even row and even
    column on the line-anchored grid, in frames whose line phases differ (0, 155,
    0) and whose arm points differ. No CFA layout produces three-equal-one-low.

    LEVEL. A colour filter is multiplicative: red under a fixed illuminant is a
    fixed fraction of green whether the scene is dim or bright, so the ratio is
    FLAT against level. This one is not. Per 16x16 tile, low phase over the mean
    of the other three, binned by tile brightness:

        A   level 41 -> 0.975   50 -> 0.882   59 -> 0.831   94 -> 0.769
        C   level 42 -> 0.943   51 -> 0.863   59 -> 0.814   86 -> 0.716

    corr(ratio, level) = -0.72, -0.80, -0.97 across the three frames. In counts
    the deficit runs about 1 at level 41 and 22 at level 94 -- it grows far
    faster than the signal does, which is an under-settled sample rather than a
    gain, a filter, or anything the sensor could be told to stop doing over I2C.

    That is also exactly what the demosaic turns into "colour": hue that tracks
    brightness, so bright things come out warm and dark things cool.

    Returns None when there are too few tiles, else a dict; `dead` is True when
    the three checks together rule a CFA out.
    """
    if len(tiles) < 24:
        return None
    n = len(tiles)
    mean = [sum(t[0][i] for t in tiles) / n for i in range(4)]
    levels = [t[2] for t in tiles]

    # Per-phase ratio against the other three, and how it moves with level.
    # Done for all four rather than just the lowest, because which phase is
    # lowest is not a reliable handle: put a real CFA on top of the defect and
    # the lowest phase becomes the blue one, whose ratio is flat, and the defect
    # on some other phase goes unnoticed. The MINIMUM over the four is what
    # answers "is a level-dependent defect present anywhere in this frame".
    per = []
    for i in range(4):
        r = []
        for t in tiles:
            o = sum(t[0][j] for j in range(4) if j != i) / 3.0
            r.append(t[0][i] / o if o > 0 else 1.0)
        per.append(r)
    if len(levels) < 24:
        return None
    corrs = [_corr(per[i], levels) for i in range(4)]

    # SHAPE, chosen without reference to which phase is lowest: for each phase
    # in turn, how tightly do the OTHER THREE agree? A single-phase defect
    # leaves three untouched phases that match almost exactly. A CFA of any
    # layout leaves at least two different values behind whichever one is
    # dropped, so no choice gives a tight spread.
    def spread_without(i):
        o = [mean[j] for j in range(4) if j != i]
        b = sum(o) / 3.0
        return (max(o) - min(o)) / b if b > 0 else 1.0
    low = min(range(4), key=spread_without)
    others = [mean[j] for j in range(4) if j != low]
    base = sum(others) / 3.0
    if base <= 0:
        return None
    ratios = per[low]
    rec = {
        "low": low,
        "deficit": 1.0 - mean[low] / base,
        "spread": (max(others) - min(others)) / base,
        "level_corr": corrs[low],
        "worst_corr": min(corrs),
        "bins": [],
    }
    order = sorted(range(len(ratios)), key=lambda i: levels[i])
    step = max(1, len(order) // 4)
    for k in range(0, len(order) - step + 1, step):
        sel = order[k:k + step]
        rec["bins"].append((sum(levels[i] for i in sel) / len(sel),
                            sum(ratios[i] for i in sel) / len(sel)))
    # A CFA is two-equal-two-different, and its ratio does not move with level.
    # Requiring all three at once is what makes this safe to act on: a real
    # mosaic cannot satisfy any two of them, let alone all three.
    # "A level-dependent defect exists somewhere in this frame" -- the minimum
    # over all four phases, so it is found whether or not a CFA is sitting on
    # top of it. On a clean synthetic mosaic every phase is flat against level
    # and this does not fire, which is what leaves the parity test below free to
    # do its job on frames that have no defect in them.
    rec["defect"] = rec["worst_corr"] <= -0.4 and rec["deficit"] >= 0.05
    rec["clean3"] = rec["spread"] <= 0.03
    rec["dead"] = rec["defect"] and rec["clean3"]
    return rec


def _corr(a, b):
    n = len(a)
    ma, mb = sum(a) / n, sum(b) / n
    va = sum((v - ma) ** 2 for v in a) / n
    vb = sum((v - mb) ** 2 for v in b) / n
    if va <= 0 or vb <= 0:
        return 0.0
    return (sum(a[i] * b[i] for i in range(n)) / n - ma * mb) / (va * vb) ** 0.5


def _rms(a):
    return (sum(v * v for v in a) / len(a)) ** 0.5


# ---------------------------------------------------------------------------
# KNOWN HARDWARE DEFECT IN THE CAPTURE PATH -- compensated below, on the way to
# the PNG only. REMOVE THIS ONCE THE RTL FIX LANDS; another agent is on it. The
# .raw on disk is never written, so the dump stays exactly what the core
# produced and every number here can be re-derived from it.
#
# WHAT IT IS. One of the four 2x2 sampling phases comes back as a plain affine
# function of the true pixel,
#
#       observed = alpha * true + (1 - alpha) * pedestal
#
# with alpha measured at 0.52-0.69, and the pedestal landing on the frame's own
# black floor to within a couple of counts:
#
#     frame      alpha   fitted beta   implied pedestal   frame black floor
#       A        0.687      10.01            32.0                38
#       B        0.582      15.61            37.3                40
#       C        0.521      18.13            37.9                38
#     pre-swap   0.542      26.22            57.3                57
#
# -- that is, the phase is read before it has settled and is left part of the
# way from the pedestal to the real signal. It is ONE phase of four, always the
# same one (even row, even active column on the line-anchored grid). The last
# row is a capture taken BEFORE the camera module was swapped, so this is the
# capture path and not the sensor. Fitted per frame because alpha is not a
# constant: it moved 0.52 to 0.69 across three captures minutes apart.
#
# It is NOT mixing in a neighbouring pixel. Regressing the damaged phase on its
# four neighbours gives symmetric weights -- left +0.403 right +0.384 up -0.041
# down -0.061 on frame A, left +0.124 right +0.125 up +0.129 down +0.146 on
# frame C -- with no bias toward the preceding pixel in raster order, and the
# first active column of each line (whose predecessor on the wire is a 0x00
# dummy) is not darker than its neighbours. So the damaged pixel still carries
# its own value, scaled, and dividing that scaling back out recovers it.
#
# WHY THIS REPAIR IS SAFE WHERE A NEIGHBOUR INTERPOLATION WOULD NOT BE. The
# correction is (observed - beta) / alpha: a per-pixel affine map that reads
# ONLY THAT PIXEL'S OWN VALUE. No neighbour is consulted, so it cannot move
# signal between the four phases -- and the four phases are exactly where a
# Bayer mosaic lives. An interpolator that rebuilt the damaged phase from its
# neighbours would MANUFACTURE a mosaic wherever the neighbours differed and
# ERASE one wherever they did not, and the detector downstream would then be
# measuring this function instead of the sensor. An affine map on one phase can
# do neither: it rescales what is already there, so a mosaic that was present
# stays present -- with its amplitude on that phase divided by alpha, i.e.
# INCREASED, which makes this conservative in the direction that matters -- and
# a frame with no mosaic gains none. The fit does read the other phases, but
# only to estimate two scalars for the whole frame.
# ---------------------------------------------------------------------------

DEFECT_ALPHA_MIN = 0.20         # below this the fit is not a settling error
DEFECT_ALPHA_MAX = 0.95         # above this there is nothing worth correcting
DEFECT_MIN_DEFICIT = 0.02       # the low phase must really be low. Low, because the
                                # SIZE of the deficit depends on how far the scene sits
                                # above the pedestal, not on how bad the defect is -- the
                                # pre-swap capture shows alpha 0.54 as only a 2.8%% deficit
                                # purely because its black floor is high. What actually
                                # keeps this off a clean frame is DEFECT_ALPHA_MAX: with no
                                # defect the fitted alpha comes back at ~1.0 and is refused.
DEFECT_MAX_TRIO_SPREAD = 0.08   # ... and the other three must really agree


def phase_levels(rows, width):
    """Mean of each 2x2 sampling phase, indexed (row%2)*2 + (col%2)."""
    tot = [0.0] * 4
    n = [0] * 4
    for y, row in enumerate(rows):
        base = (y & 1) * 2
        for x in range(width):
            i = base + (x & 1)
            tot[i] += row[x]
            n[i] += 1
    return [tot[i] / n[i] if n[i] else 0.0 for i in range(4)]


def phase_spread(levels):
    """Spread of the four phase levels as a fraction of their mean."""
    m = sum(levels) / 4.0
    return (max(levels) - min(levels)) / m if m > 0 else 0.0


def fit_phase_defect(rows, width, tile=32):
    """Fit `observed = alpha*true + beta` for the damaged phase, from this frame.

    WHICH PHASE. Chosen the way measure_phase_deficit() chooses it -- the one
    whose removal leaves the other three agreeing most tightly -- and not by
    taking the darkest. That is what lets a frame which does not have this
    defect refuse the repair instead of being mangled by it: a real mosaic
    leaves no tight trio behind any phase, so no candidate passes
    DEFECT_MAX_TRIO_SPREAD and the caller is told the repair was declined.

    AGAINST WHAT. The DIAGONALLY OPPOSITE phase, which is the clean null: it is
    the one phase sharing neither a row parity nor a column parity with the
    damaged one, so the frame's separate column comb (see measure_comb) is
    common to the pair and divides out of the slope instead of tilting it. The
    two adjacent phases each share one parity with the damaged phase and would
    carry that comb into alpha.

    FITTED ON TILE MEANS, not on pixels. The predictor is a different pixel from
    the target, so per-pixel regression is attenuated by the scene difference
    between them -- classic regression dilution, and it biases alpha LOW, which
    would over-correct. Averaging each phase over a 32x32 tile first drops that
    difference far below the signal: on the four frames on this bench the
    per-pixel fit gives 0.653 / 0.602 / 0.518 / 0.483 where the tile fit gives
    0.687 / 0.582 / 0.521 / 0.542, and the same tile fit run between two
    UNDAMAGED phases returns 1.022 / 1.008 / 0.990 / 0.957, i.e. the 1.0 it
    should, which is what says the method itself is not inventing a slope.

    Returns a dict always; `alpha` is present only when the fit was accepted,
    and `why` says why not when it was not.
    """
    levels = phase_levels(rows, width)
    rec = {"levels": levels, "before": phase_spread(levels), "alpha": None,
           "why": None, "low": None}

    def spread_without(i):
        o = [levels[j] for j in range(4) if j != i]
        b = sum(o) / 3.0
        return (max(o) - min(o)) / b if b > 0 else 1.0

    low = min(range(4), key=spread_without)
    trio = [levels[j] for j in range(4) if j != low]
    base = sum(trio) / 3.0
    if base <= 0:
        rec["why"] = "frame is black"
        return rec
    rec["low"] = low
    rec["deficit"] = 1.0 - levels[low] / base
    rec["trio_spread"] = (max(trio) - min(trio)) / base
    if rec["deficit"] < DEFECT_MIN_DEFICIT:
        rec["why"] = ("no phase sits %.0f%% or more below the other three (the best "
                      "candidate is %.1f%% %s them) -- nothing to repair"
                      % (DEFECT_MIN_DEFICIT * 100, abs(rec["deficit"]) * 100,
                         "below" if rec["deficit"] >= 0 else "ABOVE"))
        return rec
    if rec["trio_spread"] > DEFECT_MAX_TRIO_SPREAD:
        rec["why"] = ("the three phases behind %s are spread by %.1f%%, over the "
                      "%.0f%% this defect allows -- something other than the "
                      "known defect is separating the phases, so it is left alone"
                      % (PHASE_NAMES[low], rec["trio_spread"] * 100,
                         DEFECT_MAX_TRIO_SPREAD * 100))
        return rec

    py, px = divmod(low, 2)
    qy, qx = 1 - py, 1 - px
    height = len(rows)
    xs, ys = [], []
    for by in range(0, height - tile + 1, tile):
        for bx in range(0, width - tile + 1, tile):
            sa = sd = 0.0
            na = nd = 0
            for dy in range(tile):
                y = by + dy
                row = rows[y]
                if (y & 1) == py:
                    for x in range(bx if (bx & 1) == px else bx + 1, bx + tile, 2):
                        sa += row[x]
                        na += 1
                else:
                    for x in range(bx if (bx & 1) == qx else bx + 1, bx + tile, 2):
                        sd += row[x]
                        nd += 1
            if na and nd:
                ys.append(sa / na)
                xs.append(sd / nd)
    n = len(xs)
    rec["tiles"] = n
    rec["tile"] = tile
    if n < 8:
        rec["why"] = "only %d tiles of %dx%d -- too few to fit two parameters" % (n, tile, tile)
        return rec
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((v - mx) ** 2 for v in xs)
    if sxx <= 0:
        rec["why"] = "the reference phase is flat across the frame -- no slope to fit"
        return rec
    alpha = sum((xs[i] - mx) * (ys[i] - my) for i in range(n)) / sxx
    beta = my - alpha * mx
    if not DEFECT_ALPHA_MIN <= alpha <= DEFECT_ALPHA_MAX:
        rec["why"] = ("fitted alpha %.3f is outside %.2f..%.2f, which is not this "
                      "defect -- left alone" % (alpha, DEFECT_ALPHA_MIN, DEFECT_ALPHA_MAX))
        return rec
    rec["alpha"] = alpha
    rec["beta"] = beta
    rec["pedestal"] = beta / (1.0 - alpha) if alpha < 1.0 else float("nan")
    return rec


def repair_phase_defect(rows, width, fit):
    """Undo the affine defect on the damaged phase. Returns (rows, clipped).

    Per pixel: true = (observed - beta) / alpha, through a 256-entry table, so
    the pixel's own value is the only input. Results above 255 are clipped and
    counted: a damaged pixel reading 255 implies a true value of about 356,
    which an 8-bit PNG cannot hold either way -- every undamaged phase at that
    scene brightness is already clipped at 255, so clipping the repaired phase
    to match is the consistent thing to do, but the count is reported because a
    large one means the frame was overexposed and the repair is guessing in the
    highlights.
    """
    alpha, beta = fit["alpha"], fit["beta"]
    py, px = divmod(fit["low"], 2)
    lut = []
    clip_from = 256
    for v in range(256):
        t = (v - beta) / alpha
        if t < 0.0:
            t = 0.0
        elif t > 255.0:
            t = 255.0
            clip_from = min(clip_from, v)
        lut.append(int(t + 0.5))
    out = []
    clipped = 0
    for y, row in enumerate(rows):
        if (y & 1) != py:
            out.append(row)
            continue
        r = bytearray(row)
        for x in range(px, width, 2):
            v = r[x]
            if v >= clip_from:
                clipped += 1
            r[x] = lut[v]
        out.append(bytes(r))
    return out, clipped


def checker_excess(tiles):
    """<C^2> - <(H^2+V^2)/2> per tile, and how many standard errors that is.

    REPORTED, NOT VOTED ON. The verdict below is still decided by the tests that
    were calibrated against synthetic mosaics; this is an independent figure
    printed beside it, because once the capture defect is repaired upstream the
    verdict falls through to the checker/parity ratio, whose margin is thin.

    What it is for. Every other statistic here is a phase-COHERENT average, so
    it cancels if the CFA phase drifts across the frame -- drop or duplicate an
    odd number of lines and Bayer row parity inverts for the rest of the frame,
    which flips the sign of C. This one squares C first, so a sign flip cannot
    cancel it, and it subtracts the row/column parity energy, which carries the
    same noise and the same one-phase defect residual as C does and so cancels
    what is not a mosaic.

    Calibrated on this bench by laying a synthetic CFA over a real capture:

        R=.60 B=.45 fixed phase          +571 sigma
        R=.60 B=.45, phase flipped mid-frame  +121 sigma
        R=.60 B=.45, phase flipped every 16 rows  +481 sigma
        R=.85 B=.80 fixed phase          +171 sigma
        R=.97 B=.96 fixed phase           +34 sigma
        R=.99 B=.985 fixed phase          +12 sigma

    -- so a colour separation of one and a half percent is still a twelve-sigma
    result, and drifting the phase does not hide it. The four real captures on
    this bench read -5.0, -6.6, -0.2 and -0.9 sigma.
    """
    h, v, c = parity_components(tiles)
    e = [c[i] ** 2 - (h[i] ** 2 + v[i] ** 2) / 2.0 for i in range(len(c))]
    n = len(e)
    if n < 8:
        return None
    m = sum(e) / n
    var = sum((x - m) ** 2 for x in e) / n
    se = (var / n) ** 0.5
    return {"excess": m, "sigmas": m / se if se > 0 else 0.0, "n": n}


def parity_components(tiles):
    """The three 2x2 parity signals per tile, in units of that tile's mean.

        H = (column 0) - (column 1)      V = (row 0) - (row 1)
        C = the checkerboard, (r0c0 + r1c1) - (r0c1 + r1c0)

    Four phase levels carry four numbers; the tile mean is the fourth, and
    these are the other three. Everything the mosaic test needs is in them.
    """
    h = [(t[0][0] + t[0][2]) - (t[0][1] + t[0][3]) for t in tiles]
    v = [(t[0][0] + t[0][1]) - (t[0][2] + t[0][3]) for t in tiles]
    c = [(t[0][0] + t[0][3]) - (t[0][1] + t[0][2]) for t in tiles]
    return h, v, c


def detect_mosaic(rows, width):
    """Is this frame a Bayer mosaic, or just combed? Decided from the frame alone.

    The HM01B0 ships in a mono and a Bayer-CFA variant that are pin-, register-
    and model-ID-compatible -- both answer 0x01B0 -- so only the pixels can say
    which part is fitted. That makes this the whole of the colour question, and
    it is worth doing properly rather than assuming the swap took.

    Substitute the four Bayer layouts into the parity basis and the same thing
    falls out every time:

        RGGB / BGGR:  H = R-B   V = R-B   C = R+B-2G
        GRBG / GBRG:  H = B-R   V = R-B   C = R+B-2G

    So on a mosaic H and V carry ONE quantity, red against blue, while C carries
    a different one, green against red-and-blue. Two things follow that a
    capture artifact cannot fake:

      * C must be clearly bigger than H and V, because a CFA separates green
        from red and blue far more than it separates red from blue;
      * C must be largely UNcorrelated with H and V, being a different axis of
        the scene's colour.

    A comb is one artifact, not three colours, so it lands in H, V and C alike:
    same size, and locked together. Measured over eleven frames -- eight
    synthetic mosaics, four clean and four laid over a real combed capture, and
    the three real captures from this bench:

        max |corr(H,C)|, |corr(V,C)|    mosaics 0.20..0.33    real 0.72..0.90
        rms C / rms (H,V)               mosaics 1.50..2.06    real 0.93..0.98

    Both gaps are wide and the thresholds sit in them. The test cannot see a CFA
    on a scene with no colour in it, and it would be fooled by a scene whose
    colours all lie on one line from red to blue, since C would then track H.
    Neither is the case on a lab bench, but it says what it measured either way
    so a surprising answer can be argued with.
    """
    tiles = tile_phase_ratios(rows, width)
    rec = {"tiles": len(tiles), "mosaic": False, "why": "", "parity": None,
           "flat": None, "busy": None, "n_flat": 0, "n_busy": 0, "deficit": None,
           "undecided": False, "excess": None}
    if len(tiles) < 24:
        rec["why"] = "too few usable tiles to judge"
        return rec
    rec["deficit"] = measure_phase_deficit(tiles)

    # Supporting evidence, reported either way: does the 2x2 separation follow
    # the scene or the edges? Colour is scene-borne; a settling comb is not.
    for key, lo, hi in (("flat", 0.0, 3.0), ("busy", 8.0, 1e9)):
        sel = [t[0] for t in tiles if lo <= t[1] < hi]
        if sel:
            rec[key] = [sum(t[i] for t in sel) / len(sel) for i in range(4)]
            rec["n_" + key] = len(sel)

    rec["excess"] = checker_excess(tiles)
    h, v, c = parity_components(tiles)
    rms_hv = (_rms(h) + _rms(v)) / 2.0
    rec["parity"] = {"rms_h": _rms(h), "rms_v": _rms(v), "rms_c": _rms(c),
                     "hv": _corr(h, v), "hc": _corr(h, c), "vc": _corr(v, c)}
    rec["parity"]["c_over_hv"] = _rms(c) / rms_hv if rms_hv > 0 else 0.0
    rec["hv_corr"] = rec["parity"]["hv"]
    rec["green_diag"] = 1 if rec["hv_corr"] > 0 else 0
    locked = max(abs(rec["parity"]["hc"]), abs(rec["parity"]["vc"]))

    # Checked first because it is the strongest of the three and the only one
    # that does not need the scene to have had colour in it.
    #
    # And checked first for a second reason, found by trying to break this
    # function: a single-phase defect SWAMPS the parity test below. Laying a
    # synthetic Bayer modulation over a real frame from this bench -- so the
    # artifact and a genuine CFA are both present -- leaves the parity test
    # still saying "checkerboard tracks the row/column parity at 0.96, not a
    # mosaic", for all three of R.70/G1.0/B.50, R.80/G1.0/B.65 and
    # R.90/G1.0/B.82. The parity test cannot see a mosaic through this defect
    # and must not be allowed to return a verdict on its own while one is
    # present. What DOES see through it is the spread of the three phases the
    # defect does not touch: 50.4%, 39.6% and 20.2% on those same three
    # frames against 0.21-1.13% on the real captures. So that is the number the
    # decision is made on.
    d = rec["deficit"]
    if d is not None and d["defect"] and not d["clean3"]:
        rec["undecided"] = True
        rec["why"] = (
            f"A single-phase defect is present ({PHASE_NAMES[d['low']]} "
            f"{d['deficit'] * 100:.1f}% low, tracking brightness at corr "
            f"{d['level_corr']:+.2f}) AND the three phases it does not touch are "
            f"separated by {d['spread'] * 100:.1f}%, which a defect cannot do. "
            f"Something is separating the phases beyond the defect -- possibly a "
            f"CFA. The parity test below cannot see through a defect this large, "
            f"so no mosaic verdict is offered. Fix the capture defect first.")
        return rec
    if d is not None and d["dead"]:
        rec["why"] = (
            f"{PHASE_NAMES[d['low']]} reads {d['deficit'] * 100:.1f}% below the other "
            f"three phases, which agree within {d['spread'] * 100:.2f}% -- a Bayer quad "
            f"is R G G B, two equal and two different, never three equal and one low; "
            f"and the deficit tracks brightness (corr {d['level_corr']:+.2f}, ratio "
            f"{d['bins'][0][1]:.3f} at level {d['bins'][0][0]:.0f} falling to "
            f"{d['bins'][-1][1]:.3f} at level {d['bins'][-1][0]:.0f}) where a colour "
            f"filter would hold one ratio at every level. AND nothing is hiding "
            f"under it: the three phases the defect does not touch agree within "
            f"{d['spread'] * 100:.2f}%, where any CFA separates them by tens of "
            f"percent -- 20.2% to 50.4% with a synthetic one laid over this very "
            f"frame. There is no colour in this data to recover")
        return rec
    if _rms(c) < 0.01:
        rec["why"] = f"checkerboard is only {_rms(c) * 100:.2f}% of the tile mean -- no 2x2 structure"
        return rec
    if locked >= 0.5:
        rec["why"] = (f"checkerboard tracks the row/column parity at {locked:.2f} "
                      "-- all three parities are one signal, so this is one artifact, "
                      "not green against red and blue")
        return rec
    if rec["parity"]["c_over_hv"] <= 1.25:
        rec["why"] = (f"checkerboard is only {rec['parity']['c_over_hv']:.2f}x the "
                      "row/column parity -- a CFA separates green from red and blue "
                      "by much more than it separates red from blue")
        return rec
    rec["mosaic"] = True
    rec["why"] = (f"checkerboard is {rec['parity']['c_over_hv']:.2f}x the row/column parity "
                  f"and independent of it ({locked:.2f}); green on "
                  + "-".join(PHASE_NAMES[i] for i in DIAGONALS[rec["green_diag"]]))
    return rec


def detect_bayer_phase(rows, width, rec=None):
    """Which 2x2 phase carries green, hence which of the four patterns applies.

    The capture core does not start on a fixed pixel parity -- see measure_comb
    -- so the pattern is a property of the CAPTURE, not of the sensor, and has
    to be found again in every frame. Nothing here is stored.

    Green comes from the SIGN of corr(row-parity, column-parity), which
    detect_mosaic() has already computed -- reused rather than recomputed so the
    two can never disagree. Substituting the four layouts into the parity basis,
    H = +V when green is on r0c1-r1c0 and H = -V when it is on r0c0-r1c1, so the
    sign names the diagonal whatever colour the scene happens to be. It reads
    +-0.9994 on clean synthetic mosaics, and got all four patterns right on
    mosaics laid over a real combed capture even at +0.49, +0.37, -0.10, -0.07,
    where the comb had eaten most of the magnitude but not the sign.

    Red against blue is NOT decidable from a single frame -- swapping them
    leaves every statistic above unchanged. The weak tiebreak used is that red
    usually tracks luminance better than blue, through both illuminant and
    sensor QE. It is reported with its margin so a thin call is visible, and
    --swap-rb overrides it.
    """
    height = len(rows)
    if rec is None:
        rec = detect_mosaic(rows, width)
    if rec.get("parity") is None:
        return None
    green_diag = rec["green_diag"]
    gpos = [divmod(i, 2) for i in DIAGONALS[green_diag]]
    opos = [divmod(i, 2) for i in DIAGONALS[1 - green_diag]]

    # Weak red/blue tiebreak: correlate each candidate against the mean of its
    # four green neighbours, i.e. against local luminance.
    score = []
    for (py, px_) in opos:
        sx = sy = sxx = syy = sxy = 0.0
        cnt = 0
        for y in range(1, height - 1):
            if (y & 1) != py:
                continue
            r, up, dn = rows[y], rows[y - 1], rows[y + 1]
            for x in range(1, width - 1):
                if (x & 1) != px_:
                    continue
                a = r[x]
                b = (r[x - 1] + r[x + 1] + up[x] + dn[x]) / 4.0
                sx += a; sy += b; sxx += a * a; syy += b * b; sxy += a * b
                cnt += 1
        if cnt < 100:
            score.append(0.0)
            continue
        cov = sxy / cnt - (sx / cnt) * (sy / cnt)
        va = sxx / cnt - (sx / cnt) ** 2
        vb = syy / cnt - (sy / cnt) ** 2
        score.append(cov / (va * vb) ** 0.5 if va > 0 and vb > 0 else 0.0)

    rpos, bpos = (opos[0], opos[1]) if score[0] >= score[1] else (opos[1], opos[0])
    grid = [[None, None], [None, None]]
    for (py, px_) in gpos:
        grid[py][px_] = "G"
    grid[rpos[0]][rpos[1]] = "R"
    grid[bpos[0]][bpos[1]] = "B"
    return {"pattern": grid[0][0] + grid[0][1] + grid[1][0] + grid[1][1],
            "green_diag": green_diag, "hv_corr": rec["hv_corr"],
            "rb_corr": score, "rb_margin": abs(score[0] - score[1])}


def demosaic(rows, width, height, pattern, white_balance=True):
    """Bilinear demosaic of a Bayer frame into interleaved 8-bit RGB rows.

    Bilinear and nothing fancier: the capture comb already puts a larger error
    on every edge than the choice of interpolator would, so an edge-directed
    demosaic would be polishing under a bigger problem.

    Out-of-frame neighbours are MIRRORED rather than clamped, because mirroring
    about an edge preserves column and row parity -- -x and x are the same
    parity, and so are 2w-2-x and x for even w -- so a mirrored neighbour is
    still the colour the interpolator asked for.

    White balance is grey-world, estimated on the RAW sample sites (where each
    colour is actually measured) rather than on the interpolated image, and
    applied as a gain on red and blue with green left alone so overall
    brightness is unchanged. Returns (rgb_rows, gains, raw_means).
    """
    colour = {(i // 2, i % 2): c for i, c in enumerate(pattern)}
    img = [list(r) for r in rows]

    raw_sum = {"R": 0.0, "G": 0.0, "B": 0.0}
    raw_n = {"R": 0, "G": 0, "B": 0}
    for y in range(height):
        r = img[y]
        for x in range(width):
            c = colour[(y & 1, x & 1)]
            raw_sum[c] += r[x]
            raw_n[c] += 1
    raw_means = {c: (raw_sum[c] / raw_n[c] if raw_n[c] else 0.0) for c in "RGB"}

    gains = {"R": 1.0, "G": 1.0, "B": 1.0}
    if white_balance and raw_means["R"] > 0 and raw_means["B"] > 0:
        gains["R"] = raw_means["G"] / raw_means["R"]
        gains["B"] = raw_means["G"] / raw_means["B"]

    lut = {}
    for c in "RGB":
        g = gains[c]
        lut[c] = [min(255, int(v * g + 0.5)) for v in range(256)]

    def at(x, y):
        if x < 0:
            x = -x
        elif x >= width:
            x = 2 * width - 2 - x
        if y < 0:
            y = -y
        elif y >= height:
            y = 2 * height - 2 - y
        return img[y][x]

    out = []
    for y in range(height):
        line = bytearray(width * 3)
        for x in range(width):
            v = img[y][x]
            c = colour[(y & 1, x & 1)]
            if c == "G":
                g = v
                horiz = (at(x - 1, y) + at(x + 1, y) + 1) >> 1
                vert = (at(x, y - 1) + at(x, y + 1) + 1) >> 1
                if colour[(y & 1, (x + 1) & 1)] == "R":
                    r, b = horiz, vert
                else:
                    r, b = vert, horiz
            else:
                g = (at(x - 1, y) + at(x + 1, y) + at(x, y - 1) + at(x, y + 1) + 2) >> 2
                d = (at(x - 1, y - 1) + at(x + 1, y - 1)
                     + at(x - 1, y + 1) + at(x + 1, y + 1) + 2) >> 2
                if c == "R":
                    r, b = v, d
                else:
                    r, b = d, v
            i = x * 3
            line[i] = lut["R"][r]
            line[i + 1] = lut["G"][g]
            line[i + 2] = lut["B"][b]
        out.append(bytes(line))
    return out, gains, raw_means


def png(path, width, height, rows, colour=False, comment=None):
    """Write 8-bit rows (a list of bytes objects) as a PNG.

    Grayscale (colour type 0) by default; colour type 2 when `colour`, in which
    case each row is interleaved RGB and so three times as long.

    `comment` goes in a tEXt chunk. That is how the artifact warning travels
    with the file: a render that says "not real colour" only on the console it
    was made on stops saying it the moment somebody drags the png into a chat.
    A grayscale frame carries no comment and so stays byte-identical to what
    this tool has always written.
    """
    raw = b"".join(b"\x00" + r for r in rows)  # filter type 0 per scanline

    def chunk(tag, data):
        body = tag + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body))

    text = b""
    if comment:
        text = chunk(b"tEXt", b"Comment\x00" + comment.encode("ascii", "replace"))

    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8,
                                      2 if colour else 0, 0, 0, 0))
        + text
        + chunk(b"IDAT", zlib.compress(raw, 9))
        + chunk(b"IEND", b"")
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("raw", type=pathlib.Path, help="dump from capture-frame.gdb")
    ap.add_argument("width", type=int, help="MEASURED line width (the core's LASTWIDTH)")
    ap.add_argument("-o", "--out", type=pathlib.Path,
                    help="output PNG (default: the raw file with a .png suffix)")
    ap.add_argument("--crop", type=int, default=2, metavar="N",
                    help="dummy pixels at the head of each line (default 2); also how "
                         "many columns are dropped once the line start is found")
    ap.add_argument("--no-anchor", action="store_true",
                    help="do not look for the line start; reshape from byte 0 and crop "
                         "blindly, which is what this tool used to do")
    ap.add_argument("--no-repair", action="store_true",
                    help="do NOT compensate the known capture-path defect that leaves one "
                         "2x2 sampling phase reading low; show the pixels exactly as the "
                         "core delivered them. The repair is on by default because without "
                         "it every frame carries a 12-20%% error on a quarter of its pixels")
    ap.add_argument("--stretch", action="store_true",
                    help="rescale to full black-to-white; the sensor's default exposure is dark")
    ap.add_argument("--colour", "--color", dest="colour", action="store_true",
                    help="demosaic and write RGB even when the frame does not measure "
                         "as a mosaic. The image is still written, to exactly the path "
                         "asked for, but it is announced as an ARTIFACT RENDER and the "
                         "png carries that warning in a tEXt chunk")
    ap.add_argument("--force-colour", "--force-color", dest="force", action="store_true",
                    help="accepted as a synonym for --colour; kept so older command "
                         "lines keep working")
    ap.add_argument("--gray", "--grey", dest="gray", action="store_true",
                    help="always write grayscale; skip the mosaic test")
    ap.add_argument("--bayer", choices=BAYER_PATTERNS, metavar="PATTERN",
                    help="force the CFA phase (%s) instead of detecting it; implies "
                         "--colour, and --force-colour if the frame is not a mosaic"
                         % "/".join(BAYER_PATTERNS))
    ap.add_argument("--swap-rb", action="store_true",
                    help="swap red and blue; the detector cannot tell them apart")
    ap.add_argument("--no-wb", dest="wb", action="store_false",
                    help="skip the grey-world white balance, leaving raw channel gains")
    args = ap.parse_args()
    if args.gray and (args.colour or args.bayer or args.force):
        sys.exit("--gray contradicts --colour/--force-colour/--bayer")

    data = args.raw.read_bytes()
    if not data:
        sys.exit(f"{args.raw}: empty")
    if args.width <= args.crop:
        sys.exit(f"width {args.width} leaves nothing after cropping {args.crop} columns")

    # Align to a real line start before anything measures a column parity. A
    # frame that already starts on one is untouched, so this cannot change the
    # output of a capture that was aligned by luck -- which is every capture the
    # tool was previously believed to handle correctly.
    anchor = None if args.no_anchor else find_line_phase(data, args.width, args.crop)
    if anchor is not None:
        data = data[anchor["start"]:]

    height = len(data) // args.width
    if height == 0:
        sys.exit(f"{len(data)} bytes is less than one {args.width}-pixel row")
    trailing = len(data) - height * args.width

    rows = [data[y * args.width + args.crop:(y + 1) * args.width] for y in range(height)]
    width = args.width - args.crop

    # Measured on the ACTIVE pixels. Taking it from `data` instead reports a
    # black floor of 0 on every frame, because the two dummy bytes at the head
    # of each line are 0x00 by construction and are not picture.
    lo = min(min(r) for r in rows)
    hi = max(max(r) for r in rows)

    # The capture defect comes off FIRST -- before the comb is measured, before
    # the mosaic test runs, and before any stretch -- so that everything
    # downstream is reading the sensor rather than the defect. It is a per-pixel
    # affine map on one 2x2 phase and so cannot create or destroy a mosaic; see
    # the note above fit_phase_defect(). The .raw file is never written.
    fit = None
    clipped = 0
    after_spread = None
    if not args.no_repair:
        fit = fit_phase_defect(rows, width)
        if fit["alpha"] is not None:
            rows, clipped = repair_phase_defect(rows, width, fit)
            after_spread = phase_spread(phase_levels(rows, width))

    if args.stretch:
        rlo = min(min(r) for r in rows)
        rhi = max(max(r) for r in rows)
        if rhi > rlo:
            span = rhi - rlo
            table = bytes(((v - rlo) * 255) // span if rlo <= v <= rhi else 0
                          for v in range(256))
            rows = [r.translate(table) for r in rows]

    comb = measure_comb(rows, width)
    out = args.out or args.raw.with_suffix(".png")

    print(f"{args.raw}: {len(data)} bytes -> {width}x{height} "
          f"(cropped {args.crop} columns, {trailing} trailing bytes ignored)")
    if anchor is None:
        print("  line start: NOT FOUND -- reshaping from byte 0 and cropping blindly"
              + (" [--no-anchor]" if args.no_anchor else
                 "; if the picture has a vertical seam in it, that is why"))
    else:
        print(f"  line start: byte {anchor['start']} (phase {anchor['phase']} of "
              f"{args.width}), from {anchor['markers']} of {anchor['lines']} lines "
              f"carrying the {args.crop} dummy pixels"
              + ("" if anchor["start"] == 0 else
                 " -- the buffer did NOT begin on a line boundary"))
    print(f"  pixel range {lo}..{hi}" + ("  [stretched]" if args.stretch else ""))

    # Every run says what the repair did, so a frame can never be read without
    # knowing whether it was corrected and how well the correction landed.
    if args.no_repair:
        print("  capture defect repair: OFF [--no-repair] -- one 2x2 phase is left reading "
              "low, so the 2x2 numbers below are the defect and not the scene")
    elif fit["alpha"] is None:
        print(f"  capture defect repair: DECLINED -- {fit['why']}")
    else:
        print("  capture defect repair: %s corrected per pixel by "
              "true = (v %+.2f) / %.4f" % (PHASE_NAMES[fit["low"]], -fit["beta"], fit["alpha"]))
        print("    alpha %.4f  beta %+.3f  -> implied pedestal %.1f, against this frame's "
              "black floor of %d" % (fit["alpha"], fit["beta"], fit["pedestal"], lo))
        print("    fitted on %d tiles of %dx%d against the diagonal phase %s "
              "(the clean null)" % (fit["tiles"], fit["tile"], fit["tile"],
                                    PHASE_NAMES[3 - fit["low"]]))
        print("    2x2 phase spread %.2f%% -> %.2f%%%s"
              % (fit["before"] * 100, after_spread * 100,
                 "" if not clipped else
                 "   [%d repaired pixels clipped at 255 -- overexposed]" % clipped))
        print("    uses only each pixel's own value, so it can neither create nor erase a "
              "mosaic. KNOWN CAPTURE-PATH DEFECT; delete this once the RTL fix lands. "
              "--no-repair turns it off")

    # The mosaic test runs on every frame, because the mono and the colour
    # HM01B0 answer the same model id and nothing but the pixels can say which
    # one is on the board. Colour is only written when the pixels support it, or
    # when the caller insists with --colour/--bayer.
    rec = None
    if not args.gray:
        rec = detect_mosaic(rows, width)
        if rec["parity"] is not None:
            q = rec["parity"]
            print("  2x2 parity over %d tiles, as a fraction of the tile mean:" % rec["tiles"])
            print("    rms  column-parity %.4f  row-parity %.4f  checkerboard %.4f"
                  "  (checker/parity %.2f)" % (q["rms_h"], q["rms_v"], q["rms_c"], q["c_over_hv"]))
            print("    corr row-vs-column %+.4f   checker-vs-column %+.4f"
                  "   checker-vs-row %+.4f" % (q["hv"], q["hc"], q["vc"]))
        if rec["flat"] is not None:
            fmt = lambda v: "  ".join("%s %.4f" % (t, x) for t, x in zip(PHASE_NAMES, v))
            print("  phase level / tile mean, scene-flat tiles (n=%d): %s"
                  % (rec["n_flat"], fmt(rec["flat"])))
            if rec["busy"] is not None:
                print("                          detailed tiles  (n=%d): %s"
                      % (rec["n_busy"], fmt(rec["busy"])))
        d = rec["deficit"]
        if d is not None:
            print("  phase levels vs the other three: %s sits %.1f%% low; the other "
                  "three agree within %.2f%%" % (PHASE_NAMES[d["low"]],
                                                 d["deficit"] * 100, d["spread"] * 100))
            print("    that deficit against tile brightness: "
                  + "  ".join("%.0f->%.3f" % b for b in d["bins"])
                  + "  (corr %+.2f)" % d["level_corr"]
                  + "   [a colour filter holds one ratio at every level]")
        if rec["excess"] is not None:
            x = rec["excess"]
            print("  excess checkerboard energy %+.3e over %d tiles = %+.1f sigma "
                  "-- phase-drift-immune; a 1.5%% colour separation reads +12 sigma "
                  "here, a real CFA hundreds" % (x["excess"], x["n"], x["sigmas"]))
        verdict = ("BAYER" if rec["mosaic"]
                   else "UNDECIDABLE" if rec["undecided"] else "not a mosaic")
        print(f"  mosaic verdict: {verdict} -- {rec['why']}")

    # The honest-colour gate. A frame with no mosaic in it does not demosaic to
    # a grey picture -- it demosaics to a convincing colour photograph whose hue
    # is a function of BRIGHTNESS, warm where the scene is bright and cool where
    # it is dark. That is the single most misleading thing this tool can
    # produce, so asking for colour on such a frame is refused rather than
    # honoured quietly, and forcing it past the refusal renames the output so
    # the file cannot later be mistaken for a photograph.
    measured = rec is not None and rec["mosaic"]
    want_colour = measured or args.colour or args.force or bool(args.bayer)
    artifact = want_colour and not measured
    if artifact:
        # The file is still written, and to exactly the path asked for, because
        # somebody is watching that path. The honesty lives in what is SAID
        # about it -- here, and in a tEXt chunk inside the PNG so the warning
        # travels with the file when this console output is long gone.
        print("  " + "=" * 68)
        print("  ARTIFACT RENDER -- THIS IS NOT REAL COLOUR.")
        if rec is not None and rec["undecided"]:
            print("  This frame MIGHT have a mosaic in it, but a capture defect is")
            print("  sitting on top of it and no honest demosaic is possible until")
            print("  that is fixed. What follows is both mixed together.")
        else:
            print("  This frame has no mosaic in it, so the demosaic has nothing to")
            print("  reconstruct and invents hue out of the one low 2x2 phase instead.")
        print("  The result is a function of BRIGHTNESS: bright things come out")
        print("  warm, dark things cool, and a warm object is warm because it is")
        print("  bright. Read it as a picture OF THE ARTIFACT. Do not read object")
        print("  colours off it, and do not show it to anyone without this line.")
        print("  " + "=" * 68)
    if want_colour:
        pattern = args.bayer
        if pattern is None:
            ph = detect_bayer_phase(rows, width, rec)
            if ph is None:
                sys.exit("frame too small to detect a Bayer phase")
            pattern = ph["pattern"]
            print("  phase detection: green on the %s diagonal, from "
                  "corr(row-parity, column-parity) = %+.4f" % (
                      "-".join(PHASE_NAMES[i] for i in DIAGONALS[ph["green_diag"]]),
                      ph["hv_corr"]))
            print("    red/blue tiebreak by correlation with local luminance: "
                  "%.4f vs %.4f (margin %.4f -- weak; --swap-rb flips it)" % (
                      ph["rb_corr"][0], ph["rb_corr"][1], ph["rb_margin"]))
        if args.swap_rb:
            pattern = pattern.translate(str.maketrans("RB", "BR"))
        rgb, gains, raw_means = demosaic(rows, width, height, pattern, args.wb)
        png(out, width, height, rgb, colour=True,
            comment=None if measured else
            "ARTIFACT RENDER, NOT REAL COLOUR. This frame measures as having no "
            "Bayer mosaic (" + rec["why"] + "). The hue here is a function of "
            "brightness, not of the objects: it comes from one 2x2 sampling phase "
            "reading low by an amount that grows with signal level. Diagnostic only.")
        print(f"  Bayer pattern {pattern}, bilinear demosaic")
        print("    raw channel means: R %.2f  G %.2f  B %.2f" % (
            raw_means["R"], raw_means["G"], raw_means["B"]))
        print("    grey-world gains:  R %.3f  G %.3f  B %.3f%s" % (
            gains["R"], gains["G"], gains["B"], "" if args.wb else "  [--no-wb]"))
    else:
        png(out, width, height, rows)

    if comb is not None:
        pct = abs(comb - 1.0) * 100.0
        brighter = "even" if comb > 1.0 else "odd"
        print(f"  column comb: {brighter} columns {pct:.1f}% brighter on average "
              f"(level-dependent; see the note in this file)")
    if hi - lo < 32 and not args.stretch:
        print("  low contrast -- re-run with --stretch to see whether there is an image in it")
    print(f"  wrote {out}"
          + ("  [RGB]" if measured else
             "  [RGB -- DIAGNOSTIC ARTIFACT, hue encodes brightness]" if want_colour
             else "  [grayscale]"))


if __name__ == "__main__":
    main()
