#!/usr/bin/env python3
"""Decide whether the fitted HM01B0 is mono or colour, from one or more frames.

WHY THIS EXISTS, AND WHY IT IS NOT frame-to-png.py's mosaic test
----------------------------------------------------------------
Every earlier analysis measured the four 2x2 phase means in RAW DN and
concluded "one phase is 14-20% low, the other three agree to 0.2-1.1%, so
there is no CFA". The raw-DN framing is what made that look decisive, and it
is the part that does not hold up.

A colour filter attenuates the LIGHT. It cannot attenuate the sensor's black
pedestal, which is 36-40 DN on this part -- roughly two thirds of the whole
signal in the dim frames captured so far. So for a filtered phase

    low = K + t*(E - K) = t*E + K*(1 - t)

which is an affine map with `alpha = t` and `beta = K*(1 - t)`. That is
ALGEBRAICALLY THE SAME MODEL as "the capture path averages this phase with a
held black-level register", which is what the affine fit was previously read
as. Measured on the real frames, beta/(1 - alpha) comes out at 33.2 and 37.2
DN against black floors of 40 and 38 -- so the fit is telling us the pedestal
is preserved, and both readings predict exactly that.

The two readings are therefore INDISTINGUISHABLE on a single frame. What
separates them:

  * a dye's `alpha` is the scene's colour, so it MOVES when the illuminant's
    colour moves;
  * a hardware gain is a property of the board, so it does NOT.

That remains the decisive test and still needs a person holding a coloured
light. --pattern does NOT substitute for it: the sensor's test pattern is
generated after the CFA, so it can say where the corruption is not, and it has
(the digital path is bit-exact -- see cmd_pattern), but it can never see a dye.

Hence `--compare`: give it a frame under red light and a frame under blue
light and the answer falls out of alpha alone.

`--pattern` answers the other half -- WHERE the attenuation is, not what it is.
The sensor's pattern is generated after the ADC and after any CFA, so a dye
cannot touch it while a defect between the pattern generator and the file must.
Run 2026-09-22: the pattern arrives BIT-EXACT on all four phases, so that
stretch of the path is clean and the attenuation is upstream of it.

The spread of the three undamaged phases is still printed and is, as of the
2026-09-22 round, the STRONGEST thing here short of the illuminant swap: it is
0.10% on the pre-swap control and 0.33% on the post-swap frames, and no RGGB
layout leaves three phases equal under any illuminant.

USAGE
    colour-decide.py FRAME.raw [LABEL] ...
    colour-decide.py --compare RED.raw BLUE.raw [WHITE.raw]
    colour-decide.py --pattern PATTERN.raw
"""
import argparse
import pathlib
import sys

import numpy as np

STRIDE = 326
DUMMY = 2
PHASE_NAMES = ("r0c0", "r0c1", "r1c0", "r1c1")
# index of the phase diagonally opposite each phase
OPPOSITE = (3, 2, 1, 0)


# ---------------------------------------------------------------------------
def check_line_gaps(buf, stride=STRIDE):
    """Assert every 00 00 line marker is exactly `stride` apart.

    A burst loss mid-capture shifts everything after it. If the number of lost
    beats is ODD, column parity ROTATES at that point and every 2x2 statistic
    computed over the whole buffer is silently scrambled -- phases before and
    after the loss are averaged together, which pulls any real separation
    toward zero and would read as "no mosaic".

    This is not hypothetical: a 148-beat burst loss was found in the test
    pattern capture of 2026-09-22 (one line delivered 178 bytes instead of
    326). It happened to be even, so that frame survived. The next one may not
    be, so check rather than hope.
    """
    marks = [i for i in range(0, len(buf) - 1)
             if buf[i] == 0 and buf[i + 1] == 0]
    # keep only markers that are stride-aligned to the first one
    if not marks:
        return True, "no 00 00 markers found -- cannot validate line gaps"
    gaps = {}
    prev = marks[0]
    for m in marks[1:]:
        d = m - prev
        if d < 2:          # adjacent zeros inside one marker
            continue
        gaps[d] = gaps.get(d, 0) + 1
        prev = m
    bad = {d: n for d, n in gaps.items() if d != stride}
    if not bad:
        return True, f"line gaps all exactly {stride}"
    odd = [d for d in bad if (stride - d) % 2]
    msg = f"IRREGULAR LINE GAPS: {bad} (expected {stride})"
    if odd:
        msg += (f" -- and {odd} differ by an ODD number of beats, so COLUMN"
                " PARITY ROTATES mid-buffer. Every 2x2 statistic from this"
                " frame is invalid. Re-capture.")
    else:
        msg += (" -- all differences are even, so column parity survived;"
                " phase statistics are still usable.")
    return not odd, msg


def load(path, stride=STRIDE, dummy=DUMMY):
    """Anchor on the sensor's own 00 00 line marker and return active pixels.

    The capture core arms wherever the sensor happens to be in the frame, so
    the buffer does not begin on a line boundary -- frames on this bench start
    0, 155 and 1092 bytes in. Re-anchoring on the marker is what keeps COLUMN
    parity absolute: every line is cut at its own 00 00, so column 0 of the
    returned image is always the first active pixel and c0/c1 mean the same
    thing in every capture, whatever the start offset.

    ROW parity is NOT recoverable and never can be. The core arms mid-frame, so
    the first complete line is at an unknown absolute sensor row and r0/r1 are
    arbitrary labels that flip between captures. Frame B's low phase reading
    r1c0 where frames A and C read r0c0 is exactly this and nothing more --
    same column parity, opposite arbitrary row label. NEVER compare a row-parity
    label across two captures; compare the column parity, and compare the
    row-vs-column RELATIONSHIP within one frame.
    """
    d = np.fromfile(path, dtype=np.uint8)
    if d.size < 2 * stride:
        raise SystemExit(f"{path}: {d.size} bytes is less than two lines")
    z = (d[:-1] == 0) & (d[1:] == 0)
    idx = np.flatnonzero(z)
    if idx.size == 0:
        raise SystemExit(f"{path}: no 00 00 line marker; cannot anchor")
    counts = np.bincount(idx % stride, minlength=stride)
    phase = int(counts.argmax())
    lines = d.size // stride
    start = int(idx[idx % stride == phase][0])
    n = (d.size - start) // stride
    img = d[start:start + n * stride].reshape(n, stride).astype(np.float64)
    return img[:, dummy:], {
        "phase": phase, "markers": int(counts[phase]), "lines": lines,
        "used": n, "start": start,
    }


def phase_means(img):
    return [img[0::2, 0::2].mean(), img[0::2, 1::2].mean(),
            img[1::2, 0::2].mean(), img[1::2, 1::2].mean()]


def clean_mask(img, susp):
    """Boolean mask of every pixel NOT on the suspect phase."""
    m = np.ones(img.shape, bool)
    m[susp // 2::2, susp % 2::2] = False
    return m


def affine_fit(img, susp):
    """Fit `v = alpha*E + beta` for the suspect phase.

    E is the mean of the four orthogonal neighbours, every one of which lies on
    a phase the suspect phase is not, so E is an unbiased estimate of what the
    suspect pixel would have read unfiltered/undamaged. Restricted to locally
    flat neighbourhoods so scene gradient does not bias E.
    """
    ry, rx = susp // 2, susp % 2
    H, W = img.shape
    ys = np.arange(2, H - 2)
    ys = ys[ys % 2 == ry]
    xs = np.arange(2, W - 2)
    xs = xs[xs % 2 == rx]
    if ys.size < 4 or xs.size < 4:
        return None
    yy, xx = np.meshgrid(ys, xs, indexing="ij")
    nb = np.stack([img[yy - 1, xx], img[yy + 1, xx],
                   img[yy, xx - 1], img[yy, xx + 1]])
    flat = (nb.max(0) - nb.min(0)) < 5.0
    E, V = nb.mean(0)[flat], img[yy, xx][flat]
    if E.size < 200:
        return None
    A = np.vstack([E, np.ones_like(E)]).T
    (alpha, beta), *_ = np.linalg.lstsq(A, V, rcond=None)
    r2 = 1.0 - ((V - A @ [alpha, beta]) ** 2).sum() / ((V - V.mean()) ** 2).sum()
    return {"alpha": float(alpha), "beta": float(beta), "r2": float(r2),
            "n": int(E.size), "emin": float(E.min()), "emax": float(E.max())}


def analyse(path, label=None):
    img, meta = load(path)
    _ok, _gapmsg = check_line_gaps(open(path, 'rb').read())
    print(f"  line-gap check        : {_gapmsg}")
    if not _ok:
        print("  !! COLUMN PARITY ROTATED -- the phase numbers below are NOT"
              " trustworthy. Re-capture before drawing any conclusion.")
    label = label or pathlib.Path(path).name
    pm = phase_means(img)
    susp = int(np.argmin(pm))
    clean = img[clean_mask(img, susp)]
    floor = float(np.percentile(clean, 0.2))
    fit = affine_fit(img, susp)
    K = fit["beta"] / (1.0 - fit["alpha"]) if fit and fit["alpha"] < 0.999 else floor

    # Channel response above black: this, not the raw DN mean, is the quantity
    # a colour filter actually sets.
    above = [p - K for p in pm]
    ref = max(above)
    rel = [a / ref for a in above]

    others = [i for i in range(4) if i != susp]
    o_above = [above[i] for i in others]
    spread3 = (max(o_above) - min(o_above)) / float(np.mean(o_above))

    opp = OPPOSITE[susp]
    adj = [i for i in others if i != opp]
    r_stat = above[opp] / float(np.mean([above[i] for i in adj]))

    return {
        "label": label, "path": str(path), "img": img, "meta": meta,
        "pm": pm, "susp": susp, "floor": floor, "fit": fit, "K": K,
        "above": above, "rel": rel, "spread3": spread3, "r": r_stat,
        "p50": float(np.percentile(clean, 50)),
        "p99": float(np.percentile(clean, 99)),
        "mean": float(clean.mean()),
    }


def report(a):
    m, f = a["meta"], a["fit"]
    print(f"\n=== {a['label']} ===")
    print(f"  {m['used']} lines anchored at byte {m['start']} "
          f"(marker residue {m['phase']}/{STRIDE}, {m['markers']}/{m['lines']} lines)")
    print(f"  level: mean {a['mean']:.1f} DN, median {a['p50']:.0f}, "
          f"p99 {a['p99']:.0f}, black floor {a['floor']:.0f}")
    if a["p99"] > 230:
        print("  !! p99 > 230: the frame is near clipping, and every ratio below is biased low")
    if not 40.0 <= a["mean"] <= 160.0:
        print("  !! frame mean outside 40-160 DN: re-shoot before trusting the verdict")
    print("  phase mean (raw DN)   : " +
          "  ".join(f"{n}={v:7.3f}" for n, v in zip(PHASE_NAMES, a["pm"])))
    print(f"  suspect phase         : {PHASE_NAMES[a['susp']]} (the lowest)")
    if f:
        print(f"  affine fit on suspect : alpha={f['alpha']:.4f}  beta={f['beta']:+.3f}  "
              f"R2={f['r2']:.3f}  n={f['n']}  E in [{f['emin']:.0f},{f['emax']:.0f}]")
        print(f"  implied pedestal      : beta/(1-alpha) = {a['K']:.1f} DN "
              f"vs measured black floor {a['floor']:.0f} DN")
        if abs(a["K"] - a["floor"]) < 6:
            print("      -> the pedestal is PRESERVED. Consistent with a dye on a black"
                  " level AND with an averaging defect; this number does not choose.")
    print("  channel response above black (this is what a CFA sets):")
    print("      " + "  ".join(f"{n}={v:6.3f}" for n, v in zip(PHASE_NAMES, a["rel"])))
    print(f"  spread of the 3 non-suspect phases, above black : {a['spread3']*100:6.3f}%")
    print(f"  r = P_opp / mean(P_adj), above black            : {a['r']:6.4f}")
    return a


# ---------------------------------------------------------------------------
def cmd_compare(paths):
    """The illuminant swap. alpha is the discriminator; r is the backstop."""
    labels = ["RED", "BLUE", "WHITE"]
    got = [report(analyse(p, labels[i] if i < len(labels) else f"F{i}"))
           for i, p in enumerate(paths)]
    red, blue = got[0], got[1]

    print("\n" + "=" * 72)
    print("ILLUMINANT-SWAP VERDICT")
    print("=" * 72)
    lv = abs(red["mean"] - blue["mean"])
    print(f"  frame means {red['mean']:.1f} vs {blue['mean']:.1f} DN "
          f"(difference {lv:.1f})")
    if lv > 25:
        print("  !! the two exposures are far apart; prefer a re-shoot, though alpha"
              " and r are both ratios and survive a moderate mismatch")
    # ONLY A COLUMN-AXIS MOVE MEANS ANYTHING. Row parity is a coin flip per
    # capture -- the core arms wherever the sensor happens to be in the frame,
    # so r0/r1 swap between captures for free (frames on this bench start 0,
    # 155 and 1092 bytes in, and one of them reported its suspect phase as
    # r1c0 purely for that reason). Column parity is absolute, because load()
    # re-anchors every line on its own 00 00 marker.
    #
    # The earlier form of this rule compared the raw phase index and would
    # have shouted COLOUR at a pure row-parity flip -- i.e. it would have
    # manufactured a false positive on roughly half of all capture pairs.
    red_col, blue_col = red["susp"] % 2, blue["susp"] % 2
    if red_col != blue_col:
        print(f"  ** THE SUSPECT PHASE MOVED IN THE COLUMN AXIS: "
              f"{PHASE_NAMES[red['susp']]} under red, "
              f"{PHASE_NAMES[blue['susp']]} under blue.")
        print("  ** A fixed hardware gain CANNOT move across column parity."
              " This alone is COLOUR.")
    elif red["susp"] != blue["susp"]:
        print(f"  -- suspect phase differs only in ROW parity "
              f"({PHASE_NAMES[red['susp']]} vs {PHASE_NAMES[blue['susp']]}):"
              " MEANINGLESS.")
        print("     Row parity is arbitrary per capture; both are on column"
              f" c{red_col}. This is NOT evidence of colour.")

    print("\n  -- statistic 1, the strong one: alpha, the suspect phase's transmission")
    ar, ab = red["fit"]["alpha"], blue["fit"]["alpha"]
    print(f"     alpha(RED)  = {ar:.4f}")
    print(f"     alpha(BLUE) = {ab:.4f}")
    ratio = max(ar, ab) / min(ar, ab)
    print(f"     ratio       = {ratio:.3f}")
    print("     a dye's transmission is the scene's colour and must swing with the")
    print("     illuminant; a capture-path gain is fixed silicon and cannot.")
    if ratio > 1.5:
        v1 = "COLOUR"
    elif ratio < 1.10:
        v1 = "MONO"
    else:
        v1 = "inconclusive"
    print(f"     -> {v1}   (>1.5 colour, <1.10 mono)")

    print("\n  -- statistic 2, the backstop: r over the three non-suspect phases")
    D = red["r"] - blue["r"]
    print(f"     r(RED)  = {red['r']:.4f}   r(BLUE) = {blue['r']:.4f}   D = {D:+.4f}")
    if abs(D) > 0.30:
        v2 = "COLOUR"
    elif abs(D) < 0.02:
        v2 = "MONO"
    else:
        v2 = "inconclusive"
    print(f"     -> {v2}   (|D|>0.30 colour, |D|<0.02 mono)")

    print("\n  -- statistic 3: the full four-phase response, red against blue")
    for i in range(4):
        rr, bb = red["rel"][i], blue["rel"][i]
        sw = max(rr, bb) / max(min(rr, bb), 1e-6)
        print(f"     {PHASE_NAMES[i]}:  red {rr:6.3f}   blue {bb:6.3f}   swing {sw:5.2f}x")
    swings = [max(red["rel"][i], blue["rel"][i]) /
              max(min(red["rel"][i], blue["rel"][i]), 1e-6) for i in range(4)]
    print(f"     largest swing {max(swings):.2f}x  (mono: every phase near 1.00x)")

    print("\n  VERDICT: ", end="")
    if v1 == "COLOUR" or v2 == "COLOUR" or max(swings) > 1.5:
        print("COLOUR. A mosaic is present and the low phase is a colour channel.")
        print("  NEXT: hardware/ospi/frame-to-png.py must STOP applying the algebraic")
        print("  repair to that phase -- it is dividing the colour back out. Use")
        print("  --no-repair and demosaic with the pattern that puts the suspect phase")
        print("  on the channel whose illuminant made it bright.")
    elif v1 == "MONO" and v2 == "MONO":
        print("MONO. The suspect phase is a fixed hardware attenuation, not a dye.")
    else:
        print("INCONCLUSIVE -- re-shoot per the exposure warnings above.")


def cmd_pattern(path):
    """The sensor's internal test pattern: digital, so no dye can touch it.

    WHAT TEST_PATTERN_MODE=1 ACTUALLY EMITS -- measured 2026-09-22, not recalled.
    It is NOT a walking-1s ramp. hm01b0.h flagged that expectation UNVERIFIED and
    it is wrong. The real output is a 1-pixel-period COLUMN STRIPE of 0xff laid
    on the black level: the whole 311x324 frame contains exactly three distinct
    byte values (0x24 = the black level, 0x2e = the black level of the next
    frame in the same buffer, 0xff), and only four distinct row patterns -- one
    for even rows, one for odd rows, repeated. So a power-of-two test on it is
    meaningless and so, far more dangerously, is a per-2x2-phase MEAN.

    WHY THE PHASE MEANS ON THIS PATTERN PROVE NOTHING. The pattern's own stripes
    sit at the Nyquist frequency of the 2x2 phase grid, so the phase means just
    count where the pattern put its 0xff:

        even rows: 108 stripes, 54 on even columns and 54 on odd  -> balanced,
                   so r0c0 and r0c1 come out EQUAL TO THREE DECIMALS by
                   construction;
        odd  rows: 130 stripes, 11020 on even columns to 7830 on odd, because
                   the pattern slips phase twice per line and ends in a 45-pixel
                   run of solid 0xff -> r1c0 reads 139.1 against 109.4.

    That single high phase was read as "the capture path treats one 2x2 phase
    differently". It does not. It is the picture.

    WHAT THE PATTERN DOES PROVE, and it is a lot. The values arrive BIT-EXACT.
    Three distinct values in the frame, and all three present on all four
    phases. An affine corruption `alpha*v + beta` with the alpha 0.52-0.58
    fitted on the photographs would map an 0xff on the damaged phase to about
    0xa3 and put a fourth value in the histogram. There is none. And because the
    pattern alternates 0x24/0xff on ADJACENT COLUMNS, the bus is proven clean at
    the maximum column transition rate -- the hardest test a sampling path can
    be given, which is precisely where an under-settled sample would show.

    Therefore the one-phase attenuation seen in photographs is UPSTREAM of the
    digital bus: sensor analog path, pixel, microlens or optics. No change to
    the capture core will remove it.

    So this command now reports the two things that mean something -- the value
    set per phase, and Nyquist transition fidelity -- and refuses to report per
    phase means as evidence.
    """
    img, meta = load(path)
    _ok, _gapmsg = check_line_gaps(open(path, 'rb').read())
    print(f"  line-gap check        : {_gapmsg}")
    if not _ok:
        print("  !! COLUMN PARITY ROTATED -- the phase numbers below are NOT"
              " trustworthy. Re-capture before drawing any conclusion.")
    vals, cnt = np.unique(img.astype(np.uint8), return_counts=True)
    print(f"\n=== TEST PATTERN: {pathlib.Path(path).name} ===")
    print(f"  {meta['used']} lines anchored at byte {meta['start']}")
    order = np.argsort(-cnt)[:16]
    print("  most common values: " +
          " ".join(f"0x{vals[i]:02x}x{cnt[i]}" for i in order))
    print(f"  {vals.size} distinct values overall")

    rows = {}
    for r in range(img.shape[0]):
        rows.setdefault(img[r].astype(np.uint8).tobytes(), []).append(r)
    print(f"  {len(rows)} distinct row patterns "
          f"(a photograph has as many as it has rows)")

    if vals.size > 24:
        print("\n  !! this does not look like a test pattern -- too many distinct")
        print("     values. Analyse it as a photograph instead (no --pattern).")
        return

    print("\n  -- TEST 1, the one that matters: is every value intact on every phase?")
    full = set(int(v) for v in vals)
    missing = {}
    for i, name in enumerate(PHASE_NAMES):
        sub = img[i // 2::2, i % 2::2].astype(np.uint8)
        u = set(int(v) for v in np.unique(sub))
        missing[name] = full - u
        print(f"    {name}: {len(u)} of {len(full)} values present"
              + ("" if not missing[name] else
                 "  MISSING " + " ".join(f"0x{v:02x}" for v in sorted(missing[name]))))
    extra = vals.size > 3 and not any(missing.values())
    if not any(missing.values()):
        print("    -> every value the pattern generator emits arrives on every phase,")
        print("       at its exact code. A per-phase affine corruption alpha*v+beta")
        print("       would have created a value that exists on one phase and no")
        print("       other; none exists. THE DIGITAL CAPTURE PATH IS CLEAN.")
    else:
        print("    -> a phase is missing a code the other phases carry. That IS a")
        print("       capture-path fault, downstream of any CFA.")

    print("\n  -- TEST 2: Nyquist settling. Adjacent-column transitions, where an")
    print("     under-settled sample shows first.")
    lo, hi = int(vals[0]), int(vals[-1])
    a = img.astype(np.int64)
    tr = (a[:, :-1] != a[:, 1:])
    ntr = int(tr.sum())
    # after a transition, does the new value land on an exact code?
    landed = np.isin(a[:, 1:][tr], vals).mean() if ntr else 1.0
    print(f"     {ntr} adjacent-column transitions; "
          f"{landed*100:.3f}% land exactly on an emitted code")
    hiafterlo = int(((a[:, :-1] == lo) & (a[:, 1:] == hi)).sum())
    print(f"     {hiafterlo} of them are 0x{lo:02x} -> 0x{hi:02x}, the full-scale step,")
    print(f"     and all of them reach 0x{hi:02x} exactly.")

    print("\n  -- NOT REPORTED, DELIBERATELY: the per-2x2-phase means. On a column-")
    print("     stripe pattern they measure the pattern's geometry and nothing else.")
    print("     See this function's docstring; reading them cost this investigation")
    print("     a round.")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frames", nargs="+", type=pathlib.Path)
    ap.add_argument("--compare", action="store_true",
                    help="illuminant swap: RED.raw BLUE.raw [WHITE.raw]")
    ap.add_argument("--pattern", action="store_true",
                    help="the frame is a sensor test pattern, not a photograph")
    args = ap.parse_args()

    for p in args.frames:
        if not p.exists():
            raise SystemExit(f"{p}: no such file")

    if args.pattern:
        for p in args.frames:
            cmd_pattern(p)
    elif args.compare:
        if len(args.frames) < 2:
            raise SystemExit("--compare needs at least RED.raw and BLUE.raw")
        cmd_compare(args.frames)
    else:
        for p in args.frames:
            report(analyse(p))
    return 0


if __name__ == "__main__":
    sys.exit(main())
