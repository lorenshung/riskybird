# Is the fitted HM01B0 mono or colour? An adversarial re-analysis

**Verdict: the fitted sensor is MONOCHROME. Confidence: high (~95%) from data alone.**
There is no colour mosaic in any captured frame. The faint tint the renderer produces is
a capture-path defect that corrupts exactly one of the four 2x2 pixel phases, and that
same defect is present — at the same phase, with the same transfer function — in a frame
captured **before** the module was swapped, when the part was definitely mono.

This document is an independent re-derivation. Every number below was measured from the
raw bytes by this analysis, not carried over from an earlier one. Where an earlier
conclusion is confirmed it says so; where its *reasoning* was wrong it says that too.

---

## 0. Data and provenance

| label | file | bytes | note |
|---|---|---|---|
| A | scratchpad `frame-A.raw` | 102690 | post-swap |
| B | `/scratch2/loren/riskybird-dev-setup/bootup-frame.raw` (= `frame-B.raw`) | 102690 | post-swap |
| C | `/scratch2/loren/riskybird-dev/bootup-frame.raw` (= `frame-C.raw`) | 102690 | post-swap |
| CTRL | `/scratch2/loren/riskybird-dev/docs/figures/hm01b0_first_frame_326x242_gray8.raw` | 79056 | **pre-swap, known mono** |

A, B and C are three distinct captures (distinct md5s). `hardware/ospi/bootup-capture.gdb`
dumps `pixels[]` with `dump binary memory`, so these are the capture core's own bytes with
no host-side processing between the core and the file.

---

## 1. Structure: the 326 stride and the anchoring are CORRECT

This was the first thing worth attacking, because if the stride or the phase were wrong
every 2x2 statistic downstream would be meaningless. It is not wrong.

Zero-pairs (`00 00`) in each file, and how they distribute modulo candidate strides:

| frame | zero-pairs | stride 324 | **stride 326** | stride 328 | best residue |
|---|---|---|---|---|---|
| A | 315 | 162 residues, top 0.6% | **1 residue, 100.0%** | 164 residues, 0.6% | 0 |
| B | 315 | 162 residues, top 1.0% | **2 residues, top 99.0%** | 164 residues, 1.0% | 155 |
| C | 315 | 162 residues, top 0.6% | **1 residue, 100.0%** | 164 residues, 0.6% | 0 |
| CTRL | 243 | 162 residues, top 0.8% | **1 residue, 100.0%** | 164 residues, 0.8% | 0 |

Consecutive-marker gaps are 326 for 314 of 315 pairs in A and C, and 242 of 243 in CTRL.
No other stride in 322..328, nor 648/652, puts the markers at a single offset. Columns 0
and 1 of every reconstructed line are exactly `0x00` in 100.0% of lines (99.04% in B).
The two dummy bytes plus 324 active pixels is confirmed; 324 is also the HM01B0's true
active array width, which is a consistency check the file size alone does not give.

**Frame B genuinely does start mid-line** (residue 155) and must be re-anchored, exactly as
claimed. B also has one short line: a single irregular marker gap of 174 bytes at byte
offset 959. That is 0.3% of the frame and moves nothing.

**A finding the earlier analysis missed:** the pre-swap control has the **same 326-byte
stride and the same dummy-byte convention**. Its filename says `326x242` and its length is
79056 = 242 x 326 + 164 (242 full lines plus a 164-byte partial). It is therefore directly,
byte-for-byte comparably formatted with the post-swap frames, and is a legitimate control.
(79056 also happens to equal 324 x 244, which is a coincidence that could have sent this
investigation down a wrong path — the marker test rules it out.)

---

## 2. The four phase means, independently reproduced

Active columns only (2..325). Row parity is measured from the first anchored line, so it is
frame-relative, not absolute.

| frame | r0c0 | r0c1 | r1c0 | r1c1 | low phase | spread of the other 3 | low vs mean(others) |
|---|---|---|---|---|---|---|---|
| A | **50.879** | 59.610 | 59.656 | 59.465 | r0c0 | **0.320%** | **14.60% low** |
| B | 59.081 | 59.078 | **50.577** | 59.382 | r1c0 | 0.515% | 14.54% low |
| C | **51.621** | 64.604 | 64.926 | 64.195 | r0c0 | 1.131% | 20.06% low |
| CTRL | **59.328** | 61.069 | 61.094 | 61.033 | r0c0 | 0.099% | **2.84% low** |

The quoted figures (50.88 / 59.61 / 59.66 / 59.47, three equal within 0.32%, one 14.6% low)
**reproduce exactly**. B's low phase is `r1c0` rather than `r0c0` only because B's buffer
begins on the opposite line parity; the *column* parity of the defect is `c0` in all three
post-swap frames and in the control.

Brightness correlation, 16x16-tile ratio of the low phase to the mean of the other three
against tile level: **A -0.768, B -0.864, C -0.970** (quoted: -0.72 / -0.80 / -0.96). Same
sign, same magnitude; the small differences are tile-size and masking choices. Binned:

| frame | ratio at tile level ~42 | ~52 | ~61 | ~91-96 |
|---|---|---|---|---|
| A | 0.967 | 0.874 | 0.831 | 0.777 |
| C | 0.942 | 0.865 | 0.818 | 0.719 |
| CTRL | — (scene never goes there) | — | 0.975 | 0.935 (at ~67) |

So items 1 and 2 of the earlier analysis are numerically sound.

---

## 3. THE PRE-SWAP CONTROL — the most important comparison

**Same phase. Same sign. Same functional form. One fifth the magnitude.**

The control frame is definitely mono and it shows the identical signature: one phase low,
the other three equal to **0.099%**. That alone is very strong — a defect that predates the
swap cannot be caused by the swap.

But the magnitudes differ by 5-7x (2.84% vs 14.5-20.1%), and that gap is real, not a scene
artefact. Matched on tile brightness it does not close:

| tile level | CTRL deficit | frame A deficit |
|---|---|---|
| ~60 | 2.5% | 16.9% |

That looked, initially, like the hole in the conclusion: *the defect got 6x deeper exactly
when the sensor was swapped.* Section 4 shows it is not a hole — but the earlier analysis
did not measure this and would not have survived being asked about it.

---

## 4. The mechanism: an affine corruption of one phase, gain ~0.5

This is where the earlier reasoning is weakest and where the real answer is.

For a pixel on the defective phase, all four of its nearest neighbours lie on undamaged
phases, so `E` = mean(up, down, left, right) is an unbiased estimate of its true value. The
**diagonally opposite** phase has the same property and serves as a null (the earlier
analysis's implicit control — an *adjacent* phase — is contaminated, because half of an
adjacent pixel's neighbours are themselves damaged; that is worth fixing).

Fitting `low = alpha * true + beta` on locally flat pixels (neighbour spread < 5 DN):

| frame | phase | alpha | beta | R^2 |
|---|---|---|---|---|
| A | defective | 0.7550 | 7.383 | 0.905 |
| A | null (opposite) | **1.0012** | **0.103** | 0.990 |
| B | defective | 0.7128 | 9.540 | 0.899 |
| B | null | **1.0052** | **-0.094** | 0.989 |
| C | defective | **0.5237** | 17.705 | **0.987** |
| C | null | **1.0014** | **-0.732** | 0.993 |
| CTRL | defective | **0.5457** | 25.986 | 0.770 |
| CTRL | null | **1.0082** | **-0.547** | 0.910 |

The null phase is unity with zero offset in every frame, including all three post-swap
frames. **Only one phase of four is damaged** — there is no second, weaker anomaly hiding
anywhere.

The defective phase is an **affine function of the true value with a gain near 1/2 and a
pedestal**. Frame C, which has the best-conditioned fit, gives alpha = 0.524; the mono
control gives 0.546. `2*beta` tracks each frame's black floor (C: 35.4 vs floor 38;
CTRL: 52.0 vs floor 55), so the corrupted sample behaves like the average of the true pixel
with a held black-level value — a one-bit right shift with a stale pedestal, which is a
digital data-path fault, not an optical one.

**Three independent consequences, each fatal to the colour hypothesis:**

**(a) It is a function of the code value, not of the scene.** Plotting actual/expected
against expected DN for flat pixels, all three post-swap frames lie on one curve within
+/-0.02 at every level, despite different scenes and different exposures (frame C's whole
range is compressed — max 119 vs 255):

| E (DN) | 40 | 48 | 56 | 64 | 72 | 80 | 88 | 96 |
|---|---|---|---|---|---|---|---|---|
| A | 0.980 | 0.906 | 0.852 | 0.815 | 0.781 | 0.762 | 0.745 | 0.729 |
| B | 0.982 | 0.907 | 0.855 | 0.812 | 0.782 | 0.761 | 0.740 | 0.721 |
| C | 0.963 | 0.896 | 0.840 | 0.799 | 0.770 | 0.746 | 0.727 | 0.708 |
| CTRL | — | — | 1.004 | 0.961 | 0.910 | 0.865 | — | — |

A dye transmits a fixed fraction of the light. Its ratio cannot fall from 0.98 to 0.72 as
the *digital number* rises, identically across three exposures. This kills the CFA reading
of the low phase outright, and it is a much stronger argument than "the ratio is not
multiplicative" — the earlier analysis had the observation but not the mechanism.

**(b) The control lies on the same curve, shifted.** CTRL's curve is the post-swap curve
displaced by roughly +24 DN (CTRL at 72 -> 0.910 vs A at 48 -> 0.906; CTRL at 76 -> 0.892
vs A at 52 -> 0.879; CTRL at 80 -> 0.865 vs A at 56 -> 0.852). Same shape, same gain,
different onset. The post-swap frames have a black floor of 36-38 DN against the control's
55, so the pedestal that gets averaged in is lower and the corruption bites harder and
earlier. **The 6x gap of section 3 is a black-level/exposure difference, not a change of
part.** That is the answer to the most dangerous objection in this investigation.

**(c) The corruption can be undone algebraically, and doing so does not erase colour.**
Inverting the fitted affine map, `true_hat = (low - beta)/alpha`, uses *only* the pixel's
own value. Unlike replacing the phase with its neighbours' mean — which forces agreement
by construction and would destroy any genuine chroma — this repair is blind to the
neighbours and would leave a real mosaic fully intact:

| frame | 2x2 spread before | 2x2 spread after algebraic repair |
|---|---|---|
| A | 15.29% | **3.47%** |
| B | 15.44% | **3.08%** |
| C | 21.69% | **1.13%** |
| CTRL | 2.91% | **0.10%** |

A colour-preserving repair collapses the modulation to 1-3.5%. The residual sits entirely
on the repaired phase (A after repair: 57.61 / 59.61 / 59.66 / 59.47) — leftover
un-modelled defect, not a Bayer pattern, which would additionally separate one of the
*untouched* phases. This directly answers "could repairing the defect also erase genuine
colour?": **no.** The earlier analysis's neighbour-interpolation repair was vulnerable to
exactly that objection; this one is not.

---

## 5. Attacking the "three equal phases rules out Bayer" argument

The earlier claim — "a Bayer quad is R G G B, TWO equal and two different, never three
equal and one low" — is **not safe as stated**. Three-equal-one-different 2x2 mosaics exist
and are shipped: RCCC (one red, three clear, automotive), RCCB, and RGBW/RGBC variants all
produce precisely that pattern. The argument as written has a hole.

It does not matter, because three stronger arguments close it:

1. The pre-swap **mono** frame shows the same one-low-phase pattern. A mono die has no
   mosaic, so the pattern is not evidence of one.
2. The low phase's deficit is a function of DN (section 4a), which no dye can be.
3. The bound below is derived from the **three undamaged phases only** and therefore does
   not care what the fourth phase is.

### The bound, using only phases the defect never touches

For *any* 2x2 Bayer arrangement (RGGB, BGGR, GRBG, GBRG), whichever phase is damaged, one
of the remaining three carries a different dye from the other two. So any CFA must satisfy
`|ratio - 1| <=` the observed spread of those three:

| frame | three undamaged phase means | max abs deviation |
|---|---|---|
| A | 59.610 / 59.656 / 59.465 | **0.188%** |
| B | 59.081 / 59.078 / 59.382 | **0.341%** |
| C | 64.604 / 64.926 / 64.195 | **0.588%** |
| CTRL (mono) | 61.069 / 61.094 / 61.033 | 0.052% |

An uncalibrated raw Bayer sensor gives B/G in the range 0.3-0.8 under ordinary indoor
light. To read 1.000 +/- 0.006 the blue and green channels would have to integrate the
scene identically to six parts in a thousand — **and do it in three separate frames at two
different exposures.**

### Sensitivity calibration — how weak a mosaic would still have been seen

Synthetic CFAs were multiplied into real frame C (after the algebraic repair recovered a
clean scene), the measured defect was then re-applied, and the identical pipeline re-run:

| injected CFA | resulting 2x2 spread | detected? |
|---|---|---|
| none | 1.13% | — (this is the real frame) |
| R x0.97, B x0.98 | 3.12% | borderline |
| R x0.90, B x0.93 | **8.29%** | yes |
| R x0.80, B x0.85 | 16.64% | yes |
| R x0.60, B x0.50 (realistic) | **46.42%** | unmistakable |
| R x0.55, B x0.45 (typical) | 49.59% | unmistakable |

Real frames measure 1.13-3.47%. A mosaic as weak as a 7% blue deficit would already stand
out at 8.3%; a real one would read 46-50%. **The data exclude any CFA stronger than about
3% channel imbalance, roughly 15x below the weakest physically plausible Bayer filter.**

### Could a scene be colourless enough to fake this?

- **A grey lab.** Insufficient. Even a perfectly neutral scene does not equalise the phases,
  because the dyes' quantum efficiencies differ; that mismatch is what white-balance gains
  exist to correct. A grey scene removes chroma *variation*, not the channel *imbalance*.
- **Defocus.** Irrelevant. The mosaic is on the die. Blurring the scene cannot blur the
  filters, so the global channel imbalance survives any amount of defocus.
- **Sensor-side white balance.** The HM01B0 is a raw sensor with AE/AG and no per-channel
  WB gains. Even if it had them, they are global constants; local chroma would survive. The
  measured bound on spatially varying chroma (RMS of the low-pass phase difference, over and
  above the same statistic on a difference that carries no chroma) is **< 1.3% of level**
  in every frame.
- **IR swamping the filters.** This is the only escape hatch with any physics behind it, and
  it can be bounded. If a fraction `f` of the signal is visible light producing a true
  channel imbalance `d`, and `(1-f)` is IR that all dyes pass alike, the observed imbalance
  is `f*d`. For frame A (0.188% observed) and a modest true imbalance of 30%, `f <= 1.07%`
  — **over 98.9% of the signal would have to be infrared.** For frame C's looser 0.588%
  bound, still >96% IR. That would be an infrared photograph, and would itself be a
  remarkable finding about the optics.
- **A 4x4 or 6x6 mosaic (quad-Bayer, X-Trans) that a 2x2 test would miss.** Tested. Phase
  spreads on the repaired frames at period 3 are 0.19-0.80% — nothing. Period 4 and 6
  (1.33-5.45%) carry only the period-2 residual aliased in, since both are multiples of 2.

### Is the residual 1-3.5% chroma?

No — it is aliasing. Per-tile spread of the three undamaged phases correlates *positively*
with tile gradient energy in every frame: A +0.503, B +0.337, C +0.232, and **CTRL +0.621**,
the known-mono control being the strongest of the four. High-spread tiles are edge tiles,
not object tiles, which is the signature of sub-pixel sampling offsets, not of dyes.

---

## 5b. The phase-drift hypothesis — tested directly, and refuted

Every statistic above, and every statistic in the earlier analyses, is a **phase-coherent**
average: the four 2x2 phase levels are accumulated over the whole frame. If the CFA phase
**drifted within the frame** — one dropped or duplicated line inverts Bayer row parity for
everything after it — then chroma would exist locally and cancel in the frame-wide mean,
producing exactly the "three phases equal" signature from real colour data. That is the one
hole none of the numbers above can see through, so it was tested on its own terms.

### The instrument

Per 16x16 tile, take the three parity components `H`, `V`, `C` (columns, rows, checkerboard)
as fractions of the tile mean, and form

    e = <C^2> - <(H^2 + V^2)/2>

`C` is **squared before averaging**, so a sign flip anywhere in the frame cannot cancel it.
Subtracting the row/column parity energy removes what is *not* a mosaic: sensor noise and the
one-phase affine defect both land on `H`, `V` and `C` with equal magnitude, so they cancel,
whereas a CFA puts far more into `C` than into `H` and `V`.

### Calibration, including deliberately drifting phases

Synthetic CFAs laid over real frame C, repaired, same pipeline:

| injected CFA | phase behaviour | `e` | significance |
|---|---|---|---|
| R x0.60, B x0.45 | fixed | +1.55 | **+571 sigma** |
| R x0.60, B x0.45 | **flips at mid-frame** | +1.46 | **+121 sigma** |
| R x0.60, B x0.45 | **flips every 16 rows** | +1.51 | **+481 sigma** |
| R x0.85, B x0.80 | fixed | +0.155 | +171 sigma |
| R x0.85, B x0.80 | **flips every 16 rows** | +0.143 | +140 sigma |
| R x0.97, B x0.96 | fixed | +7.0e-3 | +34 sigma |
| R x0.99, B x0.985 | fixed | +1.3e-3 | +12 sigma |

Drifting the phase does **not** hide a mosaic from this statistic — it costs at most a factor
of five in significance out of several hundred. A colour separation of one and a half percent
is still a twelve-sigma result.

### The real frames

| frame | `e` | significance |
|---|---|---|
| A | -2.73e-3 | -5.0 sigma |
| B | -3.20e-3 | -6.6 sigma |
| C | -1.34e-5 | -0.2 sigma |
| CTRL (pre-swap) | -1.01e-5 | -0.9 sigma |

Zero or *negative* in all four — the negative values are scene edges, which put energy into
`H` and `V` and none into `C`. The worst single tile in any frame reaches `e` = 0.032, where a
weak `R x0.85 / B x0.80` CFA gives 0.155 in **every** tile. Nothing is chromatic anywhere in
the frame, at 8x8, 16x16 or 32x32, and re-running the fit with alpha swept from 0.04 to 1.00
never makes chroma appear.

### And the phase does not drift anyway

The defect is itself a phase marker, 12-20% deep, so row parity can be read off every line.
Per row, `mean(even columns) - mean(odd columns)`:

| frame | even rows | odd rows | rows contradicting their parity |
|---|---|---|---|
| A | -8.71 (max -2.89) | +0.19 (min -3.05) | 0 |
| B | -8.77 (max -2.69) | +0.07 (min -2.59) | 0 |
| C | -12.97 (max -3.64) | +0.73 (min -0.25) | 0 |
| CTRL | -1.74 (max -0.33) | +0.06 (min -0.28) | 0 |

**Every** even row is negative and **no** odd row is comparably negative, across all 315
lines. Line integrity agrees: all 315 lines are exactly 326 bytes, all 315 carry their
`00 00` marker, and there are zero interior `00 00` pairs. No line is dropped or duplicated
anywhere in the buffer, and row parity never flips.

### Two more destroyers, closed

- **Sensor-side Y-only subsampling** (`Y_ODD_INC = 3`), the one mode that survives the
  324-column width. On a CFA part every output row would then carry the same Bayer row type,
  so colours would alternate along each row: `H` would be enormous (-32% for a mild
  `R = 0.85G`) and `V` and `C` zero. Measured mean `H` after repair is +0.005 to +0.008, and
  `|H|` is never larger than `|V|` in any frame. Ruled out by a factor of 40.
- **On-chip per-channel gain equalising the CFA means.** This is the only mechanism that
  would null the channel imbalance while leaving local chroma, and it has a noise signature:
  a channel boosted by gain `g` carries `sqrt(g)` more shot noise at the same level. The
  three undamaged phases match to 5.9% / 6.6% / 8.5% / 14.2%, where a 2x gain would show
  +41%. Nothing is being boosted.

---

## 6. Where the earlier analysis was right, wrong, and lucky

| claim | status |
|---|---|
| 326 stride, dummy-byte anchoring | **Confirmed independently.** 100% of markers at one residue; no competing stride above 1%. |
| Phase means 50.88 / 59.61 / 59.66 / 59.47 | **Reproduced exactly.** |
| corr(ratio, level) = -0.72 / -0.80 / -0.96 | **Reproduced** as -0.768 / -0.864 / -0.970. |
| "A Bayer quad is R G G B, never three equal and one low" | **Unsound.** RCCC / RCCB / RGBW mosaics are exactly three-equal-one-low. Conclusion survives via other routes. |
| Neighbour-interpolation repair collapses the separation | True but **circular** — that repair forces agreement and would erase real colour. Replaced here with an algebraic repair that cannot. |
| Datasheet says "Monochrome" | True, but **it is the wrong datasheet for this question.** `HM01B0-ANA-00FT870` is the *previously fitted* module's part number. What the old module was says nothing about what the user just plugged in. This should carry no weight. |
| Did not test the pre-swap control against the post-swap frames at matched conditions | The gap (2.84% vs 14.6-20.1%) is real and was the strongest available objection. It is resolved here by the black-level shift, not by the earlier reasoning. |

Net: **the conclusion is correct and now rests on better evidence.** Two of its five
supporting arguments should be dropped.

---

## 7. What software can and cannot settle — physical checks

Mono and colour HM01B0s both report MODEL_ID `0x01B0`, so no register read will ever answer
this. Do one of these:

1. **Read the part number** off the module flex or carrier silkscreen. The known-mono part
   here is `HM01B0-ANA-00FT870`. A different suffix is the single most useful fact available
   and takes ten seconds with a phone macro shot.
2. **Look at the die.** Remove or look past the lens and inspect the sensor under bright
   oblique light at 10-20x (a loupe, or a phone macro lens). A Bayer CFA shows a visibly
   iridescent, coloured grid. A mono die is a uniform grey-blue anti-reflective surface.
3. **Supplier / order record.** Whatever listing the replacement came from should name the
   variant. If it just says "HM01B0", assume mono — the colour variant is the rarer SKU and
   is normally called out explicitly.
4. Confirm the sensor is not in a binning or subsample mode (`BINNING_MODE`, `X/Y_ODD_INC`)
   before trusting *any* colour test, since 2x2 binning averages a whole Bayer quad and
   would erase a mosaic at source. The captures here are 324 active columns wide, the full
   array width, so no horizontal binning is in effect.

---

## 8. THE DEFINITIVE BENCH TEST

The strongest test needs no RTL fix: it uses only the three phases the defect never
touches. It also cancels the defect, the lens, the scene and the exposure.

### Setup

- **Subject:** a matte white card filling the frame, at the module's focus distance
  (roughly 10-30 cm). Flat and uniform — this test does not need a colourful subject,
  it needs a colourful *illuminant*, which is far more controllable.
- **Illumination:** two captures, lights otherwise off.
  - **Capture RED:** illuminate the card with a saturated red source. A phone screen
    showing full-screen pure red (255,0,0) at maximum brightness, held ~10 cm from the
    card, is ideal — phone OLED/LCD primaries are narrow-band and carry essentially no IR.
  - **Capture BLUE:** same geometry, full-screen pure blue (0,0,255).
  - Do **not** use an incandescent bulb, a halogen lamp or sunlight. Those are IR-rich, and
    with no IR-cut filter in the optical path they are the one condition that can
    legitimately wash out a mosaic (section 5).
- **Exposure:** let AE settle, then confirm the frame mean lands in **60-140 DN with the
  99th percentile below 220** (no clipping). Clipping and the defect both bite hardest at
  high DN; keep the two captures at comparable mean level, within ~15 DN of each other.
- Optionally also capture a **white** reference (full-screen white) for a third point.
- If the capture-RTL fix has landed, take the same three frames anyway; the test only gets
  stronger with four clean phases instead of three.

### The number that decides it

For each capture, anchor on the `00 00` markers at stride 326, take active columns 2..325,
and compute the four 2x2 phase means. Identify the defective phase as the lowest (skip this
if the RTL is fixed and all four agree). Then, from the **three undamaged phases**, let
`P_opp` be the one diagonally opposite the defect and `P_adj1`, `P_adj2` the other two.

**Statistic 1 — within-frame phase spread:**

    S = (max - min) / mean   over the three undamaged phase means

| result | reading |
|---|---|
| S < 2% | **mono** (measured today: 0.19% / 0.34% / 0.59%; mono control 0.05%) |
| S > 5% | **colour** (injected 7% mosaic gives 8.3%; a real one gives 46-50%) |
| 2% < S < 5% | inconclusive — re-shoot with less clipping |

**Statistic 2 — the illuminant-swap differential (the decisive one):**

    r(frame) = P_opp / ((P_adj1 + P_adj2) / 2)
    D        = r(RED) - r(BLUE)

`D` cancels the lens, the scene, the exposure and the capture defect, because all of those
act identically on all three phases. Only a dye responds to the illuminant.

| result | reading |
|---|---|
| \|D\| < 0.02 | **mono**, conclusively |
| \|D\| > 0.30 | **colour**, conclusively |

Today's three frames give `r` = 0.997 / 0.997 / 0.991 and the mono control 0.999 — all
within 0.9% of unity, from which `D` cannot plausibly exceed 0.02. On a colour part, `r`
would swing from roughly 0.4 under red light to 2.0 under blue (or the reverse, depending
on which phase carries which dye): `|D| > 1.0`.

Run the analysis with `/usr/bin/python3` (numpy 1.26.4); the scripts used here are in the
session scratchpad under `advers/`, with `s12_final.py` computing exactly these statistics.

---

## 9. Verdict and the one thing that would change it

**The fitted part is monochrome.** The user is seeing a rendered tint because one of the
four pixel phases is being corrupted by an affine map of gain ~0.5 in the capture path, and
the PNG renderer is interpreting that 2x2 modulation as chroma. "Not enough colour, still
mostly grey with a little colour" is the correct description of a mono frame with one
damaged phase demosaiced as if it were Bayer. Once the capture defect is fixed the tint will
disappear entirely, and the image will be grey — that is the expected outcome, not a
regression.

**The algebraic repair now ships.** `hardware/ospi/frame-to-png.py` fits alpha and beta per
frame and applies `true = (v - beta) / alpha` to the damaged phase on the way to the PNG,
before the comb measurement, the mosaic test and any stretch. The `.raw` is untouched and
`--no-repair` turns it off. It is a per-pixel affine map reading only that pixel's own value,
so it can neither manufacture nor erase a mosaic. The mosaic test now runs on repaired data
and **still returns "not a mosaic"** — which is the stronger statement, since the defect is
no longer sitting on top of it. Measured 2x2 phase spread, before -> after: A 15.29% ->
1.28%, B 15.55% -> 0.28%, C 21.69% -> 1.13%, CTRL 2.91% -> 0.28%. Unlike the naive
single-gain equalisation recorded in `measure_comb()`, which took the ratio to 1.002 while
raising the adjacent-pixel difference from 6.39 to 6.91, this repair lowers it too:
6.01 -> 3.76, 5.99 -> 3.13, 7.28 -> 1.74, 1.59 -> 1.45.

Confidence: **high, ~95%.** The 5% is not data uncertainty — the data are decisive within
themselves — it is the residual chance that the frames were all taken under some condition
that suppresses a mosaic (extreme IR dominance, or an unnoticed binning/subsample mode).
Both are checkable, and neither is likely.

**The single measurement that would change my mind:** a capture under saturated red
illumination in which the spread of the three undamaged phases exceeds 5%, or in which the
red/blue illuminant-swap differential `|D|` exceeds 0.30. Nothing short of that should
move anyone off "mono" — and a 30-second look at the module's part number would settle it
faster than any capture.
