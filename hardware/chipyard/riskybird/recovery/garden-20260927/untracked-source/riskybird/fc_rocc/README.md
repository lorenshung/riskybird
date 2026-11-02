# Hardened FC accelerator — a custom RoCC that runs the estimator+PID loop in hardware

riskybird v3 flagship exploration. Design + feasibility prototype of a fixed-function
**RoCC accelerator** that computes the whole flight-controller loop (Mahony attitude +
complementary estimator, hierarchical PID cascade, 4×4 mixer, force→duty) as one shared
fixed-point datapath sequenced by an FSM — so the FC no longer runs on the scalar core or
the shared Saturn vector unit.

**Why:** frees the Saturn entirely for DroNet, and lets the Saturn be built **integer-only**
(it sheds its fp16 vector-FP + convert units → smaller), directly attacking the combined
SoC's **92.8 % LUT** congestion that blocks the −2 clock bump.

Status: **feasibility-quality RTL + real Vivado OOC area number + architecture verdict.**
No FPGA bench. RTL uncommitted / in a branch (git push blocked). Coordinator: riskybird-35.

---

## 1. Headline results

| | value |
|---|---|
| **FC RoCC area (Vivado OOC, xc7a200tfbg484-2 −2)** | **1,297 LUT (0.96 %) · 245 FF · 9 DSP · 1 RAMB18** |
| **Timing** | WNS **+14.46 ns @ 44 MHz** → ~120 MHz capable standalone; never the SoC critical path |
| **Loop rate** | ~1,000 cycles/FC-step → **~35 kHz @ 35 MHz** compute-bound (≥1 kHz target met with ~35× margin) |
| **Saturn FP-strip saving (int-only)** | **≈ 12 K LUT** (FPConv 5,319 + FPFMA 2,958 + FPDivSqrt 2,010 + FPComp 1,879) |
| **Net combined-area effect** | 124,924 → **≈ 114 K LUT (~85 %)** = **~8 pt off the 93 % wall** → very likely unlocks a fresh −2 route |

**Verdict: tractable, tiny, and net-positive.** The accelerator costs ~1.3 K LUT but enables
shedding ~12 K LUT of Saturn FP (plus optionally ~3 K of scalar FPU), for a net **~10–13 K LUT
freed** — real congestion relief on a LUT-bound design.

---

## 2. The datapath, from the FC math

Sourced verbatim from the fp16 kernels `samples/rose_flight_controller/src/fp16_fc_kernels.c`
(branch `fp16-estim-impl`, worktree `/home/cobble/Tools/zcs_fp16feas`) and the reference
`estimator_complementary.cpp` / `attitude_mahony.hpp` / `controller_pid.cpp`. One FC step:

**Estimator** (`kfc_estimate`)
1. Lever-arm comp: gyro-derivative LP `af += (dt/(ATAU+dt))·(deriv−af)`; accel correction from ω² terms + OFF_Y.
2. Mahony gravity-trim: `|a|`, gate `1−|‖a‖−g|/(½g)`, normalise `a/‖a‖`, predicted-up `vu` from q, error `e=â×vu`, `ω += KP·gate·e`.
3. Quaternion integrate: `dq=½·Ω(ω)·q`, `q += dq·dt`, **normalise** `q/‖q‖`.
4. Rotation matrix `R(q)` (9 elements).
5. World accel `a_w = R·a − g`; **velocity integrate** `v += a_w·dt`.
6. Flow fuse (`v_xy = R·flow`) / ZUPT; clamp `v`; **position integrate** `p += v·dt`; ToF observer `p_z,v_z += gain·(h−p_z)`.
7. get_state lead: quaternion lead by `LEADA·dt`, Rodrigues `r=q_xyz/q_w`, `p+v·h`, `v+a_w·h`, body rates = gyro.

**Controller** (`kfc_control`)
8. Altitude: `alt_int += KIH·(desH−h)·dt` (clamp); `desAcc = −2·DH·NF·v_z − NF²·(h−desH) + alt_int`; `cos(roll),cos(pitch)` (poly); `desNorm=(g+desAcc)/(cr·cp)`.
9. Horiz-vel loop: `e=desV−v`; `vel_int += KIV·e·dt` (clamp); `desAcc=(1/VTC)·e+vel_int`; `desRP=clamp([−dA2,dA1]/g)`; slew-limit.
10. Attitude loop `ratetgt=(est−tgt)·(−1/τ)`; rate loop `cmd=(gyro−ratetgt)·(−1/τ_r)`.
11. 4×4 mixer `ctrl = M·[desNorm·m, r·J0, p·J1, y·J2]`.
12. force→duty per motor: `t=in·GPN·MOT/PROP` (clamp), `disc=4·TA·t+TB²`, `d=(√disc−TB)/(2·TA)`, clamp[0,1], `−0.583`.

**Primitive ops:** multiply, add/sub, multiply-add (MAC), min/max/clamp, sign-gate, and a
handful of **reciprocal / rsqrt / sqrt** (`1/‖a‖`, `1/‖q‖`, `1/q_w`, `1/(cr·cp)`, `1/dt`,
`√disc`). Everything else (`1/TR`, `1/(2·TA)`, `GPN·MOT/PROP`, …) is a **compile-time constant**
→ no runtime divide. `cos` is a 2-term polynomial (MACs). ~300–340 ops/step, ~150 of them multiplies.

---

## 3. Arithmetic choice — minimum LUT, precision-justified

**Integer fixed-point, single global format `Q24.24` (48-bit signed).**

* Avoids FP units entirely (the whole point — and the HW probe confirms the deployed Saturn is
  fp16-only with no fp32 datapath and no fp16↔fp32 converts, so fp is not even an option here).
* **range ±8.4 × 10⁶** — covers the force-curve discriminant `disc ≈ 65,838` (which overflows any
  ≤17-int-bit format), so one format spans the whole loop.
* **resolution 6.0 × 10⁻⁸** — ≥ the fp32 mantissa near 1.0. This meets the **precision boundary**
  the fp16 Spike+pybullet harness (branch `fp16-estim-impl`) established: the *instantaneous
  algebra* tolerates fp16, but the *drift-sensitive accumulators* (quaternion state +
  integrate + normalise, velocity/position integration, gyro-derivative LP, PID integrators,
  and the `R·a − g` large−large→small cancellation) need **fp32-equivalent** precision. Q24.24
  gives ≥ fp32 abs-resolution across the entire operating range (better than fp32 near 10 m
  position and near the ~1.0 quaternion), so **no per-state format juggling** — the single wide
  int format satisfies the accumulator requirement everywhere at once.

Measured fp16-mixed-vs-fp32 drift (justifies that even fp16 algebra is fine, so Q24.24 is
comfortably conservative): closed-loop pos RMS **2.1 mm**, att **0.37 mrad**, vel **2.1 mm/s**;
open-loop replay pos RMS 0.42 mm. Orders below the drone's own ~0.15–0.3 m dead-reckoning wander.

**Divide/sqrt = Newton–Raphson on the shared multiplier (no divider, no FP):**
* reciprocal `x_{n+1}=x_n·(2−d·x_n)`, 3 iters;
* rsqrt `y_{n+1}=y_n·(1.5−0.5·d·y²)`, 3 iters; `√d = d·rsqrt(d)`.
* Seeded by a leading-zero-count initial guess (a priority encoder). Each iteration is 2–3 MACs
  that reuse the one shared multiplier → **≈ 0 extra area**, just control.

Optional future lever if DSP/LUT ever bind: a 32-bit split format (Q7.24 accumulators +
Q24.8 force path) cuts the multiplier from ~9 → ~4 DSP and shaves datapath LUT ~30 %, at the
cost of one rescale on the force path.

---

## 4. Microarchitecture

Shared, time-multiplexed datapath sequenced by a microcoded FSM — LUT is the optimisation target,
so there is exactly **one** of each expensive resource:

* **Register file** 64 × 48-bit, 3 read (a,b,c for MAC) + 1 write, inferred as **distributed RAM**
  (256 LUTRAM) → operand path stays off the general fabric.
* **One shared pipelined signed 48×48 multiplier** (3 stages, DSP-mapped → **9 DSP**), used for
  every product and every NR iteration; renormalise Q48.48→Q24.24 with saturation.
* **One shared 48-bit ALU**: add/sub (saturating), min/max, neg/abs, sign-select (gates), arith-shift,
  LZC seed. `cos`/mixer/rotation are just MAC schedules.
* **Microcode ROM** (384 × 26-bit) in **1 BRAM18** holds the op schedule (`{aluop,a,b,c,w}`).
* **FSM**: `IDLE` accepts PUSH/RUN/POP/CFG; `RUN` streams the ROM; `MULWAIT` drains the 3-stage
  multiplier. ~245 FF total.

**RoCC interface** (matches rocket-chip `RoCCCoreIO` cmd/resp/busy, custom opcode `custom0`):
* `PUSH funct=0` `rf[rs1]←rs2` — stream a Q24.24 sensor input in (FC firmware does raw→Q24.24 with
  integer shifts, no FP).
* `RUN funct=1` — run one FC step; response = done token.
* `POP funct=2` `resp←rf[rs1]` — read a Q24.24 motor duty back.
* `CFG funct=3` — load a tunable/constant.
* **Streaming interface chosen for minimum area** (no HellaCache mem port). ~15 inputs + 4 outputs
  ≈ 20 cmd cycles vs ~1,000 compute cycles. A DRAM-pointer/DMA variant (rs1/rs2 = struct pointers,
  RoCC mem port) is documented for the camera-in-loop case but costs the TLB/mem handshake logic.

---

## 5. RTL artifacts

| file | role |
|---|---|
| `FcRoCC.scala` | **Chisel prototype** — `LazyRoCC` matching chipyard, datapath+FSM+NR+regfile+`WithFcRoCC` config fragment. The integration-ready artifact. |
| `fc_accel.sv` | **OOC synthesis witness** — chipyard-independent SystemVerilog, microarchitecturally identical (same W/QF/NREG/NUOP, same one shared 48×48 mult, same ALU/NR/FSM). Synthesised for the area number. |
| `fc_accel_ooc.tcl` | Vivado OOC synth-only driver (part `xc7a200tfbg484-2`), no P&R, no bitstream — light on the garden queue. |
| `ooc/ooc_out/` | Vivado reports: `fc_accel_util.txt`, `_util_hier.txt`, `_timing_44mhz.txt`. |

**Equivalence note (honesty):** the OOC number characterises the *hardware*, which Vivado
synthesises from RTL, not from Chisel — the Chisel and SV describe the same microarchitecture,
so the SV area witnesses the Chisel. Both carry a *placeholder* microcode ROM (deterministic
non-constant fill that exercises all 14 aluops and every read/write select, so nothing is pruned);
the real C-golden-validated schedule is the same size (384×26 in 1 BRAM) driving the same datapath,
so the datapath-logic LUT (1,041) is representative and, if anything, slightly conservative.

### Reproduce the OOC number
```
scp fc_accel.sv fc_accel_ooc.tcl dima-garden:/scratch/dima/riskybird_chipyard/riskybird/fc_rocc/ooc/
ssh dima-garden 'cd .../fc_rocc/ooc && nice -n15 vivado -nojournal -mode batch -source fc_accel_ooc.tcl'
# read ooc/ooc_out/fc_accel_util.txt
```

---

## 6. Loop-rate feasibility

~300–340 ops/step; non-mul ops 1 cycle, mul/MAC ~4 cycles (3-stage drain, unpipelined worst case),
NR adds ~60 mul-ops → **~1,000 cycles/step** (generous upper bound; back-to-back mul pipelining cuts it).

* @ 35 MHz (deployed): 1,000 cyc = **28.6 µs/step → ~35 kHz** compute-bound.
* Targets: ≥1 kHz → **35× margin**; ~350 Hz-with-DroNet → **100× margin**; "much higher standalone" → **~35 kHz**.

**Honest framing:** the real FC loop is **sensor-I2C-bound** (~570 µs BMI088 read), not compute-bound,
so the RoCC does not raise the loop *rate* by itself. Its value is that it removes the FC's **~250 µs
of est+PID compute** (soft-float / Saturn-vector today) from the shared core/Saturn per iteration and
hands it to a fixed-function block that overlaps sensor I/O and DroNet — **freeing the Saturn for
DroNet**, which is the architectural goal.

---

## 7. Architecture assessment — int-only Saturn (DroNet) + FC RoCC

**Baseline (deployed combined At35, post-route, `xc7a200tfbg484-2`):**
124,924 LUT = **92.8 %** of 134,600 (LUT-bound; only ~9.7 K headroom) · 555 DSP (75 %) · FF 29 % · BRAM 67 %.
The −2 clock bump (At44) is reported to fail the stock route at this 93 % LUT / 1,681 unroutable nets,
and needs the heroic rich-phys_opt flow to close.

**The move:** FC leaves the Saturn/scalar-core → Saturn becomes **integer-only** for DroNet.

* **Saturn FP-strip = −12 K LUT** (deployed Saturn is already `robotMpcParams` fp16-only; strip the
  remaining fp16 FUs): FPConv 5,319 + FPFMA 2,958 + FPDivSqrt 2,010 + FPComp 1,879 ≈ **12,166 LUT**.
  Likely upside: fp ExecuteSequencer (~1,491) + SegmentedMultiplyPipe (~1,915) → up to ~15.5 K.
* **KEEP the integer PermuteUnit** (`noPermute=false`) — it is integer shuffle HW (vslide/vrgather/
  vcompress) that DroNet's int8 maxpool (`vslidedown`) + gather kernels **need**; dropping it saves
  only 676 LUT but **traps DroNet**. Keep the whole integer execute (`ExecutionUnitint` ~7,782 LUT).
* **Optional −~3.3 K LUT**: drop the scalar Rocket FPU (FPU 2,817 + hfma 445) — with the FC on the RoCC
  and DroNet integer, nothing needs scalar FP (already misa.F=0 in SW).
* **Add FC RoCC = +1,297 LUT** (+9 DSP, +1 BRAM18).

**Net LUT:**

| scenario | LUT | % of 134,600 |
|---|---|---|
| baseline combined At35 | 124,924 | 92.8 % |
| FP-strip Saturn (−12,166) + FC RoCC (+1,297) | **114,055** | **84.7 %** |
| + drop scalar FPU (−3,262) | **110,793** | **82.3 %** |

→ **~8–10 percentage points of LUT relief (~10–14 K LUT freed net).** For scale: that is ~2–3× the
whole LoopConv (4.8 K), on a design whose only scarce resource is LUT. DSP also improves (FP FMAs
free DSPs; FC RoCC's +9 nets well under the 185-DSP headroom) — helpful for the >44 MHz StoreController
DSP-retime path.

**Does it unlock the −2 clock bump?** The reported blocker is precisely *LUT congestion at 93 %*
(1,681 unroutable nets; only the rich phys_opt flow closes At44). Taking the design to **~83–85 %**
gives the router substantial breathing room and directly attacks that bottleneck → **very likely
enables a clean −2 route** (and eases timing closure margin), probably without the heroic flow.
Congestion is local as well as global, so this is a strong expectation, not a proof — **final
confirmation needs an actual P&R of the int-only-Saturn + FC-RoCC config.**

**Two landmines to clear first** (found in the existing `...GemminiSaturnNoFpuConfig`):
1. It **fails to elaborate** — FIRRTL `sink vector_unit.io_fp_req_ready not fully initialized`; the
   Saturn↔core FP request interface must be tied off / removed when the FP FUs are stripped.
2. It sets **`noPermute=true`** — must be **`false`** (keep the integer PermuteUnit; see above).

So the buildable target arch = **FP-strip Saturn (−12 K) + `noPermute=false` + fix the FP-interface
elaboration + FC RoCC (+1.3 K)** → net ~10–14 K LUT freed → off the 93 % wall → −2 route candidate.

---

## 8. Verdict

* **Tractable RTL?** Yes — a shared-multiplier microsequenced datapath is standard, small, and
  closes timing with enormous margin. Feasibility prototype is here (Chisel + synthesised SV witness).
* **Area?** **~1.3 K LUT / 245 FF / 9 DSP / 1 BRAM** — 0.96 % of the device. Negligible.
* **Loop rate?** ~35 kHz compute-bound; ≥1 kHz met with ~35× margin (loop stays sensor-bound, as today).
* **Composition (int-only Saturn + FC RoCC)?** **Net-reduces LUT by ~10–14 K (~8–10 pt)** → real
  relief of the 92.8 % congestion → **likely unlocks the −2 clock bump**, pending a confirming P&R and
  the two config fixes above.

### Follow-ups
1. Assemble + validate the real microprogram against the C golden (a small microassembler emitting the ROM `.hex`); wire a cocotb/Spike co-sim to the `fp16_fc_kernels.c` outputs (drift budget already characterised).
2. Integrate `FcRoCC.scala` into a chipyard config with the FP-stripped `noPermute=false` Saturn; fix the FP-interface elaboration; run `rb area` then a full −2 P&R to confirm the route + fps/loop-rate.
3. FC firmware: replace the `Fp16*`/soft-float est+PID path with RoCC `PUSH`/`RUN`/`POP` intrinsics (integer raw→Q24.24 scaling).
