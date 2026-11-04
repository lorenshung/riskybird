// NVFP4 RoCC accelerator -- host instruction interface.
//
// The ISA is specified in
//   /scratch2/loren/modelblaster/notes/nvfp4/05_nvfp4_rocc_design.md
// sections 4.1 (opcode space), 4.2 (the funct7 table and field packing),
// 4.4 (the issue sequence), and 3.5/3.5.1 (what SET_GSCALE carries and
// why). This header is the C side of that spec and nothing more: it packs
// fields and emits instructions. It does not know how the engine works.
//
// Modelled on cores/gemmini/include/gemmini.h so the mental model
// transfers. Two deliberate differences from Gemmini, both from section 4.3:
// there is no PRELOAD (the array is output-stationary, so there is no array
// state to push), and MVIN moves *two* planes -- the packed E2M1 nibbles and
// the matching E4M3 block scales -- which is why CONFIG_LD_* carries two
// strides and a second base address.
//
// v1 scope. Implemented here: CONFIG_EX, CONFIG_LD_A, CONFIG_LD_B,
// CONFIG_ST, SET_GSCALE, MVIN_A, MVIN_B, COMPUTE, COMPUTE_ACC, MVOUT,
// MVOUT_RAW, FLUSH, COUNTER.
//
// COMPUTE_ACC (funct7 8) and COUNTER (funct7 126) were listed here as
// "reserved, not implemented" until the RTL grew them; they are now emitted.
// COMPUTE_ACC is what makes a K-split dispatch expressible at all, and it is
// the instruction the section 3.3 order-independence claim is tested with
// (section 6, phase 2: "then again with K split 4 ways using COMPUTE_ACC --
// identical bits"). COUNTER is how per-dispatch accelerator-busy cycles and
// the sticky error bits are read back.
//
// Still absent: bias, and any activation other than NONE.
//
// MVOUT (funct7 9) is a special case. It is declared and emitted by this
// header, but it is NOT implemented in RTL v1: it needs a fixed-point->fp32
// normalise/round plus an fp32 multiply by the global scale, and the frozen
// vectors' OUT column uses a different global-scale convention from the ISA
// (the vectors divide by the *encode* scales gA*gB, section 3.5.1's
// reciprocals, while SET_GSCALE multiplies by the *decode* product
// fp32(s2A*s2B)), so its expected values are contested. Do not build an
// acceptance test on MVOUT until that is settled; MVOUT_RAW is the exact,
// convention-free comparison and is what the bare-metal test uses.

#ifndef SRC_MAIN_C_NVFP4_H
#define SRC_MAIN_C_NVFP4_H

#include <stdint.h>

// Reused verbatim from Gemmini -- ROCC_INSTRUCTION_0_R_R / _R_R_R are
// accelerator-agnostic and there is no reason for a second copy. Reached via
// the include path set for this target in tests/CMakeLists.txt, which points
// at <modelblaster>/cores/gemmini.
#include "rocc-software/src/xcustom.h"

// ---------------------------------------------------------------------------
// Opcode space (section 4.1)
// ---------------------------------------------------------------------------
// Gemmini owns custom3 (XCUSTOM_ACC 3, gemmini_params.h:7) and Rocket's
// RoccCommandRouter asserts if two accelerators claim the same opcode, so
// NVFP4 takes custom0 and the two can share an SoC.
#define XCUSTOM_NVFP4 0

// ---------------------------------------------------------------------------
// funct7 encoding (section 4.2)
// ---------------------------------------------------------------------------
#define k_NVFP4_CONFIG_EX    0
#define k_NVFP4_CONFIG_LD_A  1
#define k_NVFP4_CONFIG_LD_B  2
#define k_NVFP4_CONFIG_ST    3
#define k_NVFP4_SET_GSCALE   4
#define k_NVFP4_MVIN_A       5
#define k_NVFP4_MVIN_B       6
#define k_NVFP4_COMPUTE      7
#define k_NVFP4_COMPUTE_ACC  8
#define k_NVFP4_MVOUT        9    // encoded here, NOT implemented in RTL v1
#define k_NVFP4_MVOUT_RAW    10
#define k_NVFP4_FLUSH        11
#define k_NVFP4_COUNTER      126  // (matches Gemmini's k_COUNTER on purpose)

// ---------------------------------------------------------------------------
// Constants of the format and the datapath
// ---------------------------------------------------------------------------
#define NVFP4_BLOCK_K        16   // elements per E4M3 block scale
#define NVFP4_ACC_LSB_SHIFT  20   // accumulator LSB weight is 2^-20 (3.4)
#define NVFP4_ACC_BITS       64

// Activation codes. Only NONE exists in v1; the field is carried so the
// encoding does not have to change when RELU arrives.
#define NVFP4_ACT_NONE       0

// MVOUT output dtype codes (R9: phase 1 is fp32 only).
#define NVFP4_OUT_FP32       0

// CONFIG_EX rs2 flag bits.
#define NVFP4_FLAG_ACCUMULATE (1u << 0)

// COUNTER rs1 selects (funct7 126). Only these two exist.
//   0  accelerator-busy cycles -- a free-running counter that increments on
//      every cycle the engine is not idle. Read it either side of a dispatch
//      and subtract; it is cumulative, not per-dispatch.
//   1  packed sticky error bits, e.g. the section 2.4 exponent-window
//      violation. Sticky: set until reset, never cleared by a read.
#define NVFP4_COUNTER_CYCLES 0
#define NVFP4_COUNTER_ERRORS 1

// ---------------------------------------------------------------------------
// Field packing
// ---------------------------------------------------------------------------
// Tile descriptors follow Gemmini's convention exactly --
//   rows << (ADDR_LEN+16) | cols << ADDR_LEN | spad_addr
// with ADDR_LEN = 32 (gemmini_params.h:9), i.e. rows<<48 | cols<<32 | addr.
// For NVFP4 the middle field counts 16-element *blocks* along K rather than
// columns, which is what section 4.2 writes as `rows<<48 | blocks<<32 | addr`.
#define NVFP4_ADDR_LEN 32

#define NVFP4_TILE_DESC(rows, blocks, addr)                  \
  ((((uint64_t)(rows))   << (NVFP4_ADDR_LEN + 16)) |         \
   (((uint64_t)(blocks)) <<  NVFP4_ADDR_LEN)       |         \
    ((uint64_t)(uint32_t)(addr)))

// Scratchpad / accumulator addresses are 32-bit, as in Gemmini.
typedef uint32_t nvfp4_addr_t;

// fp32 <-> bits, for SET_GSCALE. The engine takes the *bit pattern*; the host
// is responsible for the rounding (see nvfp4_fuse_gscale below).
static inline uint32_t nvfp4_f32_to_bits(float f) {
  union { float f; uint32_t b; } u;
  u.f = f;
  return u.b;
}

static inline float nvfp4_bits_to_f32(uint32_t b) {
  union { float f; uint32_t b; } u;
  u.b = b;
  return u.f;
}

// ---------------------------------------------------------------------------
// Machine-state prologue
// ---------------------------------------------------------------------------
// mstatus.XS must be Dirty (0x18000) before *any* custom instruction is
// issued, or the instruction traps as illegal. Every Gemmini kernel in this
// repo does this (e.g. kernels/gemmini/gemmini_conv2d_s8_gemmini_tiled_conv.c
// :113). It is not optional and it is not a no-op.
static inline void nvfp4_enable(void) {
  asm volatile("csrs mstatus, %0" : : "r"(0x18000) : "memory");
}

// A fence is required before handing a CPU-written buffer to accelerator DMA
// and after draining one back, because the accelerator's TL port and the
// core's store buffer are not coherent with each other. Omitting it is a
// documented source of silent corruption in this repo.
static inline void nvfp4_fence(void) {
  asm volatile("fence" : : : "memory");
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
// CONFIG_EX  (funct7 0)
//   rs1  [3:0]   reserved, 0
//        [7:4]   act
//        [15:8]  M_tiles
//        [31:16] N_tiles
//        [47:32] EREF_A   (int16, section 2.4 exponent window)
//        [63:48] EREF_B   (int16)
//   rs2  [31:0]  flags   (NVFP4_FLAG_*)
//        [47:32] K_blocks
//        [63:48] reserved, 0
//
// Two notes on the field widths, both flagged rather than papered over:
//
//  * Section 4.2's table gives M_tiles 8 bits at [15:8]; the Chisel sketch in
//    section 5.5 instead decodes `cfg.mTiles := rs1(23,8)`, 16 bits, which
//    would overlap N_tiles at [31:16]. The table is the normative statement
//    and is what this header encodes. The RTL must match the table.
//  * Section 4.2's table puts K_blocks anywhere in rs2[63:32]; the sketch
//    reads rs2(47,32). [47:32] satisfies both, so that is what is used.
//
// EREF_A/EREF_B are the section 2.4 exponent-window contract, which section
// 2.4 itself recommends deferring until synthesis numbers exist. Pass 0 in v1:
// the full 29-alignment / 64-bit accumulator path ignores them.
static inline void nvfp4_extended_config_ex(uint32_t m_tiles, uint32_t n_tiles,
                                            uint32_t k_blocks, uint32_t act,
                                            uint32_t flags,
                                            int16_t eref_a, int16_t eref_b) {
  uint64_t rs1 = ((uint64_t)(uint16_t)eref_b << 48) |
                 ((uint64_t)(uint16_t)eref_a << 32) |
                 ((uint64_t)(n_tiles & 0xffffu) << 16) |
                 ((uint64_t)(m_tiles & 0xffu) << 8) |
                 ((uint64_t)(act & 0xfu) << 4);
  uint64_t rs2 = ((uint64_t)(k_blocks & 0xffffu) << 32) | (uint64_t)flags;
  ROCC_INSTRUCTION_0_R_R(XCUSTOM_NVFP4, rs1, rs2, k_NVFP4_CONFIG_EX);
}

static inline void nvfp4_config_ex(uint32_t m_tiles, uint32_t n_tiles,
                                   uint32_t k_blocks, uint32_t act,
                                   uint32_t flags) {
  nvfp4_extended_config_ex(m_tiles, n_tiles, k_blocks, act, flags, 0, 0);
}

// CONFIG_LD_A / CONFIG_LD_B  (funct7 1 / 2)
//   rs1  [63:32] data-plane row stride, bytes
//        [31:0]  scale-plane row stride, bytes
//   rs2           scale-plane base address
//
// The data-plane *base* is not configured: it is rs1 of MVIN_A / MVIN_B. Only
// the scale plane needs a base here, because one MVIN moves both planes
// (section 4.3) and there is only one address operand to spend.
//
// For the R4/R5 layouts, row strides are K/2 bytes (data) and K/16 bytes
// (scales) when the tile is a contiguous [rows, K] slab.
static inline void nvfp4_config_ld_a(uint32_t row_stride,
                                     uint32_t scale_row_stride,
                                     const void *scale_base) {
  uint64_t rs1 = ((uint64_t)row_stride << 32) | (uint64_t)scale_row_stride;
  ROCC_INSTRUCTION_0_R_R(XCUSTOM_NVFP4, rs1, (uint64_t)(uintptr_t)scale_base,
                         k_NVFP4_CONFIG_LD_A);
}

static inline void nvfp4_config_ld_b(uint32_t row_stride,
                                     uint32_t scale_row_stride,
                                     const void *scale_base) {
  uint64_t rs1 = ((uint64_t)row_stride << 32) | (uint64_t)scale_row_stride;
  ROCC_INSTRUCTION_0_R_R(XCUSTOM_NVFP4, rs1, (uint64_t)(uintptr_t)scale_base,
                         k_NVFP4_CONFIG_LD_B);
}

// CONFIG_ST  (funct7 3)
//   rs1  [63:32] C row stride, bytes
//        [31:8]  reserved, 0
//        [7:4]   act
//        [3:0]   out_dtype
//   rs2           unused; the table leaves it blank, and a RoCC instruction
//                 always sources two registers, so it is written as 0.
static inline void nvfp4_config_st(uint32_t c_row_stride, uint32_t act,
                                   uint32_t out_dtype) {
  uint64_t rs1 = ((uint64_t)c_row_stride << 32) |
                 ((uint64_t)(act & 0xfu) << 4) |
                 ((uint64_t)(out_dtype & 0xfu));
  ROCC_INSTRUCTION_0_R_R(XCUSTOM_NVFP4, rs1, (uint64_t)0, k_NVFP4_CONFIG_ST);
}

// SET_GSCALE  (funct7 4)
//   rs1  [31:0]  fp32 bits of the fused global scale g
//   rs2           bias base address, fp32 [N]; 0 = no bias
//
// The engine *multiplies* by g, once, at drain -- never inside the
// accumulation loop. g is the product of the two tensors' *decode* scales,
//
//     g = fp32( s2A * s2B ),   s2 = amax / (6 * 448)
//
// which is NVIDIA's `weight_scale_2`, i.e. the number an NVFP4 checkpoint
// already stores. Section 3.5.1 is the argument for this convention; the part
// that matters for correctness is that the *host* does the rounding to fp32.
// The exact product of two fp32 values needs up to 48 mantissa bits, so it is
// generally not an fp32 number at all and simply cannot be loaded into this
// 32-bit field. Rounding it here, explicitly, is what makes hardware and the
// software reference see the same 32 bits.
//
// Precondition the caller owns (section 3.5.1): g must be finite and normal.
// A pathologically small amax product underflows and would silently zero the
// result. nvfp4_numerics.FP32_MIN_NORMAL / FP32_MAX are the bounds.
static inline uint32_t nvfp4_fuse_gscale(uint32_t s2a_bits, uint32_t s2b_bits) {
  return nvfp4_f32_to_bits(nvfp4_bits_to_f32(s2a_bits) *
                           nvfp4_bits_to_f32(s2b_bits));
}

static inline void nvfp4_set_gscale(uint32_t g_bits, const void *bias) {
  ROCC_INSTRUCTION_0_R_R(XCUSTOM_NVFP4, (uint64_t)g_bits,
                         (uint64_t)(uintptr_t)bias, k_NVFP4_SET_GSCALE);
}

// ---------------------------------------------------------------------------
// Data movement
// ---------------------------------------------------------------------------
// MVIN_A / MVIN_B  (funct7 5 / 6)
//   rs1           DRAM address of the *data* plane (packed E2M1 nibbles)
//   rs2           rows<<48 | blocks<<32 | spad_addr
//
// Moves both planes: the nibbles from rs1 and the matching E4M3 scales from
// the base and stride set by CONFIG_LD_*. `blocks` is K/16 for the tile.
//
// R2 nibble order: element 2i is the LOW nibble of byte i, element 2i+1 the
// high nibble -- `(byte >> (4*(k&1))) & 0xF`. This matches NVIDIA's own
// packing, so a checkpoint's rows are moved in unmodified.
// R8 alignment: data base 64-byte aligned, scale base 8-byte aligned.
static inline void nvfp4_mvin_a(const void *dram, nvfp4_addr_t spad,
                                uint32_t rows, uint32_t blocks) {
  ROCC_INSTRUCTION_0_R_R(XCUSTOM_NVFP4, (uint64_t)(uintptr_t)dram,
                         NVFP4_TILE_DESC(rows, blocks, spad), k_NVFP4_MVIN_A);
}

static inline void nvfp4_mvin_b(const void *dram, nvfp4_addr_t spad,
                                uint32_t rows, uint32_t blocks) {
  ROCC_INSTRUCTION_0_R_R(XCUSTOM_NVFP4, (uint64_t)(uintptr_t)dram,
                         NVFP4_TILE_DESC(rows, blocks, spad), k_NVFP4_MVIN_B);
}

// ---------------------------------------------------------------------------
// Compute
// ---------------------------------------------------------------------------
// COMPUTE  (funct7 7)
//   rs1  A_rows<<48 | A_blocks<<32 | A_spad
//   rs2  B_rows<<48 | B_blocks<<32 | acc_addr
//
// KNOWN GAP IN THE SPEC. Section 4.2's table spends rs2's low 32 bits on
// acc_addr, so B's scratchpad address is not encoded anywhere in COMPUTE.
// There is no wording in section 4 that says what B_spad then is. This header
// encodes exactly what the table says and does not invent a field; callers
// must therefore treat the B tile as living at whatever base the preceding
// MVIN_B wrote, and the safe thing -- what the bare-metal test does -- is to
// use a single fixed B base of 0. If the RTL wants an explicit B_spad the ISA
// needs a third operand or a CONFIG field, and that is a spec change, not a
// header change.
//
// COMPUTE clears the destination accumulator tile first. COMPUTE_ACC
// (funct7 8) is the identical encoding but adds into whatever the tile
// already holds, so a K larger than the scratchpad is issued as one COMPUTE
// followed by COMPUTE_ACC per further chunk.
//
// Section 3.3 is why that is safe and not merely tolerable: the accumulator
// is plain two's-complement integer addition of exact terms, so it is
// associative, and the K-split point cannot change a single bit of the
// result. Section 6 phase 2 makes that a test rather than an assertion, and
// nvfp4_gemm.c runs it.
static inline void nvfp4_compute(nvfp4_addr_t a_spad, uint32_t a_rows,
                                 uint32_t a_blocks, uint32_t b_rows,
                                 uint32_t b_blocks, nvfp4_addr_t acc) {
  ROCC_INSTRUCTION_0_R_R(XCUSTOM_NVFP4,
                         NVFP4_TILE_DESC(a_rows, a_blocks, a_spad),
                         NVFP4_TILE_DESC(b_rows, b_blocks, acc),
                         k_NVFP4_COMPUTE);
}

// COMPUTE_ACC  (funct7 8)
//   rs1  A_rows<<48 | A_blocks<<32 | A_spad     (identical to COMPUTE)
//   rs2  B_rows<<48 | B_blocks<<32 | acc_addr
//
// Same operands, same field packing, same B_spad gap described above; the
// only difference is that the destination accumulator tile is NOT cleared
// first. The caller is responsible for having cleared it, which in practice
// means the first chunk of a K-split uses COMPUTE and every later chunk uses
// this. There is no separate "clear accumulator" instruction.
static inline void nvfp4_compute_acc(nvfp4_addr_t a_spad, uint32_t a_rows,
                                     uint32_t a_blocks, uint32_t b_rows,
                                     uint32_t b_blocks, nvfp4_addr_t acc) {
  ROCC_INSTRUCTION_0_R_R(XCUSTOM_NVFP4,
                         NVFP4_TILE_DESC(a_rows, a_blocks, a_spad),
                         NVFP4_TILE_DESC(b_rows, b_blocks, acc),
                         k_NVFP4_COMPUTE_ACC);
}

// ---------------------------------------------------------------------------
// Drain
// ---------------------------------------------------------------------------
// MVOUT      (funct7 9)   rounded output in the CONFIG_ST dtype, g applied
//                         -- ENCODED HERE BUT NOT IMPLEMENTED IN RTL v1; see
//                         the scope note at the top of this file. Kept so the
//                         encoding is fixed and so nothing else claims the
//                         funct7, but it must not be issued by a test that
//                         expects a defined result.
// MVOUT_RAW  (funct7 10)  untruncated 64-bit fixed-point accumulators,
//                         LSB 2^-20, *before* the global scale
//   rs1  DRAM address
//   rs2  rows<<48 | cols<<32 | acc_addr
//
// MVOUT_RAW has no Gemmini analogue and exists for exactly one reason
// (section 4.3): it lets the bit-exactness claim be checked against the
// software reference with zero tolerance, before any float is involved. Each
// element it writes is 8 bytes, so the destination row stride is cols*8.
static inline void nvfp4_mvout(void *dram, nvfp4_addr_t acc, uint32_t rows,
                               uint32_t cols) {
  ROCC_INSTRUCTION_0_R_R(XCUSTOM_NVFP4, (uint64_t)(uintptr_t)dram,
                         NVFP4_TILE_DESC(rows, cols, acc), k_NVFP4_MVOUT);
}

static inline void nvfp4_mvout_raw(void *dram, nvfp4_addr_t acc, uint32_t rows,
                                   uint32_t cols) {
  ROCC_INSTRUCTION_0_R_R(XCUSTOM_NVFP4, (uint64_t)(uintptr_t)dram,
                         NVFP4_TILE_DESC(rows, cols, acc), k_NVFP4_MVOUT_RAW);
}

// FLUSH  (funct7 11): rs1 = skip flag, rs2 = 0. Drains the pipeline.
static inline void nvfp4_flush(uint32_t skip) {
  ROCC_INSTRUCTION_0_R_R(XCUSTOM_NVFP4, (uint64_t)skip, (uint64_t)0,
                         k_NVFP4_FLUSH);
}

// ---------------------------------------------------------------------------
// Introspection
// ---------------------------------------------------------------------------
// COUNTER  (funct7 126)
//   rs1  counter select (NVFP4_COUNTER_*)
//   rs2  unused, 0
//   rd   the counter value
//
// This is the one instruction in the ISA that writes rd, so it is the one
// that uses ROCC_INSTRUCTION_R_R_R (funct3 0x7) rather than
// ROCC_INSTRUCTION_0_R_R (funct3 0x3). Getting that wrong encodes an
// instruction the core will not wait for a response on.
//
// The funct7 matches Gemmini's k_COUNTER (gemmini.h:43) deliberately, so
// ModelBlaster's existing per-dispatch cycle-profile plumbing transfers
// unchanged (design doc section 4.3).
//
// Ordering: the read returns whatever the engine has counted by the time the
// instruction is serviced. Issue it after a FLUSH if the number is meant to
// include the dispatch that was just launched.
static inline uint64_t nvfp4_counter(uint64_t sel) {
  uint64_t value;
  ROCC_INSTRUCTION_R_R_R(XCUSTOM_NVFP4, value, sel, (uint64_t)0,
                         k_NVFP4_COUNTER);
  return value;
}

#endif  // SRC_MAIN_C_NVFP4_H
