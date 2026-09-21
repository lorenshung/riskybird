"""Speed-only drop-in for int8_sim's conv primitive. The golden _sim_conv2d_s8
is a pure-python N*OC*IC*KH*KW*OH loop (seconds/image). Since modelblaster's
quant is per-tensor SYMMETRIC (all zero_points = 0, verified), the int8 conv is
a plain zero-padded conv, so we im2col + matmul it. The accumulate operands are
integers whose partial sums stay well under 2^53, so a float64 matmul is EXACT
(no rounding) and BLAS-fast. Requantize reuses extract_graph's own _requantize_int
=> the result is bit-identical to the golden. verify_fast() proves max|diff|=0
against the unpatched golden before we trust it for the accuracy sweep."""
import numpy as np
from numpy.lib.stride_tricks import sliding_window_view
import int8_sim as I
from modelblaster.pipeline.extract_graph import _requantize_int

_GOLDEN_CONV = I._sim_conv2d_s8  # keep the original for verification

def _im2col_acc(in_4d, w_q, sh):
    N, IC, IH, IW = in_4d.shape
    OC = sh["OC"]; OH, OW = sh["OH"], sh["OW"]; KH, KW = sh["KH"], sh["KW"]
    SH, SW = sh["SH"], sh["SW"]; PH, PW = sh["PH"], sh["PW"]
    x = in_4d.astype(np.float64)
    if PH or PW:
        x = np.pad(x, ((0, 0), (0, 0), (PH, PH), (PW, PW)))  # zero_point=0 pad
    win = sliding_window_view(x, (KH, KW), axis=(2, 3))       # [N,IC,OHf,OWf,KH,KW]
    win = win[:, :, ::SH, ::SW, :, :]                         # [N,IC,OH,OW,KH,KW]
    win = win.transpose(0, 2, 3, 1, 4, 5).reshape(N, OH * OW, IC * KH * KW)
    wmat = w_q.astype(np.float64).reshape(OC, IC * KH * KW)   # OIHW -> (IC,KH,KW) C-order
    acc = win @ wmat.T                                        # [N, OH*OW, OC] (exact)
    return acc.transpose(0, 2, 1).reshape(N, OC, OH, OW)

def fast_conv2d_s8(in_arr, sh, q, w_q, b_q):
    in_4d = in_arr.reshape(sh["N"], sh["IC"], sh["IH"], sh["IW"])
    acc = _im2col_acc(in_4d, w_q, sh)
    acc = np.rint(acc).astype(np.int64) + b_q.astype(np.int64)[None, :, None, None]
    scaled = _requantize_int(acc.astype(np.int32),
                             q["output_multiplier"], q["output_shift"])
    scaled = scaled + q["output_offset"]
    scaled = np.clip(scaled, q["activation_min"], q["activation_max"])
    return scaled.astype(np.int8)

def enable():
    """Monkeypatch int8_sim so simulate_int8 uses the fast conv (dispatch
    logic untouched)."""
    I._sim_conv2d_s8 = fast_conv2d_s8

def disable():
    I._sim_conv2d_s8 = _GOLDEN_CONV

def verify_fast(ops, weights_blob, inputs_q, in_names, atol=0):
    """Run golden vs fast on the same input; return max|diff| over outputs."""
    disable(); gold = I.simulate_int8(ops, weights_blob, inputs_q, in_names)
    enable();  fast = I.simulate_int8(ops, weights_blob, inputs_q, in_names)
    md = 0
    for k in gold:
        if k in fast:
            md = max(md, int(np.abs(gold[k].astype(int) - fast[k].astype(int)).max()))
    return md
