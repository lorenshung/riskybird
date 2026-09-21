import os, sys, json, time
os.environ.setdefault("MODELBLASTER_GEMMINI_CONFIG","q31ws_32x32_acc")
os.environ.setdefault("MODELBLASTER_CURATED_VERIFY","0")
sys.path.insert(0,"/scratch2/dima/misc_sw/xpurt_repro_wt")           # modelblaster pkg
sys.path.insert(0,"/scratch2/dima/misc_sw/dronet_int8_eval")        # int8_sim
sys.path.insert(0,"/scratch2/dima/misc_sw/gtsrb_signclf")           # model
import numpy as np, torch
from model import SignNet, SignNetSoftmax, count_params
from modelblaster.pipeline import extract_graph as EG
from int8_sim import simulate_int8

INPUT=48
for in_ch, tag in [(1,"gray"),(3,"rgb")]:
    torch.manual_seed(0)
    base=SignNet(in_ch=in_ch, n_classes=43, input_size=INPUT).eval()
    model=SignNetSoftmax(base).eval()
    # give BN non-trivial running stats so quantization is exercised
    with torch.no_grad():
        for m in base.modules():
            if isinstance(m, torch.nn.BatchNorm2d):
                m.running_mean.normal_(0,0.5); m.running_var.uniform_(0.5,1.5)
    print(f"[{tag}] params(base)={count_params(base):,}", flush=True)
    g=torch.Generator().manual_seed(1)
    calib=[torch.rand(1,in_ch,INPUT,INPUT,generator=g) for _ in range(8)]
    sample=calib[0]
    out=f"/scratch2/dima/misc_sw/gtsrb_signclf/int8_out/smoke_{tag}"
    os.makedirs(out,exist_ok=True)
    t0=time.time()
    EG.extract_int8(model, sample, f"smoke_{tag}", out,
                    calibration_samples=calib, fold_conv_bn=False)
    ir=json.load(open(f"{out}/graph.json"))
    wnpz=np.load(f"{out}/weights.npz"); wb={k:wnpz[k] for k in wnpz.files}
    io=np.load(f"{out}/io.npz")
    in_names=ir["input"]["tensors"]; out_tensors=ir["output"]["tensors"]
    inp_q=io["input"].reshape([1]+list(ir["tensors"][in_names[0]]["shape"])[1:]).astype(np.int8)
    acts=simulate_int8(ir["ops"], wb, [inp_q], in_names)
    sim_out=np.concatenate([acts[t].reshape(-1).astype(np.int8) for t in out_tensors])
    gold=io["output"].astype(np.int8)
    md=int(np.abs(sim_out.astype(int)-gold.astype(int)).max()) if sim_out.shape==gold.shape else -1
    ops_kinds=sorted({o["op"] for o in ir["ops"]})
    print(f"[{tag}] extract+sim {time.time()-t0:.1f}s  ops={ops_kinds}", flush=True)
    print(f"[{tag}] out_tensors={out_tensors}  shapes {sim_out.shape} vs {gold.shape}  max|diff|={md} (0=bit-exact)", flush=True)
print("SMOKE_DONE", flush=True)
