"""Faithful int8 top-1 for SignNet on GTSRB. extract_int8 (fold_conv_bn=False,
per-tensor sym int8, Q0.31 requant) + the golden int8_sim simulator (conv sped
up bit-exactly by fast_int8). Reports int8 vs float top-1 and STOP/YIELD recall.
Mirrors dronet_int8_eval.py but for a classifier (argmax, not EVA)."""
import os, sys, json, time, argparse
os.environ.setdefault("MODELBLASTER_GEMMINI_CONFIG","q31ws_32x32_acc")
os.environ.setdefault("MODELBLASTER_CURATED_VERIFY","0")
sys.path.insert(0,"/scratch2/dima/misc_sw/xpurt_repro_wt")
sys.path.insert(0,"/scratch2/dima/misc_sw/dronet_int8_eval")
sys.path.insert(0,"/scratch2/dima/misc_sw/gtsrb_signclf")
import numpy as np, torch
import int8_sim as I, fast_int8 as F
from modelblaster.pipeline import extract_graph as EG
from model import SignNet, SignNetSoftmax
from data import get_dataset, N_CLASSES, INPUT, STOP, YIELD

CKPT="/scratch2/dima/misc_sw/gtsrb_signclf/ckpt"
DEV="cuda" if torch.cuda.is_available() else "cpu"

def prf(preds, tgts, c):
    tp=int(((preds==c)&(tgts==c)).sum()); fp=int(((preds==c)&(tgts!=c)).sum())
    fn=int(((preds!=c)&(tgts==c)).sum()); sup=int((tgts==c).sum())
    return {"precision":tp/(tp+fp) if tp+fp else 0.0,
            "recall":tp/(tp+fn) if tp+fn else 0.0,
            "tp":tp,"fp":fp,"fn":fn,"support":sup}

def run(channels, n_calib, out_dir):
    tag="gray" if channels==1 else "rgb"
    ck=torch.load(f"{CKPT}/signnet_{tag}.pt", map_location="cpu")
    base=SignNet(in_ch=channels,n_classes=N_CLASSES,input_size=INPUT)
    base.load_state_dict(ck["state_dict"]); base.eval()
    model=SignNetSoftmax(base).eval()

    test=get_dataset("test", channels, train_aug=False)
    train=get_dataset("train", channels, train_aug=False)
    # stack test tensors + labels once (float [0,1])
    tX=torch.stack([test[i][0] for i in range(len(test))])
    tY=np.array([test[i][1] for i in range(len(test))])
    print(f"[{tag}] test={len(test)} shape={tuple(tX.shape[1:])}", flush=True)

    # ---- float top-1 (full test, GPU) ----
    model.to(DEV)
    with torch.no_grad():
        fp=[]
        for i in range(0,len(tX),512):
            fp.append(model(tX[i:i+512].to(DEV)).argmax(1).cpu().numpy())
    preds_f=np.concatenate(fp)
    top1_f=float((preds_f==tY).mean())
    print(f"[{tag}] FLOAT top1={top1_f:.4f}", flush=True)

    # ---- extract_int8 (fold_conv_bn=False) with train calibration ----
    ci=np.linspace(0,len(train)-1,n_calib).astype(int)
    calib=[train[int(i)][0].unsqueeze(0) for i in ci]
    sample=calib[0]
    os.makedirs(out_dir,exist_ok=True)
    EG.extract_int8(model.cpu(), sample, f"signnet_{tag}_int8", out_dir,
                    calibration_samples=calib, fold_conv_bn=False)
    ir=json.load(open(f"{out_dir}/graph.json"))
    wnpz=np.load(f"{out_dir}/weights.npz"); wb={k:wnpz[k] for k in wnpz.files}
    io=np.load(f"{out_dir}/io.npz")
    in_names=ir["input"]["tensors"]; out_t=ir["output"]["tensors"]
    in_scale=ir["tensors"][in_names[0]]["quant"]["scale"]
    in_shape=[1]+list(ir["tensors"][in_names[0]]["shape"])[1:]

    # ---- verify fast conv == golden (bit-exact) on one real test image;
    #      one golden call is enough to license the fast path everywhere ----
    q0=np.clip(np.round(tX[0].numpy().reshape(in_shape)/in_scale),-128,127).astype(np.int8)
    md_fast=F.verify_fast(ir["ops"], wb, [q0], in_names)
    # ---- verify the (proven-equal) fast simulator reproduces the io.npz
    #      deploy golden bit-exactly ----
    inp_q=io["input"].reshape(in_shape).astype(np.int8)
    F.enable()
    acts=I.simulate_int8(ir["ops"], wb, [inp_q], in_names)
    sim_out=np.concatenate([acts[t].reshape(-1).astype(np.int8) for t in out_t])
    gold=io["output"].astype(np.int8)
    md_io=int(np.abs(sim_out.astype(int)-gold.astype(int)).max())
    print(f"[{tag}] VERIFY io-golden max|diff|={md_io}  fast-vs-golden max|diff|={md_fast}", flush=True)

    # ---- int8 top-1 over FULL test (fast conv, per image) ----
    F.enable()
    steer_t=out_t[0]
    preds_i=np.empty(len(tX),dtype=np.int64)
    t0=time.time()
    for i in range(len(tX)):
        q=np.clip(np.round(tX[i].numpy().reshape(in_shape)/in_scale),-128,127).astype(np.int8)
        a=I.simulate_int8(ir["ops"], wb, [q], in_names)
        preds_i[i]=int(a[steer_t].reshape(-1).argmax())
        if (i+1)%2000==0:
            print(f"[{tag}] int8 {i+1}/{len(tX)} ({time.time()-t0:.1f}s)", flush=True)
    top1_i=float((preds_i==tY).mean())
    macro_rec_i=float(np.mean([prf(preds_i,tY,c)["recall"] for c in range(N_CLASSES)]))
    agree=float((preds_i==preds_f).mean())
    print(f"[{tag}] INT8 top1={top1_i:.4f} (float {top1_f:.4f})  macro_rec={macro_rec_i:.4f}  int8==float agree={agree:.4f} ({time.time()-t0:.1f}s)", flush=True)

    res={"tag":tag,"channels":channels,"n_test":len(test),"n_calib":n_calib,
         "in_scale":float(in_scale),"verify_io_maxdiff":md_io,"verify_fast_maxdiff":md_fast,
         "float_top1":top1_f,"int8_top1":top1_i,"int8_macro_recall":macro_rec_i,
         "int8_vs_float_agree":agree,
         "stop_float":prf(preds_f,tY,STOP),"stop_int8":prf(preds_i,tY,STOP),
         "yield_float":prf(preds_f,tY,YIELD),"yield_int8":prf(preds_i,tY,YIELD)}
    json.dump(res, open(f"{out_dir}/int8_results_{tag}.json","w"), indent=2)
    json.dump(res, open(f"{CKPT}/int8_results_{tag}.json","w"), indent=2)
    print(f"=== INT8_RESULTS[{tag}] ==="); print(json.dumps(res,indent=2), flush=True)
    return res

if __name__=="__main__":
    ap=argparse.ArgumentParser()
    ap.add_argument("--channels",type=int,required=True)
    ap.add_argument("--n_calib",type=int,default=128)
    a=ap.parse_args()
    tag="gray" if a.channels==1 else "rgb"
    run(a.channels, a.n_calib, f"/scratch2/dima/misc_sw/gtsrb_signclf/int8_out/signnet_{tag}")
    print("EVAL_DONE", flush=True)
