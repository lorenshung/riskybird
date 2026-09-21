import os, sys, json, time
os.environ.setdefault("MODELBLASTER_GEMMINI_CONFIG","q31ws_32x32_acc")
os.environ.setdefault("MODELBLASTER_CURATED_VERIFY","0")
sys.path.insert(0,"/scratch2/dima/misc_sw/xpurt_repro_wt")
sys.path.insert(0,"/scratch2/dima/misc_sw/dronet_int8_eval")
sys.path.insert(0,"/scratch2/dima/misc_sw/gtsrb_signclf")
import numpy as np, int8_sim as I, fast_int8 as F
OUT="/scratch2/dima/misc_sw/gtsrb_signclf/int8_out/smoke_gray"
ir=json.load(open(f"{OUT}/graph.json")); wnpz=np.load(f"{OUT}/weights.npz")
wb={k:wnpz[k] for k in wnpz.files}
in_names=ir["input"]["tensors"]; in_shape=[1]+list(ir["tensors"][in_names[0]]["shape"])[1:]
rng=np.random.default_rng(0)
imgs=[rng.integers(-128,128,size=in_shape,dtype=np.int8) for _ in range(12)]
# bit-exactness fast vs golden
md=F.verify_fast(ir["ops"], wb, [imgs[0]], in_names)
print(f"[verify] fast vs golden max|diff|={md} (0=bit-exact)", flush=True)
# timing: golden 1 image
F.disable(); t=time.time(); I.simulate_int8(ir["ops"], wb, [imgs[0]], in_names)
print(f"[time] golden/img = {time.time()-t:.3f}s", flush=True)
# timing: fast, 10 images
F.enable(); t=time.time()
for im in imgs[:10]: I.simulate_int8(ir["ops"], wb, [im], in_names)
dt=(time.time()-t)/10
print(f"[time] fast/img = {dt*1000:.1f}ms  => full 12630 test ~ {dt*12630/60:.1f} min", flush=True)
print("PROBE_DONE", flush=True)
