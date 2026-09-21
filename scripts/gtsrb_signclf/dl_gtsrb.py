import time, sys
from torchvision.datasets import GTSRB
ROOT="/scratch2/dima/misc_sw/gtsrb_signclf/data"
t0=time.time()
print("[dl] train split ...", flush=True)
tr=GTSRB(root=ROOT, split="train", download=True)
print(f"[dl] train n={len(tr)} ({time.time()-t0:.1f}s)", flush=True)
te=GTSRB(root=ROOT, split="test", download=True)
print(f"[dl] test  n={len(te)} ({time.time()-t0:.1f}s)", flush=True)
# sanity: label range + a sample
labels=set()
for i in range(0, len(tr), 500):
    labels.add(tr[i][1])
print(f"[dl] sample train labels seen (stride 500): min={min(labels)} max={max(labels)} count={len(labels)}", flush=True)
img,lab = tr[0]
print(f"[dl] sample img size={img.size} label={lab}", flush=True)
print("DONE_SENTINEL", flush=True)
