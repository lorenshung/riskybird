"""Train the MAC-efficient SignNetLite (deploy/real-time variant) on GTSRB,
gray + RGB. Same protocol as train.py; saves signnet_lite_{tag}.pt."""
import os, sys, json, time, argparse
sys.path.insert(0,"/scratch2/dima/misc_sw/gtsrb_signclf")
import numpy as np, torch
import torch.nn as nn
from torch.utils.data import DataLoader, random_split
from model import build, count_params
from data import get_dataset, N_CLASSES, INPUT, STOP, YIELD

DEV="cuda" if torch.cuda.is_available() else "cpu"
CKPT="/scratch2/dima/misc_sw/gtsrb_signclf/ckpt"
ARCH="lite"

def macs_of(model, in_ch):
    total=0; hooks=[]
    def ch(m,i,o):
        nonlocal total
        total+=o.shape[1]*o.shape[2]*o.shape[3]*(m.in_channels//m.groups)*m.kernel_size[0]*m.kernel_size[1]
    def lh(m,i,o):
        nonlocal total; total+=m.in_features*m.out_features
    for m in model.modules():
        if isinstance(m,nn.Conv2d): hooks.append(m.register_forward_hook(ch))
        elif isinstance(m,nn.Linear): hooks.append(m.register_forward_hook(lh))
    model.eval()
    with torch.no_grad(): model(torch.zeros(1,in_ch,INPUT,INPUT,device=next(model.parameters()).device))
    for h in hooks: h.remove()
    return total

@torch.no_grad()
def evaluate(model, loader):
    model.eval(); preds=[]; tgts=[]
    for x,y in loader:
        preds.append(model(x.to(DEV)).argmax(1).cpu().numpy()); tgts.append(y.numpy())
    preds=np.concatenate(preds); tgts=np.concatenate(tgts)
    def pr(c):
        tp=int(((preds==c)&(tgts==c)).sum()); fp=int(((preds==c)&(tgts!=c)).sum())
        fn=int(((preds!=c)&(tgts==c)).sum()); sup=int((tgts==c).sum())
        return {"precision":tp/(tp+fp) if tp+fp else 0.0,"recall":tp/(tp+fn) if tp+fn else 0.0,
                "tp":tp,"fp":fp,"fn":fn,"support":sup}
    return float((preds==tgts).mean()),{"stop":pr(STOP),"yield":pr(YIELD)},preds,tgts

def train_variant(channels, epochs, seed=0):
    tag="gray" if channels==1 else "rgb"
    torch.manual_seed(seed); np.random.seed(seed)
    full=get_dataset("train",channels,True)
    n_val=int(0.1*len(full)); n_tr=len(full)-n_val
    g=torch.Generator().manual_seed(seed)
    tr,va=random_split(full,[n_tr,n_val],generator=g)
    va.dataset=get_dataset("train",channels,False)
    test=get_dataset("test",channels,False)
    trL=DataLoader(tr,256,shuffle=True,num_workers=8,pin_memory=True)
    vaL=DataLoader(va,512,shuffle=False,num_workers=8,pin_memory=True)
    teL=DataLoader(test,512,shuffle=False,num_workers=8,pin_memory=True)
    model=build(ARCH,channels,N_CLASSES,INPUT).to(DEV)
    nparams=count_params(model); nmacs=macs_of(model,channels)
    print(f"[lite/{tag}] params={nparams:,} MACs={nmacs:,} ({nmacs/1e6:.1f}M) train={n_tr} test={len(test)}",flush=True)
    opt=torch.optim.AdamW(model.parameters(),lr=2e-3,weight_decay=5e-4)
    sched=torch.optim.lr_scheduler.CosineAnnealingLR(opt,T_max=epochs)
    crit=nn.CrossEntropyLoss(label_smoothing=0.05)
    best=0.0; best_state=None
    for ep in range(epochs):
        model.train(); t0=time.time(); tl=0.0; nb=0
        for x,y in trL:
            x=x.to(DEV); y=y.to(DEV); opt.zero_grad()
            loss=crit(model(x),y); loss.backward(); opt.step(); tl+=loss.item(); nb+=1
        sched.step()
        va,_,_,_=evaluate(model,vaL)
        if va>best: best=va; best_state={k:v.detach().cpu().clone() for k,v in model.state_dict().items()}
        print(f"[lite/{tag}] ep{ep+1}/{epochs} loss={tl/nb:.4f} val={va:.4f} best={best:.4f} ({time.time()-t0:.1f}s)",flush=True)
    model.load_state_dict(best_state)
    torch.save({"state_dict":best_state,"channels":channels,"input":INPUT,"arch":ARCH,
                "n_classes":N_CLASSES,"params":nparams,"macs":nmacs},f"{CKPT}/signnet_lite_{tag}.pt")
    te,pr,preds,tgts=evaluate(model,teL)
    macro=float(np.mean([int(((preds==c)&(tgts==c)).sum())/max(1,int((tgts==c).sum())) for c in range(N_CLASSES)]))
    res={"tag":tag,"arch":ARCH,"channels":channels,"params":nparams,"macs":nmacs,
         "best_val_top1":best,"test_top1":te,"test_macro_recall":macro,"stop":pr["stop"],"yield":pr["yield"]}
    print(f"[lite/{tag}] TEST top1={te:.4f} macro_rec={macro:.4f} STOP={pr['stop']} YIELD={pr['yield']}",flush=True)
    return res

if __name__=="__main__":
    ap=argparse.ArgumentParser(); ap.add_argument("--epochs",type=int,default=40); a=ap.parse_args()
    allres={}
    for ch in (1,3): allres["gray" if ch==1 else "rgb"]=train_variant(ch,a.epochs)
    json.dump(allres,open(f"{CKPT}/train_results_lite.json","w"),indent=2)
    print("=== TRAIN_LITE_RESULTS ==="); print(json.dumps(allres,indent=2),flush=True)
    print("TRAIN_LITE_DONE",flush=True)
