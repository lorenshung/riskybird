"""Train float SignNet on GTSRB, gray + RGB variants. Reports test top-1 and
STOP/YIELD per-class precision/recall. Saves best checkpoints + results json."""
import os, sys, json, time, argparse
sys.path.insert(0,"/scratch2/dima/misc_sw/gtsrb_signclf")
import numpy as np, torch
import torch.nn as nn
from torch.utils.data import DataLoader, random_split
from model import SignNet, count_params
from data import get_dataset, N_CLASSES, INPUT, STOP, YIELD

DEV = "cuda" if torch.cuda.is_available() else "cpu"
CKPT = "/scratch2/dima/misc_sw/gtsrb_signclf/ckpt"

def macs_of(model, in_ch):
    """Analytic MAC count for one 1x{in_ch}xINPUTxINPUT inference."""
    total = 0
    hooks = []
    def conv_hook(m, i, o):
        nonlocal total
        oc, oh, ow = o.shape[1], o.shape[2], o.shape[3]
        total += oc*oh*ow*(m.in_channels//m.groups)*m.kernel_size[0]*m.kernel_size[1]
    def lin_hook(m, i, o):
        nonlocal total
        total += m.in_features*m.out_features
    for m in model.modules():
        if isinstance(m, nn.Conv2d): hooks.append(m.register_forward_hook(conv_hook))
        elif isinstance(m, nn.Linear): hooks.append(m.register_forward_hook(lin_hook))
    model.eval()
    with torch.no_grad():
        model(torch.zeros(1, in_ch, INPUT, INPUT, device=next(model.parameters()).device))
    for h in hooks: h.remove()
    return total

@torch.no_grad()
def evaluate(model, loader):
    model.eval()
    preds=[]; tgts=[]
    for x,y in loader:
        x=x.to(DEV)
        logit=model(x)
        preds.append(logit.argmax(1).cpu().numpy()); tgts.append(y.numpy())
    preds=np.concatenate(preds); tgts=np.concatenate(tgts)
    top1=float((preds==tgts).mean())
    def pr(c):
        tp=int(((preds==c)&(tgts==c)).sum()); fp=int(((preds==c)&(tgts!=c)).sum())
        fn=int(((preds!=c)&(tgts==c)).sum()); support=int((tgts==c).sum())
        prec=tp/(tp+fp) if tp+fp else 0.0; rec=tp/(tp+fn) if tp+fn else 0.0
        return {"precision":prec,"recall":rec,"tp":tp,"fp":fp,"fn":fn,"support":support}
    return top1, {"stop":pr(STOP),"yield":pr(YIELD)}, preds, tgts

def train_variant(channels, epochs, seed=0):
    tag="gray" if channels==1 else "rgb"
    torch.manual_seed(seed); np.random.seed(seed)
    full=get_dataset("train", channels, train_aug=True)
    n_val=int(0.1*len(full)); n_tr=len(full)-n_val
    g=torch.Generator().manual_seed(seed)
    tr,va=random_split(full,[n_tr,n_val],generator=g)
    # val should not use train aug; wrap with an eval-transform view
    va_ds=get_dataset("train", channels, train_aug=False)
    va.dataset=va_ds  # share indices, eval transform
    test=get_dataset("test", channels, train_aug=False)
    trL=DataLoader(tr,batch_size=256,shuffle=True,num_workers=8,pin_memory=True,drop_last=False)
    vaL=DataLoader(va,batch_size=512,shuffle=False,num_workers=8,pin_memory=True)
    teL=DataLoader(test,batch_size=512,shuffle=False,num_workers=8,pin_memory=True)

    model=SignNet(in_ch=channels,n_classes=N_CLASSES,input_size=INPUT).to(DEV)
    nparams=count_params(model); nmacs=macs_of(model,channels)
    print(f"[{tag}] params={nparams:,} MACs={nmacs:,} ({nmacs/1e6:.1f}M) train={n_tr} val={n_val} test={len(test)}",flush=True)
    opt=torch.optim.AdamW(model.parameters(),lr=2e-3,weight_decay=5e-4)
    sched=torch.optim.lr_scheduler.CosineAnnealingLR(opt,T_max=epochs)
    crit=nn.CrossEntropyLoss(label_smoothing=0.05)
    best_va=0.0; best_state=None
    for ep in range(epochs):
        model.train(); t0=time.time(); tl=0.0; nb=0
        for x,y in trL:
            x=x.to(DEV); y=y.to(DEV)
            opt.zero_grad(); out=model(x); loss=crit(out,y)
            loss.backward(); opt.step(); tl+=loss.item(); nb+=1
        sched.step()
        va_top1,_,_,_=evaluate(model,vaL)
        if va_top1>best_va:
            best_va=va_top1; best_state={k:v.detach().cpu().clone() for k,v in model.state_dict().items()}
        print(f"[{tag}] ep{ep+1}/{epochs} loss={tl/nb:.4f} val_top1={va_top1:.4f} best={best_va:.4f} ({time.time()-t0:.1f}s)",flush=True)
    model.load_state_dict(best_state)
    torch.save({"state_dict":best_state,"channels":channels,"input":INPUT,
                "n_classes":N_CLASSES,"params":nparams,"macs":nmacs},
               f"{CKPT}/signnet_{tag}.pt")
    te_top1,pr,preds,tgts=evaluate(model,teL)
    macro_rec=float(np.mean([ (int(((preds==c)&(tgts==c)).sum())/max(1,int((tgts==c).sum()))) for c in range(N_CLASSES)]))
    res={"tag":tag,"channels":channels,"params":nparams,"macs":nmacs,
         "best_val_top1":best_va,"test_top1":te_top1,"test_macro_recall":macro_rec,
         "stop":pr["stop"],"yield":pr["yield"]}
    print(f"[{tag}] TEST top1={te_top1:.4f} macro_rec={macro_rec:.4f} STOP={pr['stop']} YIELD={pr['yield']}",flush=True)
    return res

if __name__=="__main__":
    ap=argparse.ArgumentParser(); ap.add_argument("--epochs",type=int,default=35)
    a=ap.parse_args()
    allres={}
    for ch in (1,3):
        allres["gray" if ch==1 else "rgb"]=train_variant(ch,a.epochs)
    json.dump(allres,open(f"{CKPT}/train_results.json","w"),indent=2)
    print("=== TRAIN_RESULTS ===",flush=True); print(json.dumps(allres,indent=2),flush=True)
    print("TRAIN_DONE",flush=True)
