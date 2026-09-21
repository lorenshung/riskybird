"""GTSRB data + transforms, shared by train.py and eval_int8.py so float and
int8 pipelines see byte-identical preprocessing. Inputs are ToTensor [0,1]
(no mean/std norm) to match the deploy path (HM01B0 raw pixels -> /255)."""
from torchvision.datasets import GTSRB
from torchvision import transforms

ROOT = "/scratch2/dima/misc_sw/gtsrb_signclf/data"
INPUT = 48
N_CLASSES = 43
STOP = 14      # GTSRB class id for STOP
YIELD = 13     # GTSRB class id for YIELD

def get_transforms(channels, train):
    ops = [transforms.Resize((INPUT, INPUT))]
    if train:
        ops += [
            transforms.ColorJitter(brightness=0.4, contrast=0.4, saturation=0.2),
            transforms.RandomAffine(degrees=15, translate=(0.1, 0.1),
                                    scale=(0.85, 1.15)),
        ]
    if channels == 1:
        ops.append(transforms.Grayscale(1))
    ops.append(transforms.ToTensor())
    return transforms.Compose(ops)

def get_dataset(split, channels, train_aug):
    return GTSRB(root=ROOT, split=split, download=False,
                 transform=get_transforms(channels, train_aug))
