"""Compact standard-conv CNN for GTSRB, envelope-matched to the KU040 Gemmini.
All convs are plain 3x3 (NO depthwise). BN + ReLU. Global avg pool + FC + softmax
head. FX-traceable so modelblaster.extract_int8 can quantize it faithfully."""
import torch
import torch.nn as nn


class SignNet(nn.Module):
    def __init__(self, in_ch=1, n_classes=43, input_size=48,
                 widths=(32, 64, 128)):
        super().__init__()
        c0, c1, c2 = widths
        # block 1
        self.conv1a = nn.Conv2d(in_ch, c0, 3, padding=1, bias=False)
        self.bn1a = nn.BatchNorm2d(c0)
        self.conv1b = nn.Conv2d(c0, c0, 3, padding=1, bias=False)
        self.bn1b = nn.BatchNorm2d(c0)
        self.pool1 = nn.MaxPool2d(2, 2)
        # block 2
        self.conv2a = nn.Conv2d(c0, c1, 3, padding=1, bias=False)
        self.bn2a = nn.BatchNorm2d(c1)
        self.conv2b = nn.Conv2d(c1, c1, 3, padding=1, bias=False)
        self.bn2b = nn.BatchNorm2d(c1)
        self.pool2 = nn.MaxPool2d(2, 2)
        # block 3
        self.conv3a = nn.Conv2d(c1, c2, 3, padding=1, bias=False)
        self.bn3a = nn.BatchNorm2d(c2)
        self.conv3b = nn.Conv2d(c2, c2, 3, padding=1, bias=False)
        self.bn3b = nn.BatchNorm2d(c2)
        self.pool3 = nn.MaxPool2d(2, 2)
        # head: global avg pool (fixed window) -> flatten -> FC
        feat = input_size // 8  # three /2 pools; 48 -> 6
        self.gap = nn.AvgPool2d(feat)
        self.fc = nn.Linear(c2, n_classes)
        self.relu = nn.ReLU()

    def forward(self, x):
        x = self.relu(self.bn1a(self.conv1a(x)))
        x = self.relu(self.bn1b(self.conv1b(x)))
        x = self.pool1(x)
        x = self.relu(self.bn2a(self.conv2a(x)))
        x = self.relu(self.bn2b(self.conv2b(x)))
        x = self.pool2(x)
        x = self.relu(self.bn3a(self.conv3a(x)))
        x = self.relu(self.bn3b(self.conv3b(x)))
        x = self.pool3(x)
        x = self.gap(x)
        x = torch.flatten(x, 1)
        x = self.fc(x)
        return x


class SignNetSoftmax(nn.Module):
    """Deploy wrapper: base logits + softmax head (exercises softmax_s8)."""
    def __init__(self, base):
        super().__init__()
        self.base = base
    def forward(self, x):
        x = self.base(x)
        return torch.softmax(x, dim=1)


def count_params(m):
    return sum(p.numel() for p in m.parameters())


class SignNetLite(nn.Module):
    """MAC-efficient deploy variant: DroNet-style stride-2 downsampling so the
    conv work lands in the ~11M-MAC (24fps) envelope. Still all plain 3x3 dense
    convs (Gemmini-friendly), BN+ReLU, global-avg-pool + FC + softmax head."""
    def __init__(self, in_ch=1, n_classes=43, input_size=48, widths=(32, 64, 128)):
        super().__init__()
        c0, c1, c2 = widths
        self.conv1 = nn.Conv2d(in_ch, c0, 3, stride=2, padding=1, bias=False)  # 48->24
        self.bn1 = nn.BatchNorm2d(c0)
        self.conv2 = nn.Conv2d(c0, c1, 3, stride=2, padding=1, bias=False)     # 24->12
        self.bn2 = nn.BatchNorm2d(c1)
        self.conv3 = nn.Conv2d(c1, c1, 3, padding=1, bias=False)               # 12
        self.bn3 = nn.BatchNorm2d(c1)
        self.pool3 = nn.MaxPool2d(2, 2)                                        # 12->6
        self.conv4 = nn.Conv2d(c1, c2, 3, padding=1, bias=False)               # 6
        self.bn4 = nn.BatchNorm2d(c2)
        feat = input_size // 8  # 48 -> 6
        self.gap = nn.AvgPool2d(feat)
        self.fc = nn.Linear(c2, n_classes)
        self.relu = nn.ReLU()

    def forward(self, x):
        x = self.relu(self.bn1(self.conv1(x)))
        x = self.relu(self.bn2(self.conv2(x)))
        x = self.relu(self.bn3(self.conv3(x)))
        x = self.pool3(x)
        x = self.relu(self.bn4(self.conv4(x)))
        x = self.gap(x)
        x = torch.flatten(x, 1)
        x = self.fc(x)
        return x


def build(arch, in_ch, n_classes=43, input_size=48):
    if arch == "full":
        return SignNet(in_ch=in_ch, n_classes=n_classes, input_size=input_size)
    if arch == "lite":
        return SignNetLite(in_ch=in_ch, n_classes=n_classes, input_size=input_size)
    raise ValueError(arch)
