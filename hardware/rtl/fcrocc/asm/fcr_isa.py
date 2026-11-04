"""FcRoCC microword format and bit-exact datapath model (mirrors vsrc/fc_rocc_core.sv).

Microword (64-bit ROM word; bits 63:43 are always zero):
    [3:0]   op    opcode
    [10:4]  sh    signed 7-bit renorm shift (MUL only: prod >>> sh, or <<< -sh if sh < 0)
    [18:11] fd    destination register
    [26:19] fa    source A (also the POP read port when idle)
    [34:27] fb    source B
    [42:35] fc    source C (SEL condition only)
"""

W = 48
MASK = (1 << W) - 1
SMAX = (1 << (W - 1)) - 1
SMIN = -(1 << (W - 1))

OPS = {0: "MUL", 1: "CLT", 2: "ADD", 3: "SUB", 4: "MIN", 5: "MAX", 6: "MOV",
       7: "NEG", 10: "MSB", 11: "VSH", 12: "SEL", 13: "HALT"}
OPCODE = {v: k for k, v in OPS.items()}


def s48(x):
    x &= MASK
    return x - (1 << W) if x >> (W - 1) else x


def sat(x):
    return SMAX if x > SMAX else SMIN if x < SMIN else x


def wrap96(x):
    x &= (1 << 96) - 1
    return x - (1 << 96) if x >> 95 else x


def decode(w):
    sh = (w >> 4) & 0x7F
    if sh >= 64:
        sh -= 128
    return dict(op=w & 0xF, sh=sh, fd=(w >> 11) & 0xFF, fa=(w >> 19) & 0xFF,
                fb=(w >> 27) & 0xFF, fc=(w >> 35) & 0xFF)


def encode(op, sh=0, fd=0, fa=0, fb=0, fc=0):
    return (op & 0xF) | ((sh & 0x7F) << 4) | ((fd & 0xFF) << 11) | \
        ((fa & 0xFF) << 19) | ((fb & 0xFF) << 27) | ((fc & 0xFF) << 35)


def alu(op, sh, a, b, c):
    """One microword's ALU result (a, b, c are signed 48-bit ints)."""
    if op == 0:                                   # MUL: 96-bit product, renorm, saturate
        p = a * b
        p = p >> sh if sh >= 0 else wrap96(p << (-sh))
        return sat(p)
    if op == 2:
        return sat(a + b)
    if op == 3:
        return sat(a - b)
    if op == 4:
        return a if a < b else b
    if op == 5:
        return a if a > b else b
    if op == 7:
        return sat(-a)
    if op == 10:                                  # MSB of |a| (|SMIN| wraps to SMIN -> 47)
        m = (-a) & MASK if a < 0 else a
        return m.bit_length() - 1 if m else 0
    if op == 11:                                  # VSH by signed b[31:0]
        vs = b & 0xFFFFFFFF
        if vs >> 31:
            vs -= 1 << 32
        if vs >= 0:
            return sat(wrap96(a << vs) if vs < 96 else 0)
        return a >> (-vs)
    if op == 1:
        return 1 if a < b else 0
    if op == 12:
        return a if c != 0 else b
    return a                                      # MOV and undefined opcodes


class Core:
    def __init__(self, rom, init=None):
        self.rom = [decode(w) for w in rom]
        self.rf = [0] * 256
        if init:
            for i, v in enumerate(init):
                self.rf[i] = s48(v)
        self.cycles = 0

    def write(self, r, v):
        self.rf[r] = s48(v)

    def read(self, r):
        return self.rf[r]

    def run(self, pc, trace=None):
        rf, rom = self.rf, self.rom
        while True:
            u = rom[pc]
            if u["op"] == 13:
                self.cycles += 1
                return
            v = alu(u["op"], u["sh"], rf[u["fa"]], rf[u["fb"]], rf[u["fc"]])
            if trace is not None:
                trace(pc, u, v)
            rf[u["fd"]] = v
            pc += 1
            self.cycles += 1


def load_hex(path):
    return [int(t, 16) for t in open(path).read().split()]
