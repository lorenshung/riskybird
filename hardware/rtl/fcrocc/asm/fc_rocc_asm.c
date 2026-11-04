/*
 * fc_rocc_asm.c -- FcRoCC microcode assembler + reference interpreter.
 *
 * Recreated 2026-11 (the original source was lost). Assembles the symbolic
 * microcode source (fc_rocc.fca) for the FcRoCC datapath (vsrc/fc_rocc_core.sv)
 * and emits:
 *   fc_rocc_rom.hex   microcode ROM, one 64-bit word per line (16 hex digits)
 *   fc_rocc_init.hex  boot register image, 256 x 48-bit (12 hex digits)
 *   fc_rocc_fw.h      firmware register map + boot image (FCR_INIT[])
 *   fc_rocc_map.svh   the same map for sim/tb_fc_rocc.sv
 *
 * It also carries exec(), a bit-exact C model of fc_rocc_core.sv (one shared
 * signed 48x48 multiply with per-op renorm shift + saturation, saturating
 * 48-bit ADD/SUB/NEG, MIN/MAX/MOV, MSB, VSH, CLT, SEL, HALT), and --replay,
 * which drives exec() with a sensor CSV exactly as tb_fc_rocc.sv drives the RTL
 * and writes a replay CSV in the same format as sim/replay_rtl.csv.
 *
 * Usage:
 *   fc_rocc_asm [-o OUTDIR] [-D NAME=EXPR]... [--lint] SOURCE.fca
 *   fc_rocc_asm [-o OUTDIR] [-D NAME=EXPR]... SOURCE.fca --replay SENSORS.csv OUT.csv
 * OUTDIR defaults to ./build (never the checked-in vsrc/ or firmware dirs).
 * -D NAME=EXPR (or -DNAME=EXPR) defines a .set symbol before the source is read;
 * a `.set NAME = ...` in the source is then skipped, so the source's .set is the
 * default and the command line overrides it.
 *
 * ---- microword (64 bits; 63:43 are zero) ----
 *   [3:0] op  [10:4] sh (signed)  [18:11] fd  [26:19] fa  [34:27] fb  [42:35] fc
 *
 * ---- source language ----
 *   ; or //            comment to end of line
 *   .set   NAME = EXPR                    compile-time symbol
 *   .if EXPR / .else / .endif             conditional assembly (EXPR != 0; nests). Lines
 *                                         in a false branch are not parsed at all, so they
 *                                         emit nothing and claim no registers.
 *   .const NAME rN = EXPR [FMT]           boot-image constant (FMT: q<N> | int; default q24)
 *   .state NAME rN = EXPR [FMT]           boot-image state (program may write it)
 *   .input NAME rN                        input register (exported as FCR_IN_NAME)
 *   .output NAME rN                       output register (exported as FCR_NAME)
 *   .sout NAME rN                         state-out register (FCR_SOUT[] in order)
 *   .reg   NAME rN                        other named register (not exported)
 *   .export NAME VALUE                    #define FCR_NAME VALUE in fc_rocc_fw.h
 *   .program NAME                         start a program here (FCR_NAME_START)
 *   OP DST, SRC...                        instruction
 * EXPR follows C typing: 9.81f is float, 9.81 is double, 4 is int; float op
 * float is evaluated in float, anything with a double in double. Constants
 * are value*2^q rounded to nearest (ties away from zero). A second .const at
 * an already-defined register is allowed when it encodes the same 48 bits
 * (one register, two meanings: e.g. 1.5 in Q30 == 3.0 in Q29).
 *
 * Instructions (dst = ...):
 *   mul.S d,a,b  sat((a*b) >>> S)    (S may be negative: <<< -S)
 *   add/sub d,a,b  saturating        min/max d,a,b      clt d,a,b  (a<b ? 1 : 0)
 *   mov/neg/msb d,a                  vsh d,a,b  (a shifted by signed b: <<sat / >>>)
 *   sel d,c,a,b  (c != 0 ? a : b)    halt
 * Operands: a declared name, a value name, or raw rN. A destination may bind
 * a value name: `name:rN` (pin to rN) or `name:new` (a register no other
 * line uses). A source may be `=EXPR[@FMT]`: an inline constant that reuses an
 * existing .const with the same bits, or else takes an unused register.
 * Checks: a value name whose register was overwritten since it was bound is
 * an error (stale), as is reading a scratch register not yet written in the
 * current program, or writing a .const register.
 */
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define W      48
#define NREG   256
#define ROMMAX 1024
#define SMAX   ((int64_t)((1ULL << (W - 1)) - 1))
#define SMIN   (-SMAX - 1)
#define MASK48 ((1ULL << W) - 1)

enum { OP_MUL = 0, OP_CLT = 1, OP_ADD = 2, OP_SUB = 3, OP_MIN = 4, OP_MAX = 5, OP_MOV = 6,
       OP_NEG = 7, OP_MSB = 10, OP_VSH = 11, OP_SEL = 12, OP_HALT = 13 };

/* ======================= datapath model: exec() ======================= */

static int64_t sat48(__int128 x) { return x > SMAX ? SMAX : x < SMIN ? SMIN : (int64_t)x; }
static __int128 wrap96(__int128 x)
{
    unsigned __int128 u = (unsigned __int128)x & (((unsigned __int128)1 << 96) - 1);
    if (u >> 95) u |= ~(((unsigned __int128)1 << 96) - 1);
    return (__int128)u;
}

typedef struct { uint64_t rom[ROMMAX]; int romd; int64_t rf[NREG]; long cycles; } core_t;

static int64_t alu(int op, int sh, int64_t a, int64_t b, int64_t c)
{
    switch (op) {
    case OP_MUL: {
        __int128 p = (__int128)a * b;
        p = sh >= 0 ? (p >> sh) : wrap96((__int128)((unsigned __int128)p << -sh));
        return sat48(p);
    }
    case OP_ADD: return sat48((__int128)a + b);
    case OP_SUB: return sat48((__int128)a - b);
    case OP_MIN: return a < b ? a : b;
    case OP_MAX: return a > b ? a : b;
    case OP_NEG: return sat48(-(__int128)a);
    case OP_MSB: {
        uint64_t m = (uint64_t)(a < 0 ? -a : a) & MASK48; /* |SMIN| wraps to SMIN -> 47 */
        int i = 0;
        for (int j = 0; j < W; j++) if ((m >> j) & 1) i = j;
        return i;
    }
    case OP_VSH: {
        int32_t vs = (int32_t)(uint32_t)((uint64_t)b & 0xffffffffu);
        if (vs >= 0) {
            if (vs >= 96) return 0;
            return sat48(wrap96((__int128)((unsigned __int128)(__int128)a << vs)));
        }
        int64_t n = -(int64_t)vs;
        return n >= 63 ? (a < 0 ? -1 : 0) : (a >> n);
    }
    case OP_CLT: return a < b ? 1 : 0;
    case OP_SEL: return c != 0 ? a : b;
    default:     return a; /* MOV and the unused opcodes */
    }
}

/* Run one program from `pc` to HALT (one microword per cycle, as the RTL). */
static void exec(core_t *k, int pc)
{
    for (;;) {
        uint64_t w = k->rom[pc];
        int op = w & 15, sh = (int)((w >> 4) & 0x7f);
        if (sh >= 64) sh -= 128;
        int fd = (w >> 11) & 255, fa = (w >> 19) & 255, fb = (w >> 27) & 255, fc = (w >> 35) & 255;
        k->cycles++;
        if (op == OP_HALT) return;
        k->rf[fd] = alu(op, sh, k->rf[fa], k->rf[fb], k->rf[fc]);
        pc++;
    }
}

/* ============================ assembler ============================ */

static const char *g_file;
static int g_line, g_pass;
static void die(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "%s:%d: error: ", g_file, g_line);
    vfprintf(stderr, fmt, ap);
    fputc('\n', stderr);
    va_end(ap);
    exit(1);
}

/* ---- typed values: C int / float / double ---- */
enum { T_INT, T_FLOAT, T_DOUBLE };
typedef struct { int t; double v; } val_t;

typedef struct { char name[64]; val_t v; int cmdline; } setsym_t;
static setsym_t g_set[512];
static int g_nset;

/* conditional assembly: one entry per open .if */
typedef struct { int active, parent_active, line, else_seen; } cond_t;
static cond_t g_cond[32];
static int g_ncond;
static int cond_active(void) { return g_ncond == 0 || g_cond[g_ncond - 1].active; }

static const char *ep; /* expression cursor */
static void skipws(void) { while (*ep == ' ' || *ep == '\t') ep++; }
static val_t expr(void);

static val_t arith(val_t a, char op, val_t b)
{
    val_t r;
    r.t = a.t > b.t ? a.t : b.t;
    if (r.t == T_INT) {
        long long x = (long long)a.v, y = (long long)b.v;
        if (op == '/' && y == 0) die("integer division by zero");
        r.v = (double)(op == '+' ? x + y : op == '-' ? x - y : op == '*' ? x * y : x / y);
        return r;
    }
    double x = a.v, y = b.v, z = op == '+' ? x + y : op == '-' ? x - y : op == '*' ? x * y : x / y;
    r.v = r.t == T_FLOAT ? (double)(float)z : z; /* one IEEE op: exact float rounding via double */
    return r;
}

static val_t primary(void)
{
    val_t r;
    skipws();
    if (*ep == '(') {
        ep++;
        r = expr();
        skipws();
        if (*ep != ')') die("expected ')' in expression");
        ep++;
        return r;
    }
    if (*ep == '-' || *ep == '+') {
        char s = *ep++;
        r = primary();
        if (s == '-') r.v = -r.v;
        return r;
    }
    if (isdigit((unsigned char)*ep) || (*ep == '.' && isdigit((unsigned char)ep[1]))) {
        const char *s = ep;
        int isfp = 0;
        if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
            r.t = T_INT;
            r.v = (double)strtoll(s, (char **)&ep, 16);
            return r;
        }
        while (isdigit((unsigned char)*ep) || *ep == '.' || *ep == 'e' || *ep == 'E' ||
               ((*ep == '-' || *ep == '+') && (ep[-1] == 'e' || ep[-1] == 'E'))) {
            if (*ep == '.' || *ep == 'e' || *ep == 'E') isfp = 1;
            ep++;
        }
        char buf[64];
        size_t n = (size_t)(ep - s);
        if (n >= sizeof buf) die("number too long");
        memcpy(buf, s, n);
        buf[n] = 0;
        if (*ep == 'f' || *ep == 'F') {           /* C float literal: decimal -> float directly */
            ep++;
            r.t = T_FLOAT;
            r.v = (double)strtof(buf, NULL);
        } else if (isfp) {
            r.t = T_DOUBLE;
            r.v = strtod(buf, NULL);
        } else {
            r.t = T_INT;
            r.v = (double)strtoll(buf, NULL, 10);
        }
        return r;
    }
    if (isalpha((unsigned char)*ep) || *ep == '_') {
        const char *s = ep;
        while (isalnum((unsigned char)*ep) || *ep == '_') ep++;
        for (int i = 0; i < g_nset; i++)
            if ((size_t)(ep - s) == strlen(g_set[i].name) && !strncmp(s, g_set[i].name, (size_t)(ep - s)))
                return g_set[i].v;
        die("unknown symbol '%.*s' in expression", (int)(ep - s), s);
    }
    die("bad expression near '%s'", ep);
    return r;
}
static val_t term(void)
{
    val_t a = primary();
    for (;;) {
        skipws();
        if (*ep != '*' && *ep != '/') return a;
        char op = *ep++;
        a = arith(a, op, primary());
    }
}
static val_t expr(void)
{
    val_t a = term();
    for (;;) {
        skipws();
        if (*ep != '+' && *ep != '-') return a;
        char op = *ep++;
        a = arith(a, op, term());
    }
}
static val_t eval(const char *s)
{
    ep = s;
    val_t v = expr();
    skipws();
    if (*ep) die("junk after expression: '%s'", ep);
    return v;
}

/* value -> 48-bit register image. fmt: "q<N>" (value*2^N, nearest, ties away) or "int". */
static int64_t encode_val(val_t v, const char *fmt)
{
    double x;
    if (!strcmp(fmt, "int")) {
        if (v.v != floor(v.v)) die("int constant is not an integer");
        x = v.v;
    } else if (fmt[0] == 'q' && isdigit((unsigned char)fmt[1])) {
        double s = ldexp(v.v, atoi(fmt + 1));
        x = s >= 0 ? floor(s + 0.5) : -floor(-s + 0.5);
    } else {
        die("bad constant format '%s' (want q<N> or int)", fmt);
        x = 0;
    }
    if (x > (double)SMAX || x < (double)SMIN) die("constant out of 48-bit range");
    return (int64_t)x;
}

/* ---- registers and names ---- */
enum { K_NONE, K_CONST, K_STATE, K_INPUT, K_OUTPUT, K_SOUT, K_REG };
static const char *kname[] = {"scratch", "const", "state", "input", "output", "sout", "reg"};
typedef struct { char name[64]; int reg, kind; } decl_t;
static decl_t g_decl[512];
static int g_ndecl;
static int g_regkind[NREG];
static int g_init_set[NREG];
static int64_t g_init[NREG];
static int g_used[NREG];      /* referenced explicitly anywhere (pass 1) */

typedef struct { char name[64]; int reg; long gen; } alias_t; /* value name -> register */
static alias_t g_alias[4096];
static int g_nalias;
static long g_gen, g_reggen[NREG]; /* binding generation that last wrote each register */
static int g_written[NREG];        /* written in the current program */

typedef struct { char name[64]; int pc; } prog_t;
static prog_t g_prog[16];
static int g_nprog;
typedef struct { char name[64]; char value[64]; } export_t;
static export_t g_export[32];
static int g_nexport;

static uint64_t g_rom[ROMMAX];
static int g_pc;

static decl_t *find_decl(const char *n)
{
    for (int i = 0; i < g_ndecl; i++) if (!strcmp(g_decl[i].name, n)) return &g_decl[i];
    return NULL;
}
static alias_t *find_alias(const char *n)
{
    for (int i = g_nalias - 1; i >= 0; i--) if (!strcmp(g_alias[i].name, n)) return &g_alias[i];
    return NULL;
}
static int parse_reg(const char *s)
{
    if (s[0] == 'r' && isdigit((unsigned char)s[1])) {
        char *e;
        long r = strtol(s + 1, &e, 10);
        if (*e) return -1; /* e.g. r0ax: a name, not a register */
        if (r < 0 || r >= NREG) die("bad register '%s'", s);
        return (int)r;
    }
    return -1;
}

static int alloc_unused(void)
{
    for (int r = 0; r < NREG; r++)
        if (!g_used[r] && g_regkind[r] == K_NONE) { g_used[r] = 2; return r; }
    die("out of registers");
    return -1;
}

static void declare(const char *name, int reg, int kind)
{
    decl_t *d = find_decl(name);
    if (d) {
        if (d->reg != reg) die("'%s' redeclared at a different register", name);
        return;
    }
    if (find_alias(name)) die("'%s' already used as a value name", name);
    if (g_regkind[reg] != K_NONE && g_regkind[reg] != kind)
        die("r%d already declared as %s", reg, kname[g_regkind[reg]]);
    if (g_ndecl >= 512) die("too many declarations");
    snprintf(g_decl[g_ndecl].name, 64, "%s", name);
    g_decl[g_ndecl].reg = reg;
    g_decl[g_ndecl].kind = kind;
    g_ndecl++;
    g_regkind[reg] = kind;
    g_used[reg] = 1;
}

static void set_init(int reg, int64_t v)
{
    if (g_init_set[reg] && g_init[reg] != v)
        die("r%d boot value conflict: 0x%llx vs 0x%llx", reg,
            (unsigned long long)(g_init[reg] & MASK48), (unsigned long long)(v & MASK48));
    g_init_set[reg] = 1;
    g_init[reg] = v;
}

/* Inline constant operand "=EXPR[@FMT]": reuse a .const with the same bits, else allocate. */
static int inline_const(const char *s)
{
    char buf[256], fmt[16] = "q24";
    snprintf(buf, sizeof buf, "%s", s + 1);
    char *at = strrchr(buf, '@');
    if (at) { *at = 0; snprintf(fmt, sizeof fmt, "%s", at + 1); }
    int64_t v = encode_val(eval(buf), fmt);
    for (int r = 0; r < NREG; r++)
        if (g_regkind[r] == K_CONST && g_init_set[r] && g_init[r] == v) return r;
    if (g_pass == 1) return 0;
    int r = alloc_unused();
    g_regkind[r] = K_CONST;
    set_init(r, v);
    return r;
}

/* Resolve a source operand to a register, with staleness / init checks (pass 2). */
static int src_reg(const char *s)
{
    int r;
    if (s[0] == '=') return inline_const(s);
    if ((r = parse_reg(s)) >= 0) {
        g_used[r] = 1;
        if (g_pass == 2 && g_regkind[r] == K_NONE && !g_written[r])
            die("reads r%d before it is written in this program", r);
        return r;
    }
    decl_t *d = find_decl(s);
    if (d) {
        if (g_pass == 2 && g_reggen[d->reg] != 0 && g_reggen[d->reg] != -(long)(d - g_decl) - 1)
            die("'%s' (r%d) was overwritten by another name", s, d->reg);
        return d->reg;
    }
    alias_t *a = find_alias(s);
    if (!a) {
        if (g_pass == 1) return 0;
        die("unknown operand '%s'", s);
    }
    if (g_pass == 2) {
        if (g_reggen[a->reg] != a->gen) die("'%s' is stale: r%d was overwritten since it was bound", s, a->reg);
        if (!g_written[a->reg] && g_regkind[a->reg] == K_NONE)
            die("'%s' (r%d) read before written in this program", s, a->reg);
    }
    return a->reg;
}

/* Resolve a destination operand: declared name, raw rN, name:rN or name:new. */
static int dst_reg(const char *s)
{
    int r;
    const char *colon = strchr(s, ':');
    if (colon) {
        char name[64];
        size_t n = (size_t)(colon - s);
        if (n == 0 || n >= sizeof name) die("bad value name in '%s'", s);
        memcpy(name, s, n);
        name[n] = 0;
        if (find_decl(name)) die("'%s' is a declared register; write it without ':'", name);
        int isnew = !strcmp(colon + 1, "new");
        if (isnew) {
            /* allocated in pass 2, after pass 1 has recorded every explicitly used register */
            r = g_pass == 2 ? alloc_unused() : 0;
        } else {
            r = parse_reg(colon + 1);
            if (r < 0) die("expected rN or 'new' after ':' in '%s'", s);
            g_used[r] = 1;
        }
        if (!(isnew && g_pass == 1) && g_regkind[r] != K_NONE)
            die("value '%s' pinned to declared %s register r%d", name, kname[g_regkind[r]], r);
        if (g_nalias >= 4096) die("too many value names");
        snprintf(g_alias[g_nalias].name, 64, "%s", name);
        g_alias[g_nalias].reg = r;
        g_alias[g_nalias].gen = ++g_gen;
        g_nalias++;
        g_reggen[r] = g_gen;
        return r;
    }
    if ((r = parse_reg(s)) >= 0) {
        g_used[r] = 1;
        g_reggen[r] = ++g_gen; /* anonymous write: stales every name bound to r */
    } else {
        decl_t *d = find_decl(s);
        if (!d) die("destination '%s' needs a register: write %s:rN or %s:new", s, s, s);
        r = d->reg;
        g_reggen[r] = -(long)(d - g_decl) - 1;
    }
    if (g_pass == 2 && g_regkind[r] == K_CONST) die("writes constant register r%d", r);
    return r;
}

static void emit(int op, int sh, int fd, int fa, int fb, int fc)
{
    if (g_pc >= ROMMAX) die("ROM overflow (%d words)", ROMMAX);
    if (sh < -64 || sh > 63) die("shift %d out of range", sh);
    g_rom[g_pc++] = (uint64_t)(op & 15) | ((uint64_t)(sh & 0x7f) << 4) | ((uint64_t)fd << 11) |
                    ((uint64_t)fa << 19) | ((uint64_t)fb << 27) | ((uint64_t)fc << 35);
    if (op != OP_HALT) g_written[fd] = 1;
}

/* split "a, b, c" into trimmed fields */
static int split_ops(char *s, char **f, int max)
{
    int n = 0;
    while (*s) {
        while (*s == ' ' || *s == '\t') s++;
        if (!*s) break;
        if (n >= max) die("too many operands");
        f[n++] = s;
        char *c = strchr(s, ',');
        char *e = c ? c : s + strlen(s);
        char *t = e;
        while (t > s && (t[-1] == ' ' || t[-1] == '\t')) t--;
        *t = 0;
        if (!c) break;
        s = c + 1;
    }
    return n;
}

static void finish_program(void)
{
    if (g_nprog && g_pass == 2) {
        if (g_pc == g_prog[g_nprog - 1].pc || (g_rom[g_pc - 1] & 15) != OP_HALT)
            die("program %s does not end with halt", g_prog[g_nprog - 1].name);
    }
}

static void do_line(char *line)
{
    char *c;
    if ((c = strchr(line, ';'))) *c = 0;
    if ((c = strstr(line, "//"))) *c = 0;
    char *s = line;
    while (isspace((unsigned char)*s)) s++;
    char *e = s + strlen(s);
    while (e > s && isspace((unsigned char)e[-1])) *--e = 0;
    if (!*s) return;

    char kw[32];
    int n = 0;
    while (s[n] && !isspace((unsigned char)s[n]) && n < 31) { kw[n] = s[n]; n++; }
    kw[n] = 0;
    char *rest = s + n;
    while (isspace((unsigned char)*rest)) rest++;

    /* conditional assembly, handled before anything else so a false branch is never parsed */
    if (!strcmp(kw, ".if")) {
        if (g_ncond >= (int)(sizeof g_cond / sizeof g_cond[0])) die(".if nested too deep");
        int parent = cond_active();
        if (parent && !*rest) die(".if EXPR");
        g_cond[g_ncond].parent_active = parent;
        g_cond[g_ncond].active = parent && eval(rest).v != 0.0;
        g_cond[g_ncond].line = g_line;
        g_cond[g_ncond].else_seen = 0;
        g_ncond++;
        return;
    }
    if (!strcmp(kw, ".else")) {
        if (!g_ncond) die(".else without .if");
        if (*rest) die(".else takes no operands");
        cond_t *c = &g_cond[g_ncond - 1];
        if (c->else_seen) die("second .else for the .if at line %d", c->line);
        c->else_seen = 1;
        c->active = c->parent_active && !c->active;
        return;
    }
    if (!strcmp(kw, ".endif")) {
        if (!g_ncond) die(".endif without .if");
        if (*rest) die(".endif takes no operands");
        g_ncond--;
        return;
    }
    if (!cond_active()) return;

    if (kw[0] == '.') {
        char name[64], regs[16];
        if (!strcmp(kw, ".set")) {
            char *eq = strchr(rest, '=');
            if (!eq || sscanf(rest, "%63[A-Za-z0-9_]", name) != 1) die(".set NAME = EXPR");
            if (g_pass == 2) return;
            for (int i = 0; i < g_nset; i++)
                if (!strcmp(g_set[i].name, name)) {
                    if (g_set[i].cmdline) { g_set[i].cmdline = 2; return; } /* -D overrides the default */
                    die("'%s' already set", name);
                }
            snprintf(g_set[g_nset].name, 64, "%s", name);
            g_set[g_nset].v = eval(eq + 1);
            g_nset++;
            return;
        }
        if (!strcmp(kw, ".const") || !strcmp(kw, ".state")) {
            char *eq = strchr(rest, '=');
            if (!eq || sscanf(rest, "%63s %15s", name, regs) != 2) die("%s NAME rN = EXPR [FMT]", kw);
            int r = parse_reg(regs);
            if (r < 0) die("bad register '%s'", regs);
            char ex[256], fmt[16] = "q24";
            snprintf(ex, sizeof ex, "%s", eq + 1);
            /* optional trailing format word */
            char *t = ex + strlen(ex);
            while (t > ex && isspace((unsigned char)t[-1])) *--t = 0;
            char *sp = strrchr(ex, ' ');
            if (sp && (sp[1] == 'q' || !strcmp(sp + 1, "int")) && (isdigit((unsigned char)sp[2]) || sp[1] == 'i')) {
                snprintf(fmt, sizeof fmt, "%s", sp + 1);
                *sp = 0;
            }
            if (g_pass == 1) {
                declare(name, r, kw[1] == 'c' ? K_CONST : K_STATE);
                set_init(r, encode_val(eval(ex), fmt));
            }
            return;
        }
        int kind = !strcmp(kw, ".input") ? K_INPUT : !strcmp(kw, ".output") ? K_OUTPUT :
                   !strcmp(kw, ".sout") ? K_SOUT : !strcmp(kw, ".reg") ? K_REG : K_NONE;
        if (kind != K_NONE) {
            if (sscanf(rest, "%63s %15s", name, regs) != 2) die("%s NAME rN", kw);
            int r = parse_reg(regs);
            if (r < 0) die("bad register '%s'", regs);
            if (g_pass == 1) declare(name, r, kind);
            return;
        }
        if (!strcmp(kw, ".export")) {
            char v[64];
            if (sscanf(rest, "%63s %63s", name, v) != 2) die(".export NAME VALUE");
            if (g_pass == 1) {
                snprintf(g_export[g_nexport].name, 64, "%s", name);
                snprintf(g_export[g_nexport].value, 64, "%s", v);
                g_nexport++;
            }
            return;
        }
        if (!strcmp(kw, ".program")) {
            if (sscanf(rest, "%63s", name) != 1) die(".program NAME");
            finish_program();
            snprintf(g_prog[g_nprog].name, 64, "%s", name);
            g_prog[g_nprog].pc = g_pc;
            g_nprog++;
            memset(g_written, 0, sizeof g_written);
            return;
        }
        die("unknown directive '%s'", kw);
    }

    if (!g_nprog) die("instruction before .program");
    char *f[4];
    int nf = split_ops(rest, f, 4);
    int op = -1, sh = 0, nsrc = 0;
    if (!strncmp(kw, "mul.", 4)) {
        char *t;
        op = OP_MUL;
        sh = (int)strtol(kw + 4, &t, 10);
        if (*t || t == kw + 4) die("bad shift in '%s'", kw);
        nsrc = 2;
    } else {
        static const struct { const char *m; int op, n; } tab[] = {
            {"add", OP_ADD, 2}, {"sub", OP_SUB, 2}, {"min", OP_MIN, 2}, {"max", OP_MAX, 2},
            {"clt", OP_CLT, 2}, {"vsh", OP_VSH, 2}, {"mov", OP_MOV, 1}, {"neg", OP_NEG, 1},
            {"msb", OP_MSB, 1}, {"sel", OP_SEL, 3}, {"halt", OP_HALT, -1}};
        for (size_t i = 0; i < sizeof tab / sizeof tab[0]; i++)
            if (!strcmp(kw, tab[i].m)) { op = tab[i].op; nsrc = tab[i].n; }
        if (op < 0) die("unknown mnemonic '%s'", kw);
    }
    if (op == OP_HALT) {
        if (nf) die("halt takes no operands");
        emit(OP_HALT, 0, 0, 0, 0, 0);
        return;
    }
    if (nf != nsrc + 1) die("'%s' wants %d operands", kw, nsrc + 1);
    /* sources are read before the destination is written (same cycle) */
    int s0 = src_reg(f[1]);
    int s1 = nsrc > 1 ? src_reg(f[2]) : 0;
    int s2 = nsrc > 2 ? src_reg(f[3]) : 0;
    int d = dst_reg(f[0]);
    if (op == OP_SEL) emit(op, 0, d, s1, s2, s0);  /* sel d, c, a, b */
    else emit(op, sh, d, s0, s1, 0);
}

static void assemble(const char *path, int pass)
{
    FILE *fp = fopen(path, "r");
    if (!fp) { perror(path); exit(1); }
    char line[1024];
    g_pass = pass;
    g_line = 0;
    g_pc = 0;
    g_nprog = 0;
    g_nalias = 0;
    g_gen = 0;
    memset(g_reggen, 0, sizeof g_reggen);
    memset(g_written, 0, sizeof g_written);
    g_ncond = 0;
    while (fgets(line, sizeof line, fp)) {
        g_line++;
        do_line(line);
    }
    fclose(fp);
    if (g_ncond) { g_line = g_cond[g_ncond - 1].line; die(".if without .endif"); }
    finish_program();
}

/* ============================== outputs ============================== */

static FILE *openout(const char *dir, const char *name)
{
    char p[1024];
    snprintf(p, sizeof p, "%s/%s", dir, name);
    FILE *fp = fopen(p, "w");
    if (!fp) { perror(p); exit(1); }
    return fp;
}

static void list_kind(FILE *fp, int kind, const char *prefix, const char *sep)
{
    int first = 1;
    for (int i = 0; i < g_ndecl; i++)
        if (g_decl[i].kind == kind) {
            fprintf(fp, "%s%s%s=%d", first ? "" : sep, prefix, g_decl[i].name, g_decl[i].reg);
            first = 0;
        }
}
static void list_sout(FILE *fp)
{
    int first = 1;
    for (int i = 0; i < g_ndecl; i++)
        if (g_decl[i].kind == K_SOUT) { fprintf(fp, "%s%d", first ? "" : ",", g_decl[i].reg); first = 0; }
}
static int count_kind(int kind)
{
    int n = 0;
    for (int i = 0; i < g_ndecl; i++) n += g_decl[i].kind == kind;
    return n;
}

static void write_outputs(const char *dir)
{
    FILE *fp = openout(dir, "fc_rocc_rom.hex");
    for (int i = 0; i < g_pc; i++) fprintf(fp, "%016llx\n", (unsigned long long)g_rom[i]);
    fclose(fp);

    fp = openout(dir, "fc_rocc_init.hex");
    for (int r = 0; r < NREG; r++) fprintf(fp, "%012llx\n", (unsigned long long)(g_init[r] & MASK48));
    fclose(fp);

    fp = openout(dir, "fc_rocc_fw.h");
    fprintf(fp, "/* auto-generated by fc_rocc_asm.c -- FC RoCC firmware map + boot image */\n");
    fprintf(fp, "#ifndef FC_ROCC_FW_H\n#define FC_ROCC_FW_H\n#include <stdint.h>\n");
    fprintf(fp, "#define FCR_NREG %d\n#define FCR_ROMD %d\n", NREG, g_pc);
    for (int i = 0; i < g_nprog; i++) fprintf(fp, "#define FCR_%s_START %d\n", g_prog[i].name, g_prog[i].pc);
    for (int i = 0; i < g_nexport; i++) fprintf(fp, "#define FCR_%s %s\n", g_export[i].name, g_export[i].value);
    fprintf(fp, "/* input regs (write Q24, flags Q0): */\n");
    fprintf(fp, "enum{");
    list_kind(fp, K_INPUT, "FCR_IN_", ",");
    fprintf(fp, "};\nenum{");
    list_kind(fp, K_OUTPUT, "FCR_", ",");
    fprintf(fp, "};\nstatic const int FCR_SOUT[%d]={", count_kind(K_SOUT));
    list_sout(fp);
    fprintf(fp, "};\n/* boot image: CFG-write reg r = FCR_INIT[r] at startup (consts+state) */\n");
    fprintf(fp, "static const uint64_t FCR_INIT[%d]={", NREG);
    for (int r = 0; r < NREG; r++)
        fprintf(fp, "%s0x%llxULL", r ? "," : "", (unsigned long long)(g_init[r] & MASK48));
    fprintf(fp, "};\n#endif\n");
    fclose(fp);

    fp = openout(dir, "fc_rocc_map.svh");
    fprintf(fp, "// auto-generated by fc_rocc_asm.c\nlocalparam int ROMD=%d", g_pc);
    for (int i = 0; i < g_nprog; i++) fprintf(fp, ", %s_START=%d", g_prog[i].name, g_prog[i].pc);
    fprintf(fp, ", NREGP=%d;\nlocalparam int ", NREG);
    list_kind(fp, K_INPUT, "IN_", ",");
    fprintf(fp, ";\nlocalparam int ");
    list_kind(fp, K_OUTPUT, "", ",");
    fprintf(fp, ";\nlocalparam int SOUT [0:%d] = '{", count_kind(K_SOUT) - 1);
    list_sout(fp);
    fprintf(fp, "};\n");
    fclose(fp);
}

/* ============================== replay ============================== */

static int reg_of(const char *name)
{
    decl_t *d = find_decl(name);
    if (!d) { fprintf(stderr, "replay: source does not declare '%s'\n", name); exit(1); }
    return d->reg;
}
static int prog_pc(const char *name)
{
    for (int i = 0; i < g_nprog; i++) if (!strcmp(g_prog[i].name, name)) return g_prog[i].pc;
    fprintf(stderr, "replay: no program '%s'\n", name);
    exit(1);
}
static int export_int(const char *name)
{
    for (int i = 0; i < g_nexport; i++) if (!strcmp(g_export[i].name, name)) return atoi(g_export[i].value);
    fprintf(stderr, "replay: no export '%s'\n", name);
    exit(1);
}
/* tb_fc_rocc.sv toq(): $rtoi(v*2^q + (v>=0 ? 0.5 : -0.5)) */
static int64_t toq(double v, int q) { return (int64_t)(v * ldexp(1.0, q) + (v >= 0.0 ? 0.5 : -0.5)); }

static void replay(const char *in, const char *out)
{
    core_t k;
    memset(&k, 0, sizeof k);
    memcpy(k.rom, g_rom, sizeof g_rom);
    k.romd = g_pc;
    for (int r = 0; r < NREG; r++) k.rf[r] = g_init[r];
    const int QA = export_int("QA"), QP = export_int("Q_POS"), QV = export_int("Q_VEL");
    static const char *innames[] = {"AX", "AY", "AZ", "GX", "GY", "GZ", "F0", "F1", "H", "DT"};
    int inreg[10], sout[12], ns = 0, u[4] = {reg_of("U0"), reg_of("U1"), reg_of("U2"), reg_of("U3")};
    for (int i = 0; i < 10; i++) inreg[i] = reg_of(innames[i]);
    for (int i = 0; i < g_ndecl && ns < 12; i++) if (g_decl[i].kind == K_SOUT) sout[ns++] = g_decl[i].reg;
    int rFV = reg_of("FV"), rTV = reg_of("TV"), rDT = reg_of("DT");
    int rDH = reg_of("DESH"), rV1 = reg_of("DESV1"), rV2 = reg_of("DESV2"), rYT = reg_of("YAWT");
    int est = prog_pc("EST"), ctl = prog_pc("CTRL");

    FILE *fi = fopen(in, "r"), *fo = fopen(out, "w");
    if (!fi || !fo) { perror("replay"); exit(1); }
    char line[1024];
    if (!fgets(line, sizeof line, fi)) exit(1);
    fprintf(fo, "# impl=fc_rocc_asm.c exec() interpreter (assembled ROM)\n");
    fprintf(fo, "step,ex,ey,ez,er1,er2,er3,evx,evy,evz,u0,u1,u2,u3\n");
    int step = 0;
    long ce = 0, cc = 0;
    while (fgets(line, sizeof line, fi)) {
        double d[13];
        if (sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &d[0], &d[1], &d[2], &d[3],
                   &d[4], &d[5], &d[6], &d[7], &d[8], &d[9], &d[10], &d[11], &d[12]) != 13)
            continue;
        /* columns: dt,ax,ay,az,gx,gy,gz,fx,fy,fvalid,height,tofvalid,zsp */
        double inv[10] = {d[1], d[2], d[3], d[4], d[5], d[6], d[7], d[8], d[10], d[0]};
        for (int i = 0; i < 10; i++) k.rf[inreg[i]] = toq(inv[i], QA);
        k.rf[rFV] = d[9] > 0.5;
        k.rf[rTV] = d[11] > 0.5;
        long c0 = k.cycles;
        exec(&k, est);
        ce = k.cycles - c0;
        k.rf[rDH] = toq(d[12], QA);
        k.rf[rV1] = k.rf[rV2] = k.rf[rYT] = 0;
        k.rf[rDT] = toq(d[0], QA);
        c0 = k.cycles;
        exec(&k, ctl);
        cc = k.cycles - c0;
        double o[13];
        for (int i = 0; i < 9; i++) o[i] = (double)k.rf[sout[i]] / ldexp(1.0, i < 3 ? QP : i < 6 ? QA : QV);
        for (int i = 0; i < 4; i++) o[9 + i] = (double)k.rf[u[i]] / ldexp(1.0, QA);
        fprintf(fo, "%d", step);
        for (int i = 0; i < 13; i++) fprintf(fo, ",%.5f", o[i]);
        fprintf(fo, "\n");
        step++;
    }
    fclose(fi);
    fclose(fo);
    fprintf(stderr, "replay: %d steps -> %s (EST %ld cycles, CTRL %ld cycles per run)\n", step, out, ce, cc);
}

/* -D NAME=EXPR: a .set symbol that the source's own .set of NAME does not override */
static void cmdline_define(const char *def)
{
    char name[64];
    const char *eq = strchr(def, '=');
    if (!eq || sscanf(def, "%63[A-Za-z0-9_]", name) != 1 || strlen(name) != (size_t)(eq - def)) {
        fprintf(stderr, "-D wants NAME=EXPR, got '%s'\n", def);
        exit(2);
    }
    for (int i = 0; i < g_nset; i++)
        if (!strcmp(g_set[i].name, name)) { fprintf(stderr, "-D %s given twice\n", name); exit(2); }
    g_file = "<command line>";
    snprintf(g_set[g_nset].name, 64, "%s", name);
    g_set[g_nset].v = eval(eq + 1);
    g_set[g_nset].cmdline = 1;
    g_nset++;
}

int main(int argc, char **argv)
{
    const char *outdir = "build", *src = NULL, *rin = NULL, *rout = NULL;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-o") && i + 1 < argc) outdir = argv[++i];
        else if (!strcmp(argv[i], "-D") && i + 1 < argc) cmdline_define(argv[++i]);
        else if (!strncmp(argv[i], "-D", 2) && argv[i][2]) cmdline_define(argv[i] + 2);
        else if (!strcmp(argv[i], "--replay") && i + 2 < argc) { rin = argv[++i]; rout = argv[++i]; }
        else if (!strcmp(argv[i], "--lint")) ;  /* checks always run; kept for scripts */
        else if (argv[i][0] == '-') { fprintf(stderr, "usage: %s [-o DIR] [-D NAME=EXPR]... SRC.fca [--replay IN.csv OUT.csv]\n", argv[0]); return 2; }
        else src = argv[i];
    }
    if (!src) { fprintf(stderr, "usage: %s [-o DIR] [-D NAME=EXPR]... SRC.fca [--replay IN.csv OUT.csv]\n", argv[0]); return 2; }
    g_file = src;
    assemble(src, 1);
    for (int i = 0; i < g_nset; i++)
        if (g_set[i].cmdline == 1)
            fprintf(stderr, "fc_rocc_asm: warning: -D %s matches no .set in %s\n", g_set[i].name, src);
    assemble(src, 2);
    char cmd[1200];
    snprintf(cmd, sizeof cmd, "mkdir -p '%s'", outdir);
    if (system(cmd) != 0) { fprintf(stderr, "cannot create %s\n", outdir); return 1; }
    write_outputs(outdir);
    fprintf(stderr, "fc_rocc_asm: %d microwords", g_pc);
    for (int i = 0; i < g_nprog; i++) {
        int end = i + 1 < g_nprog ? g_prog[i + 1].pc : g_pc;
        fprintf(stderr, ", %s @%d (%d words)", g_prog[i].name, g_prog[i].pc, end - g_prog[i].pc);
    }
    fprintf(stderr, " -> %s/\n", outdir);
    if (rin) replay(rin, rout);
    return 0;
}
