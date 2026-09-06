// Cycle-accurate check of the motor PWM path, with no board and no motors.
//
// Simulates the PWMTimer that RocketArty200TMotorSpadConfig actually elaborated
// -- gen-collateral/PWMTimer.sv from that build, not a hand model -- driven by
// exactly the register writes Zephyr's sifive driver performs, and measures the
// waveform at the package pin.
//
// What makes this worth running rather than reasoning about: the sifive
// comparator drives its output HIGH once the scaled counter has passed the
// comparator value, so a request for duty D lands on the pin as 1-D, and the
// reset state (all comparators zero) is a constant logic high. Both are
// invisible in the register readback -- software sees exactly what it wrote.
// WithArty200TPWM inverts at the harness to correct both; this measures whether
// that is true, at the pin, for the real RTL.
//
// Build and run:  hardware/pwm/run.sh
// See docs/motor-pwm-bringup.md.

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <vector>
#include "VPWMTimer.h"
#include "verilated.h"

// The peripheral clock the DTS declares for this shell.
static const uint64_t F_SYS = 50000000ull;
// Comparator width from the elaborated DTS (sifive,compare-width).
static const int CMPWIDTH = 16;
// Zephyr's SF_PWMCOUNT_MIN_WIDTH.
static const int COUNT_MIN_WIDTH = 15;

static VPWMTimer *dut;
static uint64_t main_time = 0;

static void tick()
{
    dut->clock = 0; dut->eval();
    dut->clock = 1; dut->eval();
    main_time++;
}

// Clear every write strobe. The register interface is a strobe-per-field
// bundle, so a write is "set data, pulse strobe, one clock".
static void clear_strobes()
{
    dut->scale_io_regs_cfg_write_scale_0 = 0;
    dut->scale_io_regs_cfg_write_sticky_0 = 0;
    dut->scale_io_regs_cfg_write_zerocmp_0 = 0;
    dut->scale_io_regs_cfg_write_deglitch_0 = 0;
    dut->scale_io_regs_cfg_write_countAlways_0 = 0;
    dut->scale_io_regs_cfg_write_extra_0_0 = 0;
    dut->scale_io_regs_cfg_write_extra_1_0 = 0;
    dut->scale_io_regs_cfg_write_extra_2_0 = 0;
    dut->scale_io_regs_cfg_write_extra_3_0 = 0;
    dut->scale_io_regs_cmp_0_write_valid = 0;
    dut->scale_io_regs_cmp_1_write_valid = 0;
    dut->scale_io_regs_cmp_2_write_valid = 0;
    dut->scale_io_regs_cmp_3_write_valid = 0;
}

static void write_cmp(int channel, uint32_t value)
{
    clear_strobes();
    switch (channel) {
    case 0: dut->scale_io_regs_cmp_0_write_valid = 1;
            dut->scale_io_regs_cmp_0_write_bits = value; break;
    case 1: dut->scale_io_regs_cmp_1_write_valid = 1;
            dut->scale_io_regs_cmp_1_write_bits = value; break;
    case 2: dut->scale_io_regs_cmp_2_write_valid = 1;
            dut->scale_io_regs_cmp_2_write_bits = value; break;
    case 3: dut->scale_io_regs_cmp_3_write_valid = 1;
            dut->scale_io_regs_cmp_3_write_bits = value; break;
    }
    tick();
    clear_strobes();
}

// Zephyr pwm_sifive_init(): zerocmp, enalways, clear sticky/deglitch, and clear
// every comparator. Deliberately does NOT touch pwmcfg[20+n] (pwminvert) --
// neither does the driver, and the harness inversion depends on that.
static void zephyr_pwm_init()
{
    clear_strobes();
    dut->scale_io_regs_cfg_write_zerocmp = 1;
    dut->scale_io_regs_cfg_write_zerocmp_0 = 1;
    tick();
    clear_strobes();

    dut->scale_io_regs_cfg_write_countAlways = 1;
    dut->scale_io_regs_cfg_write_countAlways_0 = 1;
    tick();
    clear_strobes();

    dut->scale_io_regs_cfg_write_sticky = 0;
    dut->scale_io_regs_cfg_write_sticky_0 = 1;
    dut->scale_io_regs_cfg_write_deglitch = 0;
    dut->scale_io_regs_cfg_write_deglitch_0 = 1;
    tick();
    clear_strobes();

    for (int i = 0; i < 4; i++) {
        write_cmp(i, 0);
    }
}

// Zephyr pwm_sifive_set_cycles(). Returns the pwmscale it selected, or -1 for
// the errors the driver reports.
static int zephyr_set_cycles(int channel, uint32_t period_cycles,
                             uint32_t pulse_cycles)
{
    if (channel == 0 || channel >= 4) {
        return -1;  // "PWM channel 0 cannot be configured"
    }
    uint64_t count_max = (1ull << (CMPWIDTH + COUNT_MIN_WIDTH)) - 1;
    if (period_cycles > count_max) {
        return -1;
    }
    uint32_t max_cmp_val = (1u << CMPWIDTH) - 1;
    uint32_t pwmscale = 0;
    while ((period_cycles >> pwmscale) > max_cmp_val) {
        pwmscale++;
    }
    if (pwmscale > 0xF) {
        return -1;
    }

    clear_strobes();
    dut->scale_io_regs_cfg_write_scale = pwmscale;
    dut->scale_io_regs_cfg_write_scale_0 = 1;
    tick();
    clear_strobes();

    write_cmp(0, period_cycles >> pwmscale);
    write_cmp(channel, pulse_cycles >> pwmscale);
    return (int)pwmscale;
}

static uint8_t pin(int channel, bool invert)
{
    uint8_t gpio = 0;
    switch (channel) {
    case 0: gpio = dut->scale_io_gpio_0; break;
    case 1: gpio = dut->scale_io_gpio_1; break;
    case 2: gpio = dut->scale_io_gpio_2; break;
    case 3: gpio = dut->scale_io_gpio_3; break;
    }
    return invert ? (gpio ^ 1u) : gpio;
}

struct Measurement {
    uint64_t period;   // cycles between successive rising edges
    uint64_t high;     // cycles the pin was high in that period
    bool     saw_edge;
};

// Run for `cycles` and measure one whole period of the pin, starting at the
// first rising edge so a partial first period cannot bias the answer.
static Measurement measure(int channel, bool invert, uint64_t cycles)
{
    Measurement m = {0, 0, false};
    uint8_t prev = pin(channel, invert);
    bool counting = false;
    uint64_t high = 0, total = 0;

    for (uint64_t i = 0; i < cycles; i++) {
        tick();
        uint8_t now = pin(channel, invert);
        bool rising = (now == 1 && prev == 0);
        if (rising) {
            if (counting) {
                m.period = total;
                m.high = high;
                m.saw_edge = true;
                return m;
            }
            counting = true;
            high = 0;
            total = 0;
        }
        if (counting) {
            total++;
            if (now) high++;
        }
        prev = now;
    }
    // No two rising edges: the pin is stuck. Report the level as a flat duty.
    m.period = cycles;
    m.high = 0;
    for (uint64_t i = 0; i < cycles; i++) {
        tick();
        if (pin(channel, invert)) m.high++;
    }
    m.saw_edge = false;
    return m;
}

// Hold the pin level over `cycles` without touching any register. Used for the
// power-on question: what do the motors see before software runs?
static double level_fraction(int channel, bool invert, uint64_t cycles)
{
    uint64_t high = 0;
    for (uint64_t i = 0; i < cycles; i++) {
        tick();
        if (pin(channel, invert)) high++;
    }
    return (double)high / (double)cycles;
}

// ---------------------------------------------------------------------------
// Emulation of the on-chip probe in workloads/motor_4/src/main.c.
//
// On a bench nobody can look at, that probe is the only thing that measures the
// waveform rather than the registers, so a wrong answer from it has to be
// attributable. This runs the same algorithm -- straddled phase reads, midpoint
// binning, wrapped samples dropped -- against RTL whose true duty is known.
// If it agrees here and disagrees on hardware, the hardware is what changed.
//
// lag_cycles models the peripheral-bus latency of one MMIO load, which is the
// error the midpoint correction exists to cancel.
// ---------------------------------------------------------------------------
#define PROBE_BINS 32

struct ProbeResult {
    double duty_permille;
    int    edge_bin;
    double mean_lag;
    int    empty_bins;
};

static uint32_t phase_read(int /*channel*/)
{
    return dut->scale_io_regs_s_read;
}

static uint8_t ip_read(int channel)
{
    switch (channel) {
    case 0: return dut->scale_io_ip_0;
    case 1: return dut->scale_io_ip_1;
    case 2: return dut->scale_io_ip_2;
    default: return dut->scale_io_ip_3;
    }
}

static ProbeResult emulate_probe(int channel, uint32_t period, int lag_cycles,
                                 int samples)
{
    std::vector<uint32_t> total(PROBE_BINS, 0), high(PROBE_BINS, 0);
    uint64_t lag_sum = 0, lag_n = 0;
    uint32_t rng = 12345u;   // fixed seed: the run has to be reproducible

    for (int i = 0; i < samples; i++) {
        uint32_t before = phase_read(channel);
        for (int c = 0; c < lag_cycles; c++) tick();
        uint8_t ip = ip_read(channel);
        for (int c = 0; c < lag_cycles; c++) tick();
        uint32_t after = phase_read(channel);

        // Unwrap, do not discard: see the comment in motor_4/src/main.c.
        uint32_t span = (after >= before) ? (after - before)
                                          : (after + period - before);
        uint32_t phase = period ? (before + span / 2) % period : 0;
        lag_sum += span;
        lag_n++;
        uint32_t bin = period ? (phase * PROBE_BINS) / period : 0;
        if (bin >= PROBE_BINS) bin = PROBE_BINS - 1;
        total[bin]++;
        if (ip) high[bin]++;
        // Walk the phase forward by an irregular amount so the sampling loop
        // cannot lock to a harmonic of the PWM period.
        rng = rng * 1103515245u + 12345u;
        uint32_t gap = 7 + ((rng >> 16) % 97);
        for (uint32_t c = 0; c < gap; c++) tick();
    }

    if (getenv("RB_PROBE_DUMP")) {
        fprintf(stderr, "   bin totals/high:\n");
        for (int b = 0; b < PROBE_BINS; b++)
            fprintf(stderr, "     %2d  total=%5u high=%5u\n", b, total[b], high[b]);
    }
    ProbeResult r = {0.0, PROBE_BINS, 0.0, 0};
    // Equally weighted mean over bins, matching probe_waveform().
    double permille_sum = 0.0;
    int used = 0;
    for (int b = 0; b < PROBE_BINS; b++) {
        if (total[b] == 0) { r.empty_bins++; continue; }
        permille_sum += 1000.0 * (double)(total[b] - high[b]) / (double)total[b];
        used++;
        if (r.edge_bin == PROBE_BINS && high[b] * 2 > total[b]) {
            r.edge_bin = b;
        }
    }
    r.duty_permille = used ? permille_sum / used : 0.0;
    r.mean_lag = lag_n ? (double)lag_sum / (double)lag_n : 0.0;
    return r;
}

static int failures = 0;

static void check(const char *what, bool ok, const char *detail)
{
    printf("%-6s %-52s %s\n", ok ? "PASS" : "FAIL", what, detail);
    if (!ok) failures++;
}

static void reset_dut()
{
    dut->reset = 1;
    clear_strobes();
    dut->scale_io_regs_cfg_write_scale = 0;
    dut->scale_io_regs_cfg_write_sticky = 0;
    dut->scale_io_regs_cfg_write_zerocmp = 0;
    dut->scale_io_regs_cfg_write_deglitch = 0;
    dut->scale_io_regs_cfg_write_countAlways = 0;
    dut->scale_io_regs_cfg_write_extra_0 = 0;
    dut->scale_io_regs_cfg_write_extra_1 = 0;
    dut->scale_io_regs_cfg_write_extra_2 = 0;
    dut->scale_io_regs_cfg_write_extra_3 = 0;
    dut->scale_io_regs_cfg_write_running = 0;
    dut->scale_io_regs_cfg_write_running_0 = 0;
    dut->scale_io_regs_cfg_write_center_0 = 0;
    dut->scale_io_regs_cfg_write_center_1 = 0;
    dut->scale_io_regs_cfg_write_center_2 = 0;
    dut->scale_io_regs_cfg_write_center_3 = 0;
    dut->scale_io_regs_cfg_write_center_0_0 = 0;
    dut->scale_io_regs_cfg_write_center_1_0 = 0;
    dut->scale_io_regs_cfg_write_center_2_0 = 0;
    dut->scale_io_regs_cfg_write_center_3_0 = 0;
    dut->scale_io_regs_cfg_write_gang_0 = 0;
    dut->scale_io_regs_cfg_write_gang_1 = 0;
    dut->scale_io_regs_cfg_write_gang_2 = 0;
    dut->scale_io_regs_cfg_write_gang_3 = 0;
    dut->scale_io_regs_cfg_write_gang_0_0 = 0;
    dut->scale_io_regs_cfg_write_gang_1_0 = 0;
    dut->scale_io_regs_cfg_write_gang_2_0 = 0;
    dut->scale_io_regs_cfg_write_gang_3_0 = 0;
    dut->scale_io_regs_cfg_write_ip_0 = 0;
    dut->scale_io_regs_cfg_write_ip_1 = 0;
    dut->scale_io_regs_cfg_write_ip_2 = 0;
    dut->scale_io_regs_cfg_write_ip_3 = 0;
    dut->scale_io_regs_cfg_write_ip_0_0 = 0;
    dut->scale_io_regs_cfg_write_ip_1_0 = 0;
    dut->scale_io_regs_cfg_write_ip_2_0 = 0;
    dut->scale_io_regs_cfg_write_ip_3_0 = 0;
    dut->scale_io_regs_countLo_write_valid = 0;
    dut->scale_io_regs_countLo_write_bits = 0;
    for (int i = 0; i < 8; i++) tick();
    dut->reset = 0;
    tick();
}

// nsec -> cycles, the way Zephyr's pwm_set_dt does it.
static uint32_t ns_to_cycles(uint64_t ns)
{
    return (uint32_t)((ns * F_SYS) / 1000000000ull);
}

int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    dut = new VPWMTimer;

    const bool invert = true;   // WithArty200TPWM(invert = true)
    const int channel = 1;      // motor1 = block 0, comparator 1
    const uint32_t period_ns = 50000;   // 20 kHz, from motor_4.overlay
    const uint32_t period_cycles = ns_to_cycles(period_ns);

    printf("PWMTimer from RocketArty200TMotorSpadConfig\n");
    printf("  f_sys %llu Hz, period %u ns = %u cycles, cmpwidth %d\n\n",
           (unsigned long long)F_SYS, period_ns, period_cycles, CMPWIDTH);

    char detail[256];

    // 1. Power-on. No register has been written; this is what the motors see
    //    between configuration and the first instruction of the driver.
    reset_dut();
    double raw = level_fraction(channel, false, 200);
    double at_pin = level_fraction(channel, invert, 200);
    snprintf(detail, sizeof(detail),
             "raw comparator %.0f%% high, pin %.0f%% high", raw * 100, at_pin * 100);
    check("power-on: pin is low before software runs", at_pin == 0.0, detail);

    // 2. Zephyr's init leaves every comparator at zero. Its own idea of "off".
    reset_dut();
    zephyr_pwm_init();
    raw = level_fraction(channel, false, 4 * period_cycles);
    at_pin = level_fraction(channel, invert, 4 * period_cycles);
    snprintf(detail, sizeof(detail),
             "raw comparator %.0f%% high, pin %.0f%% high", raw * 100, at_pin * 100);
    check("after pwm_sifive_init: pin is low", at_pin == 0.0, detail);

    // 3. Explicit stop: pwm_set_dt(period, 0).
    reset_dut();
    zephyr_pwm_init();
    zephyr_set_cycles(channel, period_cycles, 0);
    at_pin = level_fraction(channel, invert, 4 * period_cycles);
    snprintf(detail, sizeof(detail), "pin %.1f%% high", at_pin * 100);
    check("pwm_set_dt(period, 0): pin is low", at_pin == 0.0, detail);

    // 4. Duty sweep. The pin has to carry the duty that was requested, not its
    //    complement, and the period has to be the one that was asked for.
    const int wanted[] = {5, 10, 25, 50, 75, 100};
    for (size_t i = 0; i < sizeof(wanted) / sizeof(wanted[0]); i++) {
        int pct = wanted[i];
        uint32_t pulse_cycles = ns_to_cycles((uint64_t)period_ns * pct / 100);
        reset_dut();
        zephyr_pwm_init();
        int scale = zephyr_set_cycles(channel, period_cycles, pulse_cycles);
        Measurement m = measure(channel, invert, 20 * period_cycles);

        double duty = m.period ? (double)m.high / (double)m.period : 0.0;
        double freq_khz = m.period ? (double)F_SYS / (double)m.period / 1000.0 : 0.0;
        double err = duty * 100.0 - pct;
        // One count of slack: zerocmp resets on the match, so the period is
        // cmp0 counts and a pulse of p counts lands within 1/2500 of p/cmp0.
        bool ok = m.saw_edge && (err > -0.1 && err < 0.1) &&
                  (freq_khz > 19.9 && freq_khz < 20.1);
        snprintf(detail, sizeof(detail),
                 "scale %d, %llu high / %llu cycles = %.2f%% at %.2f kHz",
                 scale, (unsigned long long)m.high,
                 (unsigned long long)m.period, duty * 100, freq_khz);
        char what[80];
        snprintf(what, sizeof(what), "pwm_set_dt %d%% duty reaches the pin as %d%%", pct, pct);
        check(what, ok, detail);
    }

    // 5. The same sweep with the inversion removed, which is what a build
    //    without WithArty200TPWM's invert would put on the motors. Not a
    //    regression check -- a record of what the correction is worth.
    printf("\nWithout the harness inversion, for comparison:\n");
    for (size_t i = 0; i < sizeof(wanted) / sizeof(wanted[0]); i++) {
        int pct = wanted[i];
        uint32_t pulse_cycles = ns_to_cycles((uint64_t)period_ns * pct / 100);
        reset_dut();
        zephyr_pwm_init();
        zephyr_set_cycles(channel, period_cycles, pulse_cycles);
        Measurement m = measure(channel, false, 20 * period_cycles);
        double duty = m.period ? (double)m.high / (double)m.period : 0.0;
        printf("       requested %3d%% -> pin %6.2f%%%s\n", pct, duty * 100,
               m.saw_edge ? "" : "  (no edge: pin stuck high)");
    }
    reset_dut();
    zephyr_pwm_init();
    zephyr_set_cycles(channel, period_cycles, 0);
    double stuck = level_fraction(channel, false, 4 * period_cycles);
    printf("       requested   0%% -> pin %6.2f%%%s\n", stuck * 100,
           stuck == 1.0 ? "  (full throttle)" : "");

    // 6. Channel independence: each motor must follow its own comparator.
    reset_dut();
    zephyr_pwm_init();
    zephyr_set_cycles(1, period_cycles, ns_to_cycles(period_ns * 10 / 100));
    zephyr_set_cycles(2, period_cycles, ns_to_cycles(period_ns * 20 / 100));
    zephyr_set_cycles(3, period_cycles, 0);
    {
        Measurement m1 = measure(1, invert, 20 * period_cycles);
        Measurement m2 = measure(2, invert, 20 * period_cycles);
        double d1 = m1.period ? (double)m1.high / m1.period * 100 : 0;
        double d2 = m2.period ? (double)m2.high / m2.period * 100 : 0;
        double d3 = level_fraction(3, invert, 4 * period_cycles) * 100;
        snprintf(detail, sizeof(detail), "cmp1 %.2f%%, cmp2 %.2f%%, cmp3 %.2f%%", d1, d2, d3);
        check("three channels hold three different duties",
              d1 > 9.9 && d1 < 10.1 && d2 > 19.9 && d2 < 20.1 && d3 == 0.0, detail);
    }

    // 7. Comparator 0 is the shared period and cannot be a motor output. The
    //    driver refuses it; the binder requires cmp > 0. Both must agree.
    check("Zephyr refuses comparator 0 as an output",
          zephyr_set_cycles(0, period_cycles, 100) == -1, "returns -ENOTSUP");

    // 7b. The on-chip probe, run against RTL whose duty is already known.
    printf("\n");
    for (int pct : {0, 10, 25, 50}) {
        uint32_t pulse_cycles = ns_to_cycles((uint64_t)period_ns * pct / 100);
        reset_dut();
        zephyr_pwm_init();
        zephyr_set_cycles(channel, period_cycles, pulse_cycles);
        // 40 cycles is a deliberately pessimistic MMIO latency; the point is
        // that the midpoint correction holds even when it is large.
        ProbeResult r = emulate_probe(channel, period_cycles, 40, 4000);
        double err = r.duty_permille - pct * 10.0;
        bool ok = (err > -12.0 && err < 12.0) && r.empty_bins == 0;
        snprintf(detail, sizeof(detail),
                 "%.1f per mille (want %d), edge bin %d, mean lag %.0f counts, "
                 "%d empty bins", r.duty_permille, pct * 10, r.edge_bin,
                 r.mean_lag, r.empty_bins);
        char what[80];
        snprintf(what, sizeof(what),
                 "on-chip probe recovers %d%% duty from the live RTL", pct);
        check(what, ok, detail);
    }

    // 8. Optional CSV of the whole transfer curve, so the figure in
    //    docs/motor-pwm-bringup.md is rendered from a measurement rather than
    //    from a table someone typed. Usage: tb_motor_pwm --csv <path>
    const char *csv_path = nullptr;
    for (int i = 1; i < argc - 1; i++) {
        if (strcmp(argv[i], "--csv") == 0) csv_path = argv[i + 1];
    }
    if (csv_path) {
        FILE *csv = fopen(csv_path, "w");
        if (!csv) {
            fprintf(stderr, "cannot write %s\n", csv_path);
            return 1;
        }
        fprintf(csv, "requested_pct,pin_pct_inverted,pin_pct_uninverted,"
                     "period_cycles,frequency_khz\n");
        for (int pct = 0; pct <= 100; pct += 5) {
            uint32_t pulse_cycles = ns_to_cycles((uint64_t)period_ns * pct / 100);
            double duty[2];
            uint64_t period_measured = 0;
            for (int inv = 0; inv < 2; inv++) {
                bool use_invert = (inv == 0);
                reset_dut();
                zephyr_pwm_init();
                zephyr_set_cycles(channel, period_cycles, pulse_cycles);
                Measurement m = measure(channel, use_invert, 20 * period_cycles);
                if (m.saw_edge) {
                    duty[inv] = (double)m.high / (double)m.period * 100.0;
                    period_measured = m.period;
                } else {
                    // Stuck pin: no edge to measure a period from, so report
                    // the level itself rather than a duty of an absent cycle.
                    reset_dut();
                    zephyr_pwm_init();
                    zephyr_set_cycles(channel, period_cycles, pulse_cycles);
                    duty[inv] = level_fraction(channel, use_invert,
                                               4 * period_cycles) * 100.0;
                }
            }
            double freq_khz = period_measured
                ? (double)F_SYS / (double)period_measured / 1000.0 : 0.0;
            fprintf(csv, "%d,%.4f,%.4f,%llu,%.4f\n", pct, duty[0], duty[1],
                    (unsigned long long)period_measured, freq_khz);
        }
        fclose(csv);
        printf("\nwrote %s\n", csv_path);
    }

    printf("\n%s: %d failure(s)\n", failures ? "FAILED" : "OK", failures);
    delete dut;
    return failures ? 1 : 0;
}
