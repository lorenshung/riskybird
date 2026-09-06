// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Primary model header
//
// This header should be included by all source files instantiating the design.
// The class here is then constructed to instantiate the design.
// See the Verilator manual for examples.

#ifndef VERILATED_VPWMTIMER_H_
#define VERILATED_VPWMTIMER_H_  // guard

#include "verilated.h"

class VPWMTimer__Syms;
class VPWMTimer___024root;

// This class is the main interface to the Verilated model
class alignas(VL_CACHE_LINE_BYTES) VPWMTimer VL_NOT_FINAL : public VerilatedModel {
  private:
    // Symbol table holding complete model state (owned by this class)
    VPWMTimer__Syms* const vlSymsp;

  public:

    // PORTS
    // The application code writes and reads these signals to
    // propagate new values into/out from the Verilated model.
    VL_IN8(&clock,0,0);
    VL_IN8(&reset,0,0);
    VL_IN8(&scale_io_regs_cfg_write_ip_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_ip_1,0,0);
    VL_IN8(&scale_io_regs_cfg_write_ip_2,0,0);
    VL_IN8(&scale_io_regs_cfg_write_ip_3,0,0);
    VL_IN8(&scale_io_regs_cfg_write_gang_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_gang_1,0,0);
    VL_IN8(&scale_io_regs_cfg_write_gang_2,0,0);
    VL_IN8(&scale_io_regs_cfg_write_gang_3,0,0);
    VL_IN8(&scale_io_regs_cfg_write_extra_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_extra_1,0,0);
    VL_IN8(&scale_io_regs_cfg_write_extra_2,0,0);
    VL_IN8(&scale_io_regs_cfg_write_extra_3,0,0);
    VL_IN8(&scale_io_regs_cfg_write_center_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_center_1,0,0);
    VL_IN8(&scale_io_regs_cfg_write_center_2,0,0);
    VL_IN8(&scale_io_regs_cfg_write_center_3,0,0);
    VL_IN8(&scale_io_regs_cfg_write_running,0,0);
    VL_IN8(&scale_io_regs_cfg_write_countAlways,0,0);
    VL_IN8(&scale_io_regs_cfg_write_deglitch,0,0);
    VL_IN8(&scale_io_regs_cfg_write_zerocmp,0,0);
    VL_IN8(&scale_io_regs_cfg_write_sticky,0,0);
    VL_IN8(&scale_io_regs_cfg_write_scale,3,0);
    VL_OUT8(&scale_io_regs_cfg_read_ip_0,0,0);
    VL_OUT8(&scale_io_regs_cfg_read_ip_1,0,0);
    VL_OUT8(&scale_io_regs_cfg_read_ip_2,0,0);
    VL_OUT8(&scale_io_regs_cfg_read_ip_3,0,0);
    VL_OUT8(&scale_io_regs_cfg_read_gang_0,0,0);
    VL_OUT8(&scale_io_regs_cfg_read_gang_1,0,0);
    VL_OUT8(&scale_io_regs_cfg_read_gang_2,0,0);
    VL_OUT8(&scale_io_regs_cfg_read_gang_3,0,0);
    VL_OUT8(&scale_io_regs_cfg_read_extra_0,0,0);
    VL_OUT8(&scale_io_regs_cfg_read_extra_1,0,0);
    VL_OUT8(&scale_io_regs_cfg_read_extra_2,0,0);
    VL_OUT8(&scale_io_regs_cfg_read_extra_3,0,0);
    VL_OUT8(&scale_io_regs_cfg_read_center_0,0,0);
    VL_OUT8(&scale_io_regs_cfg_read_center_1,0,0);
    VL_OUT8(&scale_io_regs_cfg_read_center_2,0,0);
    VL_OUT8(&scale_io_regs_cfg_read_center_3,0,0);
    VL_OUT8(&scale_io_regs_cfg_read_running,0,0);
    VL_OUT8(&scale_io_regs_cfg_read_countAlways,0,0);
    VL_OUT8(&scale_io_regs_cfg_read_deglitch,0,0);
    VL_OUT8(&scale_io_regs_cfg_read_zerocmp,0,0);
    VL_OUT8(&scale_io_regs_cfg_read_sticky,0,0);
    VL_OUT8(&scale_io_regs_cfg_read_scale,3,0);
    VL_IN8(&scale_io_regs_cfg_write_ip_0_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_ip_1_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_ip_2_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_ip_3_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_gang_0_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_gang_1_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_gang_2_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_gang_3_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_extra_0_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_extra_1_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_extra_2_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_extra_3_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_center_0_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_center_1_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_center_2_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_center_3_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_running_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_countAlways_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_deglitch_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_zerocmp_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_sticky_0,0,0);
    VL_IN8(&scale_io_regs_cfg_write_scale_0,0,0);
    VL_IN8(&scale_io_regs_countLo_write_valid,0,0);
    VL_IN8(&scale_io_regs_cmp_0_write_valid,0,0);
    VL_IN8(&scale_io_regs_cmp_1_write_valid,0,0);
    VL_IN8(&scale_io_regs_cmp_2_write_valid,0,0);
    VL_IN8(&scale_io_regs_cmp_3_write_valid,0,0);
    VL_OUT8(&scale_io_ip_0,0,0);
    VL_OUT8(&scale_io_ip_1,0,0);
    VL_OUT8(&scale_io_ip_2,0,0);
    VL_OUT8(&scale_io_ip_3,0,0);
    VL_OUT8(&scale_io_gpio_0,0,0);
    VL_OUT8(&scale_io_gpio_1,0,0);
    VL_OUT8(&scale_io_gpio_2,0,0);
    VL_OUT8(&scale_io_gpio_3,0,0);
    VL_OUT16(&scale_io_regs_s_read,15,0);
    VL_IN16(&scale_io_regs_cmp_0_write_bits,15,0);
    VL_OUT16(&scale_io_regs_cmp_0_read,15,0);
    VL_IN16(&scale_io_regs_cmp_1_write_bits,15,0);
    VL_OUT16(&scale_io_regs_cmp_1_read,15,0);
    VL_IN16(&scale_io_regs_cmp_2_write_bits,15,0);
    VL_OUT16(&scale_io_regs_cmp_2_read,15,0);
    VL_IN16(&scale_io_regs_cmp_3_write_bits,15,0);
    VL_OUT16(&scale_io_regs_cmp_3_read,15,0);
    VL_IN(&scale_io_regs_countLo_write_bits,30,0);
    VL_OUT(&scale_io_regs_countLo_read,30,0);

    // CELLS
    // Public to allow access to /* verilator public */ items.
    // Otherwise the application code can consider these internals.

    // Root instance pointer to allow access to model internals,
    // including inlined /* verilator public_flat_* */ items.
    VPWMTimer___024root* const rootp;

    // CONSTRUCTORS
    /// Construct the model; called by application code
    /// If contextp is null, then the model will use the default global context
    /// If name is "", then makes a wrapper with a
    /// single model invisible with respect to DPI scope names.
    explicit VPWMTimer(VerilatedContext* contextp, const char* name = "TOP");
    explicit VPWMTimer(const char* name = "TOP");
    /// Destroy the model; called (often implicitly) by application code
    virtual ~VPWMTimer();
  private:
    VL_UNCOPYABLE(VPWMTimer);  ///< Copying not allowed

  public:
    // API METHODS
    /// Evaluate the model.  Application must call when inputs change.
    void eval() { eval_step(); }
    /// Evaluate when calling multiple units/models per time step.
    void eval_step();
    /// Evaluate at end of a timestep for tracing, when using eval_step().
    /// Application must call after all eval() and before time changes.
    void eval_end_step() {}
    /// Simulation complete, run final blocks.  Application must call on completion.
    void final();
    /// Are there scheduled events to handle?
    bool eventsPending();
    /// Returns time at next time slot. Aborts if !eventsPending()
    uint64_t nextTimeSlot();
    /// Trace signals in the model; called by application code
    void trace(VerilatedVcdC* tfp, int levels, int options = 0);
    /// Retrieve name of this model instance (as passed to constructor).
    const char* name() const;

    // Abstract methods from VerilatedModel
    const char* hierName() const override final;
    const char* modelName() const override final;
    unsigned threads() const override final;
    /// Prepare for cloning the model at the process level (e.g. fork in Linux)
    /// Release necessary resources. Called before cloning.
    void prepareClone() const;
    /// Re-init after cloning the model at the process level (e.g. fork in Linux)
    /// Re-allocate necessary resources. Called after cloning.
    void atClone() const;
};

#endif  // guard
