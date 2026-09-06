// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Design internal header
// See VPWMTimer.h for the primary calling header

#ifndef VERILATED_VPWMTIMER___024ROOT_H_
#define VERILATED_VPWMTIMER___024ROOT_H_  // guard

#include "verilated.h"


class VPWMTimer__Syms;

class alignas(VL_CACHE_LINE_BYTES) VPWMTimer___024root final : public VerilatedModule {
  public:

    // DESIGN SPECIFIC STATE
    // Anonymous structures to workaround compiler member-count bugs
    struct {
        VL_IN8(clock,0,0);
        VL_IN8(reset,0,0);
        VL_IN8(scale_io_regs_cfg_write_ip_0,0,0);
        VL_IN8(scale_io_regs_cfg_write_ip_1,0,0);
        VL_IN8(scale_io_regs_cfg_write_ip_2,0,0);
        VL_IN8(scale_io_regs_cfg_write_ip_3,0,0);
        VL_IN8(scale_io_regs_cfg_write_gang_0,0,0);
        VL_IN8(scale_io_regs_cfg_write_gang_1,0,0);
        VL_IN8(scale_io_regs_cfg_write_gang_2,0,0);
        VL_IN8(scale_io_regs_cfg_write_gang_3,0,0);
        VL_IN8(scale_io_regs_cfg_write_extra_0,0,0);
        VL_IN8(scale_io_regs_cfg_write_extra_1,0,0);
        VL_IN8(scale_io_regs_cfg_write_extra_2,0,0);
        VL_IN8(scale_io_regs_cfg_write_extra_3,0,0);
        VL_IN8(scale_io_regs_cfg_write_center_0,0,0);
        VL_IN8(scale_io_regs_cfg_write_center_1,0,0);
        VL_IN8(scale_io_regs_cfg_write_center_2,0,0);
        VL_IN8(scale_io_regs_cfg_write_center_3,0,0);
        VL_IN8(scale_io_regs_cfg_write_running,0,0);
        VL_IN8(scale_io_regs_cfg_write_countAlways,0,0);
        VL_IN8(scale_io_regs_cfg_write_deglitch,0,0);
        VL_IN8(scale_io_regs_cfg_write_zerocmp,0,0);
        VL_IN8(scale_io_regs_cfg_write_sticky,0,0);
        VL_IN8(scale_io_regs_cfg_write_scale,3,0);
        VL_OUT8(scale_io_regs_cfg_read_ip_0,0,0);
        VL_OUT8(scale_io_regs_cfg_read_ip_1,0,0);
        VL_OUT8(scale_io_regs_cfg_read_ip_2,0,0);
        VL_OUT8(scale_io_regs_cfg_read_ip_3,0,0);
        VL_OUT8(scale_io_regs_cfg_read_gang_0,0,0);
        VL_OUT8(scale_io_regs_cfg_read_gang_1,0,0);
        VL_OUT8(scale_io_regs_cfg_read_gang_2,0,0);
        VL_OUT8(scale_io_regs_cfg_read_gang_3,0,0);
        VL_OUT8(scale_io_regs_cfg_read_extra_0,0,0);
        VL_OUT8(scale_io_regs_cfg_read_extra_1,0,0);
        VL_OUT8(scale_io_regs_cfg_read_extra_2,0,0);
        VL_OUT8(scale_io_regs_cfg_read_extra_3,0,0);
        VL_OUT8(scale_io_regs_cfg_read_center_0,0,0);
        VL_OUT8(scale_io_regs_cfg_read_center_1,0,0);
        VL_OUT8(scale_io_regs_cfg_read_center_2,0,0);
        VL_OUT8(scale_io_regs_cfg_read_center_3,0,0);
        VL_OUT8(scale_io_regs_cfg_read_running,0,0);
        VL_OUT8(scale_io_regs_cfg_read_countAlways,0,0);
        VL_OUT8(scale_io_regs_cfg_read_deglitch,0,0);
        VL_OUT8(scale_io_regs_cfg_read_zerocmp,0,0);
        VL_OUT8(scale_io_regs_cfg_read_sticky,0,0);
        VL_OUT8(scale_io_regs_cfg_read_scale,3,0);
        VL_IN8(scale_io_regs_cfg_write_ip_0_0,0,0);
        VL_IN8(scale_io_regs_cfg_write_ip_1_0,0,0);
        VL_IN8(scale_io_regs_cfg_write_ip_2_0,0,0);
        VL_IN8(scale_io_regs_cfg_write_ip_3_0,0,0);
        VL_IN8(scale_io_regs_cfg_write_gang_0_0,0,0);
        VL_IN8(scale_io_regs_cfg_write_gang_1_0,0,0);
        VL_IN8(scale_io_regs_cfg_write_gang_2_0,0,0);
        VL_IN8(scale_io_regs_cfg_write_gang_3_0,0,0);
        VL_IN8(scale_io_regs_cfg_write_extra_0_0,0,0);
        VL_IN8(scale_io_regs_cfg_write_extra_1_0,0,0);
        VL_IN8(scale_io_regs_cfg_write_extra_2_0,0,0);
        VL_IN8(scale_io_regs_cfg_write_extra_3_0,0,0);
        VL_IN8(scale_io_regs_cfg_write_center_0_0,0,0);
        VL_IN8(scale_io_regs_cfg_write_center_1_0,0,0);
        VL_IN8(scale_io_regs_cfg_write_center_2_0,0,0);
        VL_IN8(scale_io_regs_cfg_write_center_3_0,0,0);
        VL_IN8(scale_io_regs_cfg_write_running_0,0,0);
        VL_IN8(scale_io_regs_cfg_write_countAlways_0,0,0);
    };
    struct {
        VL_IN8(scale_io_regs_cfg_write_deglitch_0,0,0);
        VL_IN8(scale_io_regs_cfg_write_zerocmp_0,0,0);
        VL_IN8(scale_io_regs_cfg_write_sticky_0,0,0);
        VL_IN8(scale_io_regs_cfg_write_scale_0,0,0);
        VL_IN8(scale_io_regs_countLo_write_valid,0,0);
        VL_IN8(scale_io_regs_cmp_0_write_valid,0,0);
        VL_IN8(scale_io_regs_cmp_1_write_valid,0,0);
        VL_IN8(scale_io_regs_cmp_2_write_valid,0,0);
        VL_IN8(scale_io_regs_cmp_3_write_valid,0,0);
        VL_OUT8(scale_io_ip_0,0,0);
        VL_OUT8(scale_io_ip_1,0,0);
        VL_OUT8(scale_io_ip_2,0,0);
        VL_OUT8(scale_io_ip_3,0,0);
        VL_OUT8(scale_io_gpio_0,0,0);
        VL_OUT8(scale_io_gpio_1,0,0);
        VL_OUT8(scale_io_gpio_2,0,0);
        VL_OUT8(scale_io_gpio_3,0,0);
        CData/*3:0*/ PWMTimer__DOT__scale;
        CData/*4:0*/ PWMTimer__DOT__small_0;
        CData/*0:0*/ PWMTimer__DOT__elapsed_center_0;
        CData/*0:0*/ PWMTimer__DOT__elapsed_center_1;
        CData/*0:0*/ PWMTimer__DOT__elapsed_center_2;
        CData/*0:0*/ PWMTimer__DOT__elapsed_center_3;
        CData/*0:0*/ PWMTimer__DOT__countReset_zerocmp;
        CData/*0:0*/ PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_doSticky_deglitch;
        CData/*0:0*/ PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_doSticky_sticky;
        CData/*0:0*/ PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_doSticky;
        CData/*0:0*/ PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_0;
        CData/*0:0*/ PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_1;
        CData/*0:0*/ PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_2;
        CData/*0:0*/ PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_3;
        CData/*0:0*/ PWMTimer__DOT__gang_0;
        CData/*0:0*/ PWMTimer__DOT__gang_1;
        CData/*0:0*/ PWMTimer__DOT__gang_2;
        CData/*0:0*/ PWMTimer__DOT__gang_3;
        CData/*0:0*/ PWMTimer__DOT__extra_0;
        CData/*0:0*/ PWMTimer__DOT__extra_1;
        CData/*0:0*/ PWMTimer__DOT__extra_2;
        CData/*0:0*/ PWMTimer__DOT__extra_3;
        CData/*0:0*/ PWMTimer__DOT__io_regs_cfg_read_running_oneShot;
        CData/*0:0*/ PWMTimer__DOT__io_regs_cfg_read_countAlways_countAlways;
        CData/*5:0*/ PWMTimer__DOT__nextSmall;
        CData/*0:0*/ PWMTimer__DOT__elapsed_0;
        CData/*0:0*/ PWMTimer__DOT__elapsed_1;
        CData/*0:0*/ PWMTimer__DOT__elapsed_2;
        CData/*0:0*/ PWMTimer__DOT__elapsed_3;
        CData/*0:0*/ PWMTimer__DOT__countReset;
        CData/*0:0*/ PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_sel_0;
        CData/*0:0*/ PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_sel_1;
        CData/*0:0*/ PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_sel_2;
        CData/*0:0*/ PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_sel_3;
        CData/*0:0*/ PWMTimer__DOT___io_regs_cfg_read_ip_0_ip_T_28;
        CData/*0:0*/ __VstlFirstIteration;
        CData/*0:0*/ __VicoFirstIteration;
        CData/*0:0*/ __Vtrigprevexpr___TOP__clock__0;
        CData/*0:0*/ __VactContinue;
        VL_OUT16(scale_io_regs_s_read,15,0);
        VL_IN16(scale_io_regs_cmp_0_write_bits,15,0);
        VL_OUT16(scale_io_regs_cmp_0_read,15,0);
        VL_IN16(scale_io_regs_cmp_1_write_bits,15,0);
        VL_OUT16(scale_io_regs_cmp_1_read,15,0);
        VL_IN16(scale_io_regs_cmp_2_write_bits,15,0);
        VL_OUT16(scale_io_regs_cmp_2_read,15,0);
        VL_IN16(scale_io_regs_cmp_3_write_bits,15,0);
    };
    struct {
        VL_OUT16(scale_io_regs_cmp_3_read,15,0);
        SData/*15:0*/ PWMTimer__DOT__cmp_0;
        SData/*15:0*/ PWMTimer__DOT__cmp_1;
        SData/*15:0*/ PWMTimer__DOT__cmp_2;
        SData/*15:0*/ PWMTimer__DOT__cmp_3;
        VL_IN(scale_io_regs_countLo_write_bits,30,0);
        VL_OUT(scale_io_regs_countLo_read,30,0);
        IData/*25:0*/ PWMTimer__DOT__large_0;
        IData/*31:0*/ __VactIterCount;
    };
    VlTriggerVec<1> __VstlTriggered;
    VlTriggerVec<1> __VicoTriggered;
    VlTriggerVec<1> __VactTriggered;
    VlTriggerVec<1> __VnbaTriggered;

    // INTERNAL VARIABLES
    VPWMTimer__Syms* const vlSymsp;

    // CONSTRUCTORS
    VPWMTimer___024root(VPWMTimer__Syms* symsp, const char* v__name);
    ~VPWMTimer___024root();
    VL_UNCOPYABLE(VPWMTimer___024root);

    // INTERNAL METHODS
    void __Vconfigure(bool first);
};


#endif  // guard
