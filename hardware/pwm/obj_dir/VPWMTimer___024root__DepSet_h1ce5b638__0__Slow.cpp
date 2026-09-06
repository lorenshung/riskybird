// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Design implementation internals
// See VPWMTimer.h for the primary calling header

#include "VPWMTimer__pch.h"
#include "VPWMTimer___024root.h"

VL_ATTR_COLD void VPWMTimer___024root___eval_static(VPWMTimer___024root* vlSelf) {
    (void)vlSelf;  // Prevent unused variable warning
    VPWMTimer__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    VL_DEBUG_IF(VL_DBG_MSGF("+    VPWMTimer___024root___eval_static\n"); );
}

VL_ATTR_COLD void VPWMTimer___024root___eval_initial(VPWMTimer___024root* vlSelf) {
    (void)vlSelf;  // Prevent unused variable warning
    VPWMTimer__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    VL_DEBUG_IF(VL_DBG_MSGF("+    VPWMTimer___024root___eval_initial\n"); );
    // Body
    vlSelf->__Vtrigprevexpr___TOP__clock__0 = vlSelf->clock;
}

VL_ATTR_COLD void VPWMTimer___024root___eval_final(VPWMTimer___024root* vlSelf) {
    (void)vlSelf;  // Prevent unused variable warning
    VPWMTimer__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    VL_DEBUG_IF(VL_DBG_MSGF("+    VPWMTimer___024root___eval_final\n"); );
}

#ifdef VL_DEBUG
VL_ATTR_COLD void VPWMTimer___024root___dump_triggers__stl(VPWMTimer___024root* vlSelf);
#endif  // VL_DEBUG
VL_ATTR_COLD bool VPWMTimer___024root___eval_phase__stl(VPWMTimer___024root* vlSelf);

VL_ATTR_COLD void VPWMTimer___024root___eval_settle(VPWMTimer___024root* vlSelf) {
    (void)vlSelf;  // Prevent unused variable warning
    VPWMTimer__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    VL_DEBUG_IF(VL_DBG_MSGF("+    VPWMTimer___024root___eval_settle\n"); );
    // Init
    IData/*31:0*/ __VstlIterCount;
    CData/*0:0*/ __VstlContinue;
    // Body
    __VstlIterCount = 0U;
    vlSelf->__VstlFirstIteration = 1U;
    __VstlContinue = 1U;
    while (__VstlContinue) {
        if (VL_UNLIKELY((0x64U < __VstlIterCount))) {
#ifdef VL_DEBUG
            VPWMTimer___024root___dump_triggers__stl(vlSelf);
#endif
            VL_FATAL_MT("/scratch2/loren/riskybird-dev/backends/chipyard/fpga/generated-src/chipyard.fpga.arty200t.Arty200THarness.RocketArty200TMotorSpadConfig/gen-collateral/PWMTimer.sv", 46, "", "Settle region did not converge.");
        }
        __VstlIterCount = ((IData)(1U) + __VstlIterCount);
        __VstlContinue = 0U;
        if (VPWMTimer___024root___eval_phase__stl(vlSelf)) {
            __VstlContinue = 1U;
        }
        vlSelf->__VstlFirstIteration = 0U;
    }
}

#ifdef VL_DEBUG
VL_ATTR_COLD void VPWMTimer___024root___dump_triggers__stl(VPWMTimer___024root* vlSelf) {
    (void)vlSelf;  // Prevent unused variable warning
    VPWMTimer__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    VL_DEBUG_IF(VL_DBG_MSGF("+    VPWMTimer___024root___dump_triggers__stl\n"); );
    // Body
    if ((1U & (~ (IData)(vlSelf->__VstlTriggered.any())))) {
        VL_DBG_MSGF("         No triggers active\n");
    }
    if ((1ULL & vlSelf->__VstlTriggered.word(0U))) {
        VL_DBG_MSGF("         'stl' region trigger index 0 is active: Internal 'stl' trigger - first iteration\n");
    }
}
#endif  // VL_DEBUG

VL_ATTR_COLD void VPWMTimer___024root___stl_sequent__TOP__0(VPWMTimer___024root* vlSelf) {
    (void)vlSelf;  // Prevent unused variable warning
    VPWMTimer__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    VL_DEBUG_IF(VL_DBG_MSGF("+    VPWMTimer___024root___stl_sequent__TOP__0\n"); );
    // Init
    IData/*30:0*/ PWMTimer__DOT___s_T;
    PWMTimer__DOT___s_T = 0;
    CData/*3:0*/ PWMTimer__DOT___GEN;
    PWMTimer__DOT___GEN = 0;
    // Body
    vlSelf->scale_io_regs_cfg_read_gang_0 = vlSelf->PWMTimer__DOT__gang_0;
    vlSelf->scale_io_regs_cfg_read_gang_1 = vlSelf->PWMTimer__DOT__gang_1;
    vlSelf->scale_io_regs_cfg_read_gang_2 = vlSelf->PWMTimer__DOT__gang_2;
    vlSelf->scale_io_regs_cfg_read_gang_3 = vlSelf->PWMTimer__DOT__gang_3;
    vlSelf->scale_io_regs_cfg_read_extra_0 = vlSelf->PWMTimer__DOT__extra_0;
    vlSelf->scale_io_regs_cfg_read_extra_1 = vlSelf->PWMTimer__DOT__extra_1;
    vlSelf->scale_io_regs_cfg_read_extra_2 = vlSelf->PWMTimer__DOT__extra_2;
    vlSelf->scale_io_regs_cfg_read_extra_3 = vlSelf->PWMTimer__DOT__extra_3;
    vlSelf->scale_io_regs_cfg_read_center_0 = vlSelf->PWMTimer__DOT__elapsed_center_0;
    vlSelf->scale_io_regs_cfg_read_center_1 = vlSelf->PWMTimer__DOT__elapsed_center_1;
    vlSelf->scale_io_regs_cfg_read_center_2 = vlSelf->PWMTimer__DOT__elapsed_center_2;
    vlSelf->scale_io_regs_cfg_read_center_3 = vlSelf->PWMTimer__DOT__elapsed_center_3;
    vlSelf->scale_io_regs_cfg_read_running = vlSelf->PWMTimer__DOT__io_regs_cfg_read_running_oneShot;
    vlSelf->scale_io_regs_cfg_read_countAlways = vlSelf->PWMTimer__DOT__io_regs_cfg_read_countAlways_countAlways;
    vlSelf->scale_io_regs_cfg_read_zerocmp = vlSelf->PWMTimer__DOT__countReset_zerocmp;
    vlSelf->scale_io_regs_cfg_read_scale = vlSelf->PWMTimer__DOT__scale;
    vlSelf->scale_io_regs_cmp_0_read = vlSelf->PWMTimer__DOT__cmp_0;
    vlSelf->scale_io_regs_cmp_1_read = vlSelf->PWMTimer__DOT__cmp_1;
    vlSelf->scale_io_regs_cmp_2_read = vlSelf->PWMTimer__DOT__cmp_2;
    vlSelf->scale_io_regs_cmp_3_read = vlSelf->PWMTimer__DOT__cmp_3;
    vlSelf->scale_io_regs_cfg_read_deglitch = vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_doSticky_deglitch;
    vlSelf->scale_io_regs_cfg_read_sticky = vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_doSticky_sticky;
    vlSelf->scale_io_regs_cfg_read_ip_0 = vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_0;
    vlSelf->scale_io_regs_cfg_read_ip_1 = vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_1;
    vlSelf->scale_io_regs_cfg_read_ip_2 = vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_2;
    vlSelf->scale_io_regs_cfg_read_ip_3 = vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_3;
    vlSelf->PWMTimer__DOT___io_regs_cfg_read_ip_0_ip_T_28 
        = ((IData)(vlSelf->scale_io_regs_cfg_write_ip_0_0) 
           | ((IData)(vlSelf->scale_io_regs_cfg_write_ip_1_0) 
              | ((IData)(vlSelf->scale_io_regs_cfg_write_ip_2_0) 
                 | (IData)(vlSelf->scale_io_regs_cfg_write_ip_3_0))));
    vlSelf->PWMTimer__DOT__nextSmall = (0x3fU & ((IData)(vlSelf->PWMTimer__DOT__small_0) 
                                                 + 
                                                 ((IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_countAlways_countAlways) 
                                                  | (IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_running_oneShot))));
    PWMTimer__DOT___GEN = (((~ ((((IData)(vlSelf->PWMTimer__DOT__gang_3) 
                                  << 3U) | (((IData)(vlSelf->PWMTimer__DOT__gang_2) 
                                             << 2U) 
                                            | (((IData)(vlSelf->PWMTimer__DOT__gang_1) 
                                                << 1U) 
                                               | (IData)(vlSelf->PWMTimer__DOT__gang_0)))) 
                                & (((IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_0) 
                                    << 3U) | (((IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_3) 
                                               << 2U) 
                                              | (((IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_2) 
                                                  << 1U) 
                                                 | (IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_1)))))) 
                            & (((IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_3) 
                                << 3U) | (((IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_2) 
                                           << 2U) | 
                                          (((IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_1) 
                                            << 1U) 
                                           | (IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_0))))) 
                           ^ (((IData)(vlSelf->PWMTimer__DOT__extra_3) 
                               << 3U) | (((IData)(vlSelf->PWMTimer__DOT__extra_2) 
                                          << 2U) | 
                                         (((IData)(vlSelf->PWMTimer__DOT__extra_1) 
                                           << 1U) | (IData)(vlSelf->PWMTimer__DOT__extra_0)))));
    vlSelf->scale_io_regs_countLo_read = ((vlSelf->PWMTimer__DOT__large_0 
                                           << 5U) | (IData)(vlSelf->PWMTimer__DOT__small_0));
    vlSelf->scale_io_ip_0 = vlSelf->scale_io_regs_cfg_read_ip_0;
    vlSelf->scale_io_ip_1 = vlSelf->scale_io_regs_cfg_read_ip_1;
    vlSelf->scale_io_ip_2 = vlSelf->scale_io_regs_cfg_read_ip_2;
    vlSelf->scale_io_ip_3 = vlSelf->scale_io_regs_cfg_read_ip_3;
    vlSelf->scale_io_gpio_0 = (1U & (IData)(PWMTimer__DOT___GEN));
    vlSelf->scale_io_gpio_1 = (1U & ((IData)(PWMTimer__DOT___GEN) 
                                     >> 1U));
    vlSelf->scale_io_gpio_2 = (1U & ((IData)(PWMTimer__DOT___GEN) 
                                     >> 2U));
    vlSelf->scale_io_gpio_3 = (1U & ((IData)(PWMTimer__DOT___GEN) 
                                     >> 3U));
    PWMTimer__DOT___s_T = (0x7fffffffU & (vlSelf->scale_io_regs_countLo_read 
                                          >> (IData)(vlSelf->PWMTimer__DOT__scale)));
    vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_sel_1 
        = ((PWMTimer__DOT___s_T >> 0xfU) & (IData)(vlSelf->PWMTimer__DOT__elapsed_center_1));
    vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_sel_2 
        = ((PWMTimer__DOT___s_T >> 0xfU) & (IData)(vlSelf->PWMTimer__DOT__elapsed_center_2));
    vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_sel_3 
        = ((PWMTimer__DOT___s_T >> 0xfU) & (IData)(vlSelf->PWMTimer__DOT__elapsed_center_3));
    vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_sel_0 
        = ((PWMTimer__DOT___s_T >> 0xfU) & (IData)(vlSelf->PWMTimer__DOT__elapsed_center_0));
    vlSelf->scale_io_regs_s_read = (0xffffU & PWMTimer__DOT___s_T);
    vlSelf->PWMTimer__DOT__elapsed_1 = ((0xffffU & 
                                         ((- (IData)((IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_sel_1))) 
                                          ^ (IData)(vlSelf->scale_io_regs_s_read))) 
                                        >= (IData)(vlSelf->PWMTimer__DOT__cmp_1));
    vlSelf->PWMTimer__DOT__elapsed_2 = ((0xffffU & 
                                         ((- (IData)((IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_sel_2))) 
                                          ^ (IData)(vlSelf->scale_io_regs_s_read))) 
                                        >= (IData)(vlSelf->PWMTimer__DOT__cmp_2));
    vlSelf->PWMTimer__DOT__elapsed_3 = ((0xffffU & 
                                         ((- (IData)((IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_sel_3))) 
                                          ^ (IData)(vlSelf->scale_io_regs_s_read))) 
                                        >= (IData)(vlSelf->PWMTimer__DOT__cmp_3));
    vlSelf->PWMTimer__DOT__elapsed_0 = ((0xffffU & 
                                         ((- (IData)((IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_sel_0))) 
                                          ^ (IData)(vlSelf->scale_io_regs_s_read))) 
                                        >= (IData)(vlSelf->PWMTimer__DOT__cmp_0));
    vlSelf->PWMTimer__DOT__countReset = (1U & ((0x7fffffffU 
                                                & (((((0x20U 
                                                       & (IData)(vlSelf->PWMTimer__DOT__nextSmall))
                                                       ? 
                                                      (0x3ffffffU 
                                                       & (VL_SHIFTR_III(26,26,32, vlSelf->PWMTimer__DOT__large_0, 1U) 
                                                          ^ 
                                                          (0x3ffffffU 
                                                           & (((IData)(1U) 
                                                               + vlSelf->PWMTimer__DOT__large_0) 
                                                              >> 1U))))
                                                       : 0U) 
                                                     << 5U) 
                                                    | (0x1fU 
                                                       & (VL_SHIFTR_III(5,5,32, (IData)(vlSelf->PWMTimer__DOT__small_0), 1U) 
                                                          ^ 
                                                          ((IData)(vlSelf->PWMTimer__DOT__nextSmall) 
                                                           >> 1U)))) 
                                                   >> 
                                                   (0x1fU 
                                                    & ((IData)(vlSelf->PWMTimer__DOT__scale) 
                                                       - (IData)(0x10U))))) 
                                               | ((IData)(vlSelf->PWMTimer__DOT__countReset_zerocmp) 
                                                  & (IData)(vlSelf->PWMTimer__DOT__elapsed_0))));
}

VL_ATTR_COLD void VPWMTimer___024root___eval_stl(VPWMTimer___024root* vlSelf) {
    (void)vlSelf;  // Prevent unused variable warning
    VPWMTimer__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    VL_DEBUG_IF(VL_DBG_MSGF("+    VPWMTimer___024root___eval_stl\n"); );
    // Body
    if ((1ULL & vlSelf->__VstlTriggered.word(0U))) {
        VPWMTimer___024root___stl_sequent__TOP__0(vlSelf);
    }
}

VL_ATTR_COLD void VPWMTimer___024root___eval_triggers__stl(VPWMTimer___024root* vlSelf);

VL_ATTR_COLD bool VPWMTimer___024root___eval_phase__stl(VPWMTimer___024root* vlSelf) {
    (void)vlSelf;  // Prevent unused variable warning
    VPWMTimer__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    VL_DEBUG_IF(VL_DBG_MSGF("+    VPWMTimer___024root___eval_phase__stl\n"); );
    // Init
    CData/*0:0*/ __VstlExecute;
    // Body
    VPWMTimer___024root___eval_triggers__stl(vlSelf);
    __VstlExecute = vlSelf->__VstlTriggered.any();
    if (__VstlExecute) {
        VPWMTimer___024root___eval_stl(vlSelf);
    }
    return (__VstlExecute);
}

#ifdef VL_DEBUG
VL_ATTR_COLD void VPWMTimer___024root___dump_triggers__ico(VPWMTimer___024root* vlSelf) {
    (void)vlSelf;  // Prevent unused variable warning
    VPWMTimer__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    VL_DEBUG_IF(VL_DBG_MSGF("+    VPWMTimer___024root___dump_triggers__ico\n"); );
    // Body
    if ((1U & (~ (IData)(vlSelf->__VicoTriggered.any())))) {
        VL_DBG_MSGF("         No triggers active\n");
    }
    if ((1ULL & vlSelf->__VicoTriggered.word(0U))) {
        VL_DBG_MSGF("         'ico' region trigger index 0 is active: Internal 'ico' trigger - first iteration\n");
    }
}
#endif  // VL_DEBUG

#ifdef VL_DEBUG
VL_ATTR_COLD void VPWMTimer___024root___dump_triggers__act(VPWMTimer___024root* vlSelf) {
    (void)vlSelf;  // Prevent unused variable warning
    VPWMTimer__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    VL_DEBUG_IF(VL_DBG_MSGF("+    VPWMTimer___024root___dump_triggers__act\n"); );
    // Body
    if ((1U & (~ (IData)(vlSelf->__VactTriggered.any())))) {
        VL_DBG_MSGF("         No triggers active\n");
    }
    if ((1ULL & vlSelf->__VactTriggered.word(0U))) {
        VL_DBG_MSGF("         'act' region trigger index 0 is active: @(posedge clock)\n");
    }
}
#endif  // VL_DEBUG

#ifdef VL_DEBUG
VL_ATTR_COLD void VPWMTimer___024root___dump_triggers__nba(VPWMTimer___024root* vlSelf) {
    (void)vlSelf;  // Prevent unused variable warning
    VPWMTimer__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    VL_DEBUG_IF(VL_DBG_MSGF("+    VPWMTimer___024root___dump_triggers__nba\n"); );
    // Body
    if ((1U & (~ (IData)(vlSelf->__VnbaTriggered.any())))) {
        VL_DBG_MSGF("         No triggers active\n");
    }
    if ((1ULL & vlSelf->__VnbaTriggered.word(0U))) {
        VL_DBG_MSGF("         'nba' region trigger index 0 is active: @(posedge clock)\n");
    }
}
#endif  // VL_DEBUG

VL_ATTR_COLD void VPWMTimer___024root___ctor_var_reset(VPWMTimer___024root* vlSelf) {
    (void)vlSelf;  // Prevent unused variable warning
    VPWMTimer__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    VL_DEBUG_IF(VL_DBG_MSGF("+    VPWMTimer___024root___ctor_var_reset\n"); );
    // Body
    vlSelf->clock = VL_RAND_RESET_I(1);
    vlSelf->reset = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_ip_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_ip_1 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_ip_2 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_ip_3 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_gang_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_gang_1 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_gang_2 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_gang_3 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_extra_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_extra_1 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_extra_2 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_extra_3 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_center_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_center_1 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_center_2 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_center_3 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_running = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_countAlways = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_deglitch = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_zerocmp = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_sticky = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_scale = VL_RAND_RESET_I(4);
    vlSelf->scale_io_regs_cfg_read_ip_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_read_ip_1 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_read_ip_2 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_read_ip_3 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_read_gang_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_read_gang_1 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_read_gang_2 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_read_gang_3 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_read_extra_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_read_extra_1 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_read_extra_2 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_read_extra_3 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_read_center_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_read_center_1 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_read_center_2 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_read_center_3 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_read_running = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_read_countAlways = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_read_deglitch = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_read_zerocmp = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_read_sticky = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_read_scale = VL_RAND_RESET_I(4);
    vlSelf->scale_io_regs_cfg_write_ip_0_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_ip_1_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_ip_2_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_ip_3_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_gang_0_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_gang_1_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_gang_2_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_gang_3_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_extra_0_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_extra_1_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_extra_2_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_extra_3_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_center_0_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_center_1_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_center_2_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_center_3_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_running_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_countAlways_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_deglitch_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_zerocmp_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_sticky_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cfg_write_scale_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_countLo_write_valid = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_countLo_write_bits = VL_RAND_RESET_I(31);
    vlSelf->scale_io_regs_countLo_read = VL_RAND_RESET_I(31);
    vlSelf->scale_io_regs_s_read = VL_RAND_RESET_I(16);
    vlSelf->scale_io_regs_cmp_0_write_valid = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cmp_0_write_bits = VL_RAND_RESET_I(16);
    vlSelf->scale_io_regs_cmp_0_read = VL_RAND_RESET_I(16);
    vlSelf->scale_io_regs_cmp_1_write_valid = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cmp_1_write_bits = VL_RAND_RESET_I(16);
    vlSelf->scale_io_regs_cmp_1_read = VL_RAND_RESET_I(16);
    vlSelf->scale_io_regs_cmp_2_write_valid = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cmp_2_write_bits = VL_RAND_RESET_I(16);
    vlSelf->scale_io_regs_cmp_2_read = VL_RAND_RESET_I(16);
    vlSelf->scale_io_regs_cmp_3_write_valid = VL_RAND_RESET_I(1);
    vlSelf->scale_io_regs_cmp_3_write_bits = VL_RAND_RESET_I(16);
    vlSelf->scale_io_regs_cmp_3_read = VL_RAND_RESET_I(16);
    vlSelf->scale_io_ip_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_ip_1 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_ip_2 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_ip_3 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_gpio_0 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_gpio_1 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_gpio_2 = VL_RAND_RESET_I(1);
    vlSelf->scale_io_gpio_3 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__scale = VL_RAND_RESET_I(4);
    vlSelf->PWMTimer__DOT__cmp_0 = VL_RAND_RESET_I(16);
    vlSelf->PWMTimer__DOT__cmp_1 = VL_RAND_RESET_I(16);
    vlSelf->PWMTimer__DOT__cmp_2 = VL_RAND_RESET_I(16);
    vlSelf->PWMTimer__DOT__cmp_3 = VL_RAND_RESET_I(16);
    vlSelf->PWMTimer__DOT__small_0 = VL_RAND_RESET_I(5);
    vlSelf->PWMTimer__DOT__large_0 = VL_RAND_RESET_I(26);
    vlSelf->PWMTimer__DOT__elapsed_center_0 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__elapsed_center_1 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__elapsed_center_2 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__elapsed_center_3 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__countReset_zerocmp = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_doSticky_deglitch = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_doSticky_sticky = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_doSticky = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_0 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_1 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_2 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_3 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__gang_0 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__gang_1 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__gang_2 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__gang_3 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__extra_0 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__extra_1 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__extra_2 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__extra_3 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__io_regs_cfg_read_running_oneShot = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__io_regs_cfg_read_countAlways_countAlways = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__nextSmall = VL_RAND_RESET_I(6);
    vlSelf->PWMTimer__DOT__elapsed_0 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__elapsed_1 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__elapsed_2 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__elapsed_3 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__countReset = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_sel_0 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_sel_1 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_sel_2 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_sel_3 = VL_RAND_RESET_I(1);
    vlSelf->PWMTimer__DOT___io_regs_cfg_read_ip_0_ip_T_28 = VL_RAND_RESET_I(1);
    vlSelf->__Vtrigprevexpr___TOP__clock__0 = VL_RAND_RESET_I(1);
}
