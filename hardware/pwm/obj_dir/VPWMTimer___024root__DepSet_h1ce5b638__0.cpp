// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Design implementation internals
// See VPWMTimer.h for the primary calling header

#include "VPWMTimer__pch.h"
#include "VPWMTimer___024root.h"

VL_INLINE_OPT void VPWMTimer___024root___ico_sequent__TOP__0(VPWMTimer___024root* vlSelf) {
    (void)vlSelf;  // Prevent unused variable warning
    VPWMTimer__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    VL_DEBUG_IF(VL_DBG_MSGF("+    VPWMTimer___024root___ico_sequent__TOP__0\n"); );
    // Body
    vlSelf->PWMTimer__DOT___io_regs_cfg_read_ip_0_ip_T_28 
        = ((IData)(vlSelf->scale_io_regs_cfg_write_ip_0_0) 
           | ((IData)(vlSelf->scale_io_regs_cfg_write_ip_1_0) 
              | ((IData)(vlSelf->scale_io_regs_cfg_write_ip_2_0) 
                 | (IData)(vlSelf->scale_io_regs_cfg_write_ip_3_0))));
}

void VPWMTimer___024root___eval_ico(VPWMTimer___024root* vlSelf) {
    (void)vlSelf;  // Prevent unused variable warning
    VPWMTimer__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    VL_DEBUG_IF(VL_DBG_MSGF("+    VPWMTimer___024root___eval_ico\n"); );
    // Body
    if ((1ULL & vlSelf->__VicoTriggered.word(0U))) {
        VPWMTimer___024root___ico_sequent__TOP__0(vlSelf);
    }
}

void VPWMTimer___024root___eval_triggers__ico(VPWMTimer___024root* vlSelf);

bool VPWMTimer___024root___eval_phase__ico(VPWMTimer___024root* vlSelf) {
    (void)vlSelf;  // Prevent unused variable warning
    VPWMTimer__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    VL_DEBUG_IF(VL_DBG_MSGF("+    VPWMTimer___024root___eval_phase__ico\n"); );
    // Init
    CData/*0:0*/ __VicoExecute;
    // Body
    VPWMTimer___024root___eval_triggers__ico(vlSelf);
    __VicoExecute = vlSelf->__VicoTriggered.any();
    if (__VicoExecute) {
        VPWMTimer___024root___eval_ico(vlSelf);
    }
    return (__VicoExecute);
}

void VPWMTimer___024root___eval_act(VPWMTimer___024root* vlSelf) {
    (void)vlSelf;  // Prevent unused variable warning
    VPWMTimer__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    VL_DEBUG_IF(VL_DBG_MSGF("+    VPWMTimer___024root___eval_act\n"); );
}

VL_INLINE_OPT void VPWMTimer___024root___nba_sequent__TOP__0(VPWMTimer___024root* vlSelf) {
    (void)vlSelf;  // Prevent unused variable warning
    VPWMTimer__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    VL_DEBUG_IF(VL_DBG_MSGF("+    VPWMTimer___024root___nba_sequent__TOP__0\n"); );
    // Init
    IData/*30:0*/ PWMTimer__DOT___s_T;
    PWMTimer__DOT___s_T = 0;
    CData/*3:0*/ PWMTimer__DOT___GEN;
    PWMTimer__DOT___GEN = 0;
    IData/*25:0*/ __Vdly__PWMTimer__DOT__large_0;
    __Vdly__PWMTimer__DOT__large_0 = 0;
    // Body
    __Vdly__PWMTimer__DOT__large_0 = vlSelf->PWMTimer__DOT__large_0;
    if (vlSelf->reset) {
        __Vdly__PWMTimer__DOT__large_0 = 0U;
        vlSelf->PWMTimer__DOT__io_regs_cfg_read_countAlways_countAlways = 0U;
        vlSelf->PWMTimer__DOT__io_regs_cfg_read_running_oneShot = 0U;
        vlSelf->PWMTimer__DOT__extra_2 = 0U;
        vlSelf->PWMTimer__DOT__extra_0 = 0U;
        vlSelf->PWMTimer__DOT__extra_3 = 0U;
        vlSelf->PWMTimer__DOT__extra_1 = 0U;
        vlSelf->PWMTimer__DOT__small_0 = 0U;
    } else {
        if (vlSelf->scale_io_regs_countLo_write_valid) {
            __Vdly__PWMTimer__DOT__large_0 = (0x3ffffffU 
                                              & (vlSelf->scale_io_regs_countLo_write_bits 
                                                 >> 5U));
            vlSelf->PWMTimer__DOT__small_0 = (0x1fU 
                                              & vlSelf->scale_io_regs_countLo_write_bits);
        } else if (vlSelf->PWMTimer__DOT__countReset) {
            __Vdly__PWMTimer__DOT__large_0 = 0U;
            vlSelf->PWMTimer__DOT__small_0 = (0x1fU 
                                              & 0U);
        } else {
            if ((0x20U & (IData)(vlSelf->PWMTimer__DOT__nextSmall))) {
                __Vdly__PWMTimer__DOT__large_0 = (0x3ffffffU 
                                                  & ((IData)(1U) 
                                                     + vlSelf->PWMTimer__DOT__large_0));
            }
            vlSelf->PWMTimer__DOT__small_0 = (0x1fU 
                                              & (IData)(vlSelf->PWMTimer__DOT__nextSmall));
        }
        if (vlSelf->scale_io_regs_cfg_write_countAlways_0) {
            vlSelf->PWMTimer__DOT__io_regs_cfg_read_countAlways_countAlways 
                = vlSelf->scale_io_regs_cfg_write_countAlways;
        }
        if (((IData)(vlSelf->scale_io_regs_cfg_write_running_0) 
             | (IData)(vlSelf->PWMTimer__DOT__countReset))) {
            vlSelf->PWMTimer__DOT__io_regs_cfg_read_running_oneShot 
                = ((IData)(vlSelf->scale_io_regs_cfg_write_running) 
                   & (~ (IData)(vlSelf->PWMTimer__DOT__countReset)));
        }
        if (((((IData)(vlSelf->scale_io_regs_cfg_write_extra_0_0) 
               | (IData)(vlSelf->scale_io_regs_cfg_write_extra_1_0)) 
              | (IData)(vlSelf->scale_io_regs_cfg_write_extra_2_0)) 
             | (IData)(vlSelf->scale_io_regs_cfg_write_extra_3_0))) {
            vlSelf->PWMTimer__DOT__extra_2 = vlSelf->scale_io_regs_cfg_write_extra_2;
            vlSelf->PWMTimer__DOT__extra_0 = vlSelf->scale_io_regs_cfg_write_extra_0;
            vlSelf->PWMTimer__DOT__extra_3 = vlSelf->scale_io_regs_cfg_write_extra_3;
            vlSelf->PWMTimer__DOT__extra_1 = vlSelf->scale_io_regs_cfg_write_extra_1;
        }
    }
    if (vlSelf->PWMTimer__DOT___io_regs_cfg_read_ip_0_ip_T_28) {
        vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_0 
            = vlSelf->scale_io_regs_cfg_write_ip_0;
        vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_1 
            = vlSelf->scale_io_regs_cfg_write_ip_1;
        vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_2 
            = vlSelf->scale_io_regs_cfg_write_ip_2;
        vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_3 
            = vlSelf->scale_io_regs_cfg_write_ip_3;
    } else {
        vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_0 
            = (((IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_sel_0) 
                & (IData)(vlSelf->PWMTimer__DOT__elapsed_0)) 
               | ((~ (IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_sel_0)) 
                  & ((IData)(vlSelf->PWMTimer__DOT__elapsed_0) 
                     | ((IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_doSticky) 
                        & (IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_0)))));
        vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_1 
            = (((IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_sel_1) 
                & (IData)(vlSelf->PWMTimer__DOT__elapsed_1)) 
               | ((~ (IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_sel_1)) 
                  & ((IData)(vlSelf->PWMTimer__DOT__elapsed_1) 
                     | ((IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_doSticky) 
                        & (IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_1)))));
        vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_2 
            = (((IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_sel_2) 
                & (IData)(vlSelf->PWMTimer__DOT__elapsed_2)) 
               | ((~ (IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_sel_2)) 
                  & ((IData)(vlSelf->PWMTimer__DOT__elapsed_2) 
                     | ((IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_doSticky) 
                        & (IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_2)))));
        vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_3 
            = (((IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_sel_3) 
                & (IData)(vlSelf->PWMTimer__DOT__elapsed_3)) 
               | ((~ (IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_sel_3)) 
                  & ((IData)(vlSelf->PWMTimer__DOT__elapsed_3) 
                     | ((IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_doSticky) 
                        & (IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_3)))));
    }
    if (vlSelf->scale_io_regs_cmp_2_write_valid) {
        vlSelf->PWMTimer__DOT__cmp_2 = vlSelf->scale_io_regs_cmp_2_write_bits;
    }
    if (vlSelf->scale_io_regs_cmp_1_write_valid) {
        vlSelf->PWMTimer__DOT__cmp_1 = vlSelf->scale_io_regs_cmp_1_write_bits;
    }
    if (vlSelf->scale_io_regs_cmp_3_write_valid) {
        vlSelf->PWMTimer__DOT__cmp_3 = vlSelf->scale_io_regs_cmp_3_write_bits;
    }
    if (vlSelf->scale_io_regs_cfg_write_zerocmp_0) {
        vlSelf->PWMTimer__DOT__countReset_zerocmp = vlSelf->scale_io_regs_cfg_write_zerocmp;
    }
    if (vlSelf->scale_io_regs_cmp_0_write_valid) {
        vlSelf->PWMTimer__DOT__cmp_0 = vlSelf->scale_io_regs_cmp_0_write_bits;
    }
    if (((((IData)(vlSelf->scale_io_regs_cfg_write_center_0_0) 
           | (IData)(vlSelf->scale_io_regs_cfg_write_center_1_0)) 
          | (IData)(vlSelf->scale_io_regs_cfg_write_center_2_0)) 
         | (IData)(vlSelf->scale_io_regs_cfg_write_center_3_0))) {
        vlSelf->PWMTimer__DOT__elapsed_center_3 = vlSelf->scale_io_regs_cfg_write_center_3;
        vlSelf->PWMTimer__DOT__elapsed_center_2 = vlSelf->scale_io_regs_cfg_write_center_2;
        vlSelf->PWMTimer__DOT__elapsed_center_1 = vlSelf->scale_io_regs_cfg_write_center_1;
        vlSelf->PWMTimer__DOT__elapsed_center_0 = vlSelf->scale_io_regs_cfg_write_center_0;
    }
    if (((((IData)(vlSelf->scale_io_regs_cfg_write_gang_0_0) 
           | (IData)(vlSelf->scale_io_regs_cfg_write_gang_1_0)) 
          | (IData)(vlSelf->scale_io_regs_cfg_write_gang_2_0)) 
         | (IData)(vlSelf->scale_io_regs_cfg_write_gang_3_0))) {
        vlSelf->PWMTimer__DOT__gang_2 = vlSelf->scale_io_regs_cfg_write_gang_2;
        vlSelf->PWMTimer__DOT__gang_1 = vlSelf->scale_io_regs_cfg_write_gang_1;
        vlSelf->PWMTimer__DOT__gang_3 = vlSelf->scale_io_regs_cfg_write_gang_3;
        vlSelf->PWMTimer__DOT__gang_0 = vlSelf->scale_io_regs_cfg_write_gang_0;
    }
    if (vlSelf->scale_io_regs_cfg_write_scale_0) {
        vlSelf->PWMTimer__DOT__scale = vlSelf->scale_io_regs_cfg_write_scale;
    }
    vlSelf->PWMTimer__DOT__large_0 = __Vdly__PWMTimer__DOT__large_0;
    vlSelf->scale_io_regs_cfg_read_ip_0 = vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_0;
    vlSelf->scale_io_regs_cfg_read_ip_1 = vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_1;
    vlSelf->scale_io_regs_cfg_read_ip_2 = vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_2;
    vlSelf->scale_io_regs_cfg_read_ip_3 = vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_3;
    vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_doSticky 
        = (((IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_doSticky_deglitch) 
            & (~ (IData)(vlSelf->PWMTimer__DOT__countReset))) 
           | (IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_doSticky_sticky));
    vlSelf->scale_io_regs_cmp_2_read = vlSelf->PWMTimer__DOT__cmp_2;
    vlSelf->scale_io_regs_cmp_1_read = vlSelf->PWMTimer__DOT__cmp_1;
    vlSelf->scale_io_regs_cmp_3_read = vlSelf->PWMTimer__DOT__cmp_3;
    vlSelf->scale_io_regs_cfg_read_zerocmp = vlSelf->PWMTimer__DOT__countReset_zerocmp;
    vlSelf->scale_io_regs_cmp_0_read = vlSelf->PWMTimer__DOT__cmp_0;
    vlSelf->scale_io_regs_cfg_read_countAlways = vlSelf->PWMTimer__DOT__io_regs_cfg_read_countAlways_countAlways;
    vlSelf->scale_io_regs_cfg_read_center_3 = vlSelf->PWMTimer__DOT__elapsed_center_3;
    vlSelf->scale_io_regs_cfg_read_center_2 = vlSelf->PWMTimer__DOT__elapsed_center_2;
    vlSelf->scale_io_regs_cfg_read_center_1 = vlSelf->PWMTimer__DOT__elapsed_center_1;
    vlSelf->scale_io_regs_cfg_read_running = vlSelf->PWMTimer__DOT__io_regs_cfg_read_running_oneShot;
    vlSelf->scale_io_regs_cfg_read_center_0 = vlSelf->PWMTimer__DOT__elapsed_center_0;
    vlSelf->scale_io_regs_cfg_read_gang_2 = vlSelf->PWMTimer__DOT__gang_2;
    vlSelf->scale_io_regs_cfg_read_gang_1 = vlSelf->PWMTimer__DOT__gang_1;
    vlSelf->scale_io_regs_cfg_read_gang_3 = vlSelf->PWMTimer__DOT__gang_3;
    vlSelf->scale_io_regs_cfg_read_gang_0 = vlSelf->PWMTimer__DOT__gang_0;
    vlSelf->scale_io_regs_cfg_read_extra_2 = vlSelf->PWMTimer__DOT__extra_2;
    vlSelf->scale_io_regs_cfg_read_extra_0 = vlSelf->PWMTimer__DOT__extra_0;
    vlSelf->scale_io_regs_cfg_read_extra_3 = vlSelf->PWMTimer__DOT__extra_3;
    vlSelf->scale_io_regs_cfg_read_extra_1 = vlSelf->PWMTimer__DOT__extra_1;
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
    vlSelf->scale_io_regs_cfg_read_scale = vlSelf->PWMTimer__DOT__scale;
    vlSelf->PWMTimer__DOT__nextSmall = (0x3fU & ((IData)(vlSelf->PWMTimer__DOT__small_0) 
                                                 + 
                                                 ((IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_countAlways_countAlways) 
                                                  | (IData)(vlSelf->PWMTimer__DOT__io_regs_cfg_read_running_oneShot))));
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
    if (vlSelf->scale_io_regs_cfg_write_deglitch_0) {
        vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_doSticky_deglitch 
            = vlSelf->scale_io_regs_cfg_write_deglitch;
    }
    if (vlSelf->scale_io_regs_cfg_write_sticky_0) {
        vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_doSticky_sticky 
            = vlSelf->scale_io_regs_cfg_write_sticky;
    }
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
    vlSelf->scale_io_regs_cfg_read_deglitch = vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_doSticky_deglitch;
    vlSelf->scale_io_regs_cfg_read_sticky = vlSelf->PWMTimer__DOT__io_regs_cfg_read_ip_0_ip_doSticky_sticky;
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

void VPWMTimer___024root___eval_nba(VPWMTimer___024root* vlSelf) {
    (void)vlSelf;  // Prevent unused variable warning
    VPWMTimer__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    VL_DEBUG_IF(VL_DBG_MSGF("+    VPWMTimer___024root___eval_nba\n"); );
    // Body
    if ((1ULL & vlSelf->__VnbaTriggered.word(0U))) {
        VPWMTimer___024root___nba_sequent__TOP__0(vlSelf);
    }
}

void VPWMTimer___024root___eval_triggers__act(VPWMTimer___024root* vlSelf);

bool VPWMTimer___024root___eval_phase__act(VPWMTimer___024root* vlSelf) {
    (void)vlSelf;  // Prevent unused variable warning
    VPWMTimer__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    VL_DEBUG_IF(VL_DBG_MSGF("+    VPWMTimer___024root___eval_phase__act\n"); );
    // Init
    VlTriggerVec<1> __VpreTriggered;
    CData/*0:0*/ __VactExecute;
    // Body
    VPWMTimer___024root___eval_triggers__act(vlSelf);
    __VactExecute = vlSelf->__VactTriggered.any();
    if (__VactExecute) {
        __VpreTriggered.andNot(vlSelf->__VactTriggered, vlSelf->__VnbaTriggered);
        vlSelf->__VnbaTriggered.thisOr(vlSelf->__VactTriggered);
        VPWMTimer___024root___eval_act(vlSelf);
    }
    return (__VactExecute);
}

bool VPWMTimer___024root___eval_phase__nba(VPWMTimer___024root* vlSelf) {
    (void)vlSelf;  // Prevent unused variable warning
    VPWMTimer__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    VL_DEBUG_IF(VL_DBG_MSGF("+    VPWMTimer___024root___eval_phase__nba\n"); );
    // Init
    CData/*0:0*/ __VnbaExecute;
    // Body
    __VnbaExecute = vlSelf->__VnbaTriggered.any();
    if (__VnbaExecute) {
        VPWMTimer___024root___eval_nba(vlSelf);
        vlSelf->__VnbaTriggered.clear();
    }
    return (__VnbaExecute);
}

#ifdef VL_DEBUG
VL_ATTR_COLD void VPWMTimer___024root___dump_triggers__ico(VPWMTimer___024root* vlSelf);
#endif  // VL_DEBUG
#ifdef VL_DEBUG
VL_ATTR_COLD void VPWMTimer___024root___dump_triggers__nba(VPWMTimer___024root* vlSelf);
#endif  // VL_DEBUG
#ifdef VL_DEBUG
VL_ATTR_COLD void VPWMTimer___024root___dump_triggers__act(VPWMTimer___024root* vlSelf);
#endif  // VL_DEBUG

void VPWMTimer___024root___eval(VPWMTimer___024root* vlSelf) {
    (void)vlSelf;  // Prevent unused variable warning
    VPWMTimer__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    VL_DEBUG_IF(VL_DBG_MSGF("+    VPWMTimer___024root___eval\n"); );
    // Init
    IData/*31:0*/ __VicoIterCount;
    CData/*0:0*/ __VicoContinue;
    IData/*31:0*/ __VnbaIterCount;
    CData/*0:0*/ __VnbaContinue;
    // Body
    __VicoIterCount = 0U;
    vlSelf->__VicoFirstIteration = 1U;
    __VicoContinue = 1U;
    while (__VicoContinue) {
        if (VL_UNLIKELY((0x64U < __VicoIterCount))) {
#ifdef VL_DEBUG
            VPWMTimer___024root___dump_triggers__ico(vlSelf);
#endif
            VL_FATAL_MT("/scratch2/loren/riskybird-dev/backends/chipyard/fpga/generated-src/chipyard.fpga.arty200t.Arty200THarness.RocketArty200TMotorSpadConfig/gen-collateral/PWMTimer.sv", 46, "", "Input combinational region did not converge.");
        }
        __VicoIterCount = ((IData)(1U) + __VicoIterCount);
        __VicoContinue = 0U;
        if (VPWMTimer___024root___eval_phase__ico(vlSelf)) {
            __VicoContinue = 1U;
        }
        vlSelf->__VicoFirstIteration = 0U;
    }
    __VnbaIterCount = 0U;
    __VnbaContinue = 1U;
    while (__VnbaContinue) {
        if (VL_UNLIKELY((0x64U < __VnbaIterCount))) {
#ifdef VL_DEBUG
            VPWMTimer___024root___dump_triggers__nba(vlSelf);
#endif
            VL_FATAL_MT("/scratch2/loren/riskybird-dev/backends/chipyard/fpga/generated-src/chipyard.fpga.arty200t.Arty200THarness.RocketArty200TMotorSpadConfig/gen-collateral/PWMTimer.sv", 46, "", "NBA region did not converge.");
        }
        __VnbaIterCount = ((IData)(1U) + __VnbaIterCount);
        __VnbaContinue = 0U;
        vlSelf->__VactIterCount = 0U;
        vlSelf->__VactContinue = 1U;
        while (vlSelf->__VactContinue) {
            if (VL_UNLIKELY((0x64U < vlSelf->__VactIterCount))) {
#ifdef VL_DEBUG
                VPWMTimer___024root___dump_triggers__act(vlSelf);
#endif
                VL_FATAL_MT("/scratch2/loren/riskybird-dev/backends/chipyard/fpga/generated-src/chipyard.fpga.arty200t.Arty200THarness.RocketArty200TMotorSpadConfig/gen-collateral/PWMTimer.sv", 46, "", "Active region did not converge.");
            }
            vlSelf->__VactIterCount = ((IData)(1U) 
                                       + vlSelf->__VactIterCount);
            vlSelf->__VactContinue = 0U;
            if (VPWMTimer___024root___eval_phase__act(vlSelf)) {
                vlSelf->__VactContinue = 1U;
            }
        }
        if (VPWMTimer___024root___eval_phase__nba(vlSelf)) {
            __VnbaContinue = 1U;
        }
    }
}

#ifdef VL_DEBUG
void VPWMTimer___024root___eval_debug_assertions(VPWMTimer___024root* vlSelf) {
    (void)vlSelf;  // Prevent unused variable warning
    VPWMTimer__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    VL_DEBUG_IF(VL_DBG_MSGF("+    VPWMTimer___024root___eval_debug_assertions\n"); );
    // Body
    if (VL_UNLIKELY((vlSelf->clock & 0xfeU))) {
        Verilated::overWidthError("clock");}
    if (VL_UNLIKELY((vlSelf->reset & 0xfeU))) {
        Verilated::overWidthError("reset");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_ip_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_ip_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_ip_1 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_ip_1");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_ip_2 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_ip_2");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_ip_3 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_ip_3");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_gang_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_gang_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_gang_1 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_gang_1");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_gang_2 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_gang_2");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_gang_3 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_gang_3");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_extra_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_extra_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_extra_1 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_extra_1");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_extra_2 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_extra_2");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_extra_3 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_extra_3");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_center_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_center_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_center_1 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_center_1");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_center_2 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_center_2");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_center_3 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_center_3");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_running 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_running");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_countAlways 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_countAlways");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_deglitch 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_deglitch");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_zerocmp 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_zerocmp");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_sticky 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_sticky");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_scale 
                     & 0xf0U))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_scale");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_ip_0_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_ip_0_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_ip_1_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_ip_1_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_ip_2_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_ip_2_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_ip_3_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_ip_3_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_gang_0_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_gang_0_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_gang_1_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_gang_1_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_gang_2_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_gang_2_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_gang_3_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_gang_3_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_extra_0_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_extra_0_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_extra_1_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_extra_1_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_extra_2_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_extra_2_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_extra_3_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_extra_3_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_center_0_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_center_0_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_center_1_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_center_1_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_center_2_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_center_2_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_center_3_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_center_3_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_running_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_running_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_countAlways_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_countAlways_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_deglitch_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_deglitch_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_zerocmp_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_zerocmp_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_sticky_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_sticky_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cfg_write_scale_0 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cfg_write_scale_0");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_countLo_write_valid 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_countLo_write_valid");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_countLo_write_bits 
                     & 0x80000000U))) {
        Verilated::overWidthError("scale_io_regs_countLo_write_bits");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cmp_0_write_valid 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cmp_0_write_valid");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cmp_1_write_valid 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cmp_1_write_valid");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cmp_2_write_valid 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cmp_2_write_valid");}
    if (VL_UNLIKELY((vlSelf->scale_io_regs_cmp_3_write_valid 
                     & 0xfeU))) {
        Verilated::overWidthError("scale_io_regs_cmp_3_write_valid");}
}
#endif  // VL_DEBUG
