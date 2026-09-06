// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Model implementation (design independent parts)

#include "VPWMTimer__pch.h"

//============================================================
// Constructors

VPWMTimer::VPWMTimer(VerilatedContext* _vcontextp__, const char* _vcname__)
    : VerilatedModel{*_vcontextp__}
    , vlSymsp{new VPWMTimer__Syms(contextp(), _vcname__, this)}
    , clock{vlSymsp->TOP.clock}
    , reset{vlSymsp->TOP.reset}
    , scale_io_regs_cfg_write_ip_0{vlSymsp->TOP.scale_io_regs_cfg_write_ip_0}
    , scale_io_regs_cfg_write_ip_1{vlSymsp->TOP.scale_io_regs_cfg_write_ip_1}
    , scale_io_regs_cfg_write_ip_2{vlSymsp->TOP.scale_io_regs_cfg_write_ip_2}
    , scale_io_regs_cfg_write_ip_3{vlSymsp->TOP.scale_io_regs_cfg_write_ip_3}
    , scale_io_regs_cfg_write_gang_0{vlSymsp->TOP.scale_io_regs_cfg_write_gang_0}
    , scale_io_regs_cfg_write_gang_1{vlSymsp->TOP.scale_io_regs_cfg_write_gang_1}
    , scale_io_regs_cfg_write_gang_2{vlSymsp->TOP.scale_io_regs_cfg_write_gang_2}
    , scale_io_regs_cfg_write_gang_3{vlSymsp->TOP.scale_io_regs_cfg_write_gang_3}
    , scale_io_regs_cfg_write_extra_0{vlSymsp->TOP.scale_io_regs_cfg_write_extra_0}
    , scale_io_regs_cfg_write_extra_1{vlSymsp->TOP.scale_io_regs_cfg_write_extra_1}
    , scale_io_regs_cfg_write_extra_2{vlSymsp->TOP.scale_io_regs_cfg_write_extra_2}
    , scale_io_regs_cfg_write_extra_3{vlSymsp->TOP.scale_io_regs_cfg_write_extra_3}
    , scale_io_regs_cfg_write_center_0{vlSymsp->TOP.scale_io_regs_cfg_write_center_0}
    , scale_io_regs_cfg_write_center_1{vlSymsp->TOP.scale_io_regs_cfg_write_center_1}
    , scale_io_regs_cfg_write_center_2{vlSymsp->TOP.scale_io_regs_cfg_write_center_2}
    , scale_io_regs_cfg_write_center_3{vlSymsp->TOP.scale_io_regs_cfg_write_center_3}
    , scale_io_regs_cfg_write_running{vlSymsp->TOP.scale_io_regs_cfg_write_running}
    , scale_io_regs_cfg_write_countAlways{vlSymsp->TOP.scale_io_regs_cfg_write_countAlways}
    , scale_io_regs_cfg_write_deglitch{vlSymsp->TOP.scale_io_regs_cfg_write_deglitch}
    , scale_io_regs_cfg_write_zerocmp{vlSymsp->TOP.scale_io_regs_cfg_write_zerocmp}
    , scale_io_regs_cfg_write_sticky{vlSymsp->TOP.scale_io_regs_cfg_write_sticky}
    , scale_io_regs_cfg_write_scale{vlSymsp->TOP.scale_io_regs_cfg_write_scale}
    , scale_io_regs_cfg_read_ip_0{vlSymsp->TOP.scale_io_regs_cfg_read_ip_0}
    , scale_io_regs_cfg_read_ip_1{vlSymsp->TOP.scale_io_regs_cfg_read_ip_1}
    , scale_io_regs_cfg_read_ip_2{vlSymsp->TOP.scale_io_regs_cfg_read_ip_2}
    , scale_io_regs_cfg_read_ip_3{vlSymsp->TOP.scale_io_regs_cfg_read_ip_3}
    , scale_io_regs_cfg_read_gang_0{vlSymsp->TOP.scale_io_regs_cfg_read_gang_0}
    , scale_io_regs_cfg_read_gang_1{vlSymsp->TOP.scale_io_regs_cfg_read_gang_1}
    , scale_io_regs_cfg_read_gang_2{vlSymsp->TOP.scale_io_regs_cfg_read_gang_2}
    , scale_io_regs_cfg_read_gang_3{vlSymsp->TOP.scale_io_regs_cfg_read_gang_3}
    , scale_io_regs_cfg_read_extra_0{vlSymsp->TOP.scale_io_regs_cfg_read_extra_0}
    , scale_io_regs_cfg_read_extra_1{vlSymsp->TOP.scale_io_regs_cfg_read_extra_1}
    , scale_io_regs_cfg_read_extra_2{vlSymsp->TOP.scale_io_regs_cfg_read_extra_2}
    , scale_io_regs_cfg_read_extra_3{vlSymsp->TOP.scale_io_regs_cfg_read_extra_3}
    , scale_io_regs_cfg_read_center_0{vlSymsp->TOP.scale_io_regs_cfg_read_center_0}
    , scale_io_regs_cfg_read_center_1{vlSymsp->TOP.scale_io_regs_cfg_read_center_1}
    , scale_io_regs_cfg_read_center_2{vlSymsp->TOP.scale_io_regs_cfg_read_center_2}
    , scale_io_regs_cfg_read_center_3{vlSymsp->TOP.scale_io_regs_cfg_read_center_3}
    , scale_io_regs_cfg_read_running{vlSymsp->TOP.scale_io_regs_cfg_read_running}
    , scale_io_regs_cfg_read_countAlways{vlSymsp->TOP.scale_io_regs_cfg_read_countAlways}
    , scale_io_regs_cfg_read_deglitch{vlSymsp->TOP.scale_io_regs_cfg_read_deglitch}
    , scale_io_regs_cfg_read_zerocmp{vlSymsp->TOP.scale_io_regs_cfg_read_zerocmp}
    , scale_io_regs_cfg_read_sticky{vlSymsp->TOP.scale_io_regs_cfg_read_sticky}
    , scale_io_regs_cfg_read_scale{vlSymsp->TOP.scale_io_regs_cfg_read_scale}
    , scale_io_regs_cfg_write_ip_0_0{vlSymsp->TOP.scale_io_regs_cfg_write_ip_0_0}
    , scale_io_regs_cfg_write_ip_1_0{vlSymsp->TOP.scale_io_regs_cfg_write_ip_1_0}
    , scale_io_regs_cfg_write_ip_2_0{vlSymsp->TOP.scale_io_regs_cfg_write_ip_2_0}
    , scale_io_regs_cfg_write_ip_3_0{vlSymsp->TOP.scale_io_regs_cfg_write_ip_3_0}
    , scale_io_regs_cfg_write_gang_0_0{vlSymsp->TOP.scale_io_regs_cfg_write_gang_0_0}
    , scale_io_regs_cfg_write_gang_1_0{vlSymsp->TOP.scale_io_regs_cfg_write_gang_1_0}
    , scale_io_regs_cfg_write_gang_2_0{vlSymsp->TOP.scale_io_regs_cfg_write_gang_2_0}
    , scale_io_regs_cfg_write_gang_3_0{vlSymsp->TOP.scale_io_regs_cfg_write_gang_3_0}
    , scale_io_regs_cfg_write_extra_0_0{vlSymsp->TOP.scale_io_regs_cfg_write_extra_0_0}
    , scale_io_regs_cfg_write_extra_1_0{vlSymsp->TOP.scale_io_regs_cfg_write_extra_1_0}
    , scale_io_regs_cfg_write_extra_2_0{vlSymsp->TOP.scale_io_regs_cfg_write_extra_2_0}
    , scale_io_regs_cfg_write_extra_3_0{vlSymsp->TOP.scale_io_regs_cfg_write_extra_3_0}
    , scale_io_regs_cfg_write_center_0_0{vlSymsp->TOP.scale_io_regs_cfg_write_center_0_0}
    , scale_io_regs_cfg_write_center_1_0{vlSymsp->TOP.scale_io_regs_cfg_write_center_1_0}
    , scale_io_regs_cfg_write_center_2_0{vlSymsp->TOP.scale_io_regs_cfg_write_center_2_0}
    , scale_io_regs_cfg_write_center_3_0{vlSymsp->TOP.scale_io_regs_cfg_write_center_3_0}
    , scale_io_regs_cfg_write_running_0{vlSymsp->TOP.scale_io_regs_cfg_write_running_0}
    , scale_io_regs_cfg_write_countAlways_0{vlSymsp->TOP.scale_io_regs_cfg_write_countAlways_0}
    , scale_io_regs_cfg_write_deglitch_0{vlSymsp->TOP.scale_io_regs_cfg_write_deglitch_0}
    , scale_io_regs_cfg_write_zerocmp_0{vlSymsp->TOP.scale_io_regs_cfg_write_zerocmp_0}
    , scale_io_regs_cfg_write_sticky_0{vlSymsp->TOP.scale_io_regs_cfg_write_sticky_0}
    , scale_io_regs_cfg_write_scale_0{vlSymsp->TOP.scale_io_regs_cfg_write_scale_0}
    , scale_io_regs_countLo_write_valid{vlSymsp->TOP.scale_io_regs_countLo_write_valid}
    , scale_io_regs_cmp_0_write_valid{vlSymsp->TOP.scale_io_regs_cmp_0_write_valid}
    , scale_io_regs_cmp_1_write_valid{vlSymsp->TOP.scale_io_regs_cmp_1_write_valid}
    , scale_io_regs_cmp_2_write_valid{vlSymsp->TOP.scale_io_regs_cmp_2_write_valid}
    , scale_io_regs_cmp_3_write_valid{vlSymsp->TOP.scale_io_regs_cmp_3_write_valid}
    , scale_io_ip_0{vlSymsp->TOP.scale_io_ip_0}
    , scale_io_ip_1{vlSymsp->TOP.scale_io_ip_1}
    , scale_io_ip_2{vlSymsp->TOP.scale_io_ip_2}
    , scale_io_ip_3{vlSymsp->TOP.scale_io_ip_3}
    , scale_io_gpio_0{vlSymsp->TOP.scale_io_gpio_0}
    , scale_io_gpio_1{vlSymsp->TOP.scale_io_gpio_1}
    , scale_io_gpio_2{vlSymsp->TOP.scale_io_gpio_2}
    , scale_io_gpio_3{vlSymsp->TOP.scale_io_gpio_3}
    , scale_io_regs_s_read{vlSymsp->TOP.scale_io_regs_s_read}
    , scale_io_regs_cmp_0_write_bits{vlSymsp->TOP.scale_io_regs_cmp_0_write_bits}
    , scale_io_regs_cmp_0_read{vlSymsp->TOP.scale_io_regs_cmp_0_read}
    , scale_io_regs_cmp_1_write_bits{vlSymsp->TOP.scale_io_regs_cmp_1_write_bits}
    , scale_io_regs_cmp_1_read{vlSymsp->TOP.scale_io_regs_cmp_1_read}
    , scale_io_regs_cmp_2_write_bits{vlSymsp->TOP.scale_io_regs_cmp_2_write_bits}
    , scale_io_regs_cmp_2_read{vlSymsp->TOP.scale_io_regs_cmp_2_read}
    , scale_io_regs_cmp_3_write_bits{vlSymsp->TOP.scale_io_regs_cmp_3_write_bits}
    , scale_io_regs_cmp_3_read{vlSymsp->TOP.scale_io_regs_cmp_3_read}
    , scale_io_regs_countLo_write_bits{vlSymsp->TOP.scale_io_regs_countLo_write_bits}
    , scale_io_regs_countLo_read{vlSymsp->TOP.scale_io_regs_countLo_read}
    , rootp{&(vlSymsp->TOP)}
{
    // Register model with the context
    contextp()->addModel(this);
}

VPWMTimer::VPWMTimer(const char* _vcname__)
    : VPWMTimer(Verilated::threadContextp(), _vcname__)
{
}

//============================================================
// Destructor

VPWMTimer::~VPWMTimer() {
    delete vlSymsp;
}

//============================================================
// Evaluation function

#ifdef VL_DEBUG
void VPWMTimer___024root___eval_debug_assertions(VPWMTimer___024root* vlSelf);
#endif  // VL_DEBUG
void VPWMTimer___024root___eval_static(VPWMTimer___024root* vlSelf);
void VPWMTimer___024root___eval_initial(VPWMTimer___024root* vlSelf);
void VPWMTimer___024root___eval_settle(VPWMTimer___024root* vlSelf);
void VPWMTimer___024root___eval(VPWMTimer___024root* vlSelf);

void VPWMTimer::eval_step() {
    VL_DEBUG_IF(VL_DBG_MSGF("+++++TOP Evaluate VPWMTimer::eval_step\n"); );
#ifdef VL_DEBUG
    // Debug assertions
    VPWMTimer___024root___eval_debug_assertions(&(vlSymsp->TOP));
#endif  // VL_DEBUG
    vlSymsp->__Vm_deleter.deleteAll();
    if (VL_UNLIKELY(!vlSymsp->__Vm_didInit)) {
        vlSymsp->__Vm_didInit = true;
        VL_DEBUG_IF(VL_DBG_MSGF("+ Initial\n"););
        VPWMTimer___024root___eval_static(&(vlSymsp->TOP));
        VPWMTimer___024root___eval_initial(&(vlSymsp->TOP));
        VPWMTimer___024root___eval_settle(&(vlSymsp->TOP));
    }
    VL_DEBUG_IF(VL_DBG_MSGF("+ Eval\n"););
    VPWMTimer___024root___eval(&(vlSymsp->TOP));
    // Evaluate cleanup
    Verilated::endOfEval(vlSymsp->__Vm_evalMsgQp);
}

//============================================================
// Events and timing
bool VPWMTimer::eventsPending() { return false; }

uint64_t VPWMTimer::nextTimeSlot() {
    VL_FATAL_MT(__FILE__, __LINE__, "", "%Error: No delays in the design");
    return 0;
}

//============================================================
// Utilities

const char* VPWMTimer::name() const {
    return vlSymsp->name();
}

//============================================================
// Invoke final blocks

void VPWMTimer___024root___eval_final(VPWMTimer___024root* vlSelf);

VL_ATTR_COLD void VPWMTimer::final() {
    VPWMTimer___024root___eval_final(&(vlSymsp->TOP));
}

//============================================================
// Implementations of abstract methods from VerilatedModel

const char* VPWMTimer::hierName() const { return vlSymsp->name(); }
const char* VPWMTimer::modelName() const { return "VPWMTimer"; }
unsigned VPWMTimer::threads() const { return 1; }
void VPWMTimer::prepareClone() const { contextp()->prepareClone(); }
void VPWMTimer::atClone() const {
    contextp()->threadPoolpOnClone();
}

//============================================================
// Trace configuration

VL_ATTR_COLD void VPWMTimer::trace(VerilatedVcdC* tfp, int levels, int options) {
    vl_fatal(__FILE__, __LINE__, __FILE__,"'VPWMTimer::trace()' called on model that was Verilated without --trace option");
}
