// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Design implementation internals
// See VPWMTimer.h for the primary calling header

#include "VPWMTimer__pch.h"
#include "VPWMTimer__Syms.h"
#include "VPWMTimer___024root.h"

void VPWMTimer___024root___ctor_var_reset(VPWMTimer___024root* vlSelf);

VPWMTimer___024root::VPWMTimer___024root(VPWMTimer__Syms* symsp, const char* v__name)
    : VerilatedModule{v__name}
    , vlSymsp{symsp}
 {
    // Reset structure values
    VPWMTimer___024root___ctor_var_reset(this);
}

void VPWMTimer___024root::__Vconfigure(bool first) {
    (void)first;  // Prevent unused variable warning
}

VPWMTimer___024root::~VPWMTimer___024root() {
}
