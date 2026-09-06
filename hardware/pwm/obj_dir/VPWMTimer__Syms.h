// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Symbol table internal header
//
// Internal details; most calling programs do not need this header,
// unless using verilator public meta comments.

#ifndef VERILATED_VPWMTIMER__SYMS_H_
#define VERILATED_VPWMTIMER__SYMS_H_  // guard

#include "verilated.h"

// INCLUDE MODEL CLASS

#include "VPWMTimer.h"

// INCLUDE MODULE CLASSES
#include "VPWMTimer___024root.h"

// SYMS CLASS (contains all model state)
class alignas(VL_CACHE_LINE_BYTES)VPWMTimer__Syms final : public VerilatedSyms {
  public:
    // INTERNAL STATE
    VPWMTimer* const __Vm_modelp;
    VlDeleter __Vm_deleter;
    bool __Vm_didInit = false;

    // MODULE INSTANCE STATE
    VPWMTimer___024root            TOP;

    // CONSTRUCTORS
    VPWMTimer__Syms(VerilatedContext* contextp, const char* namep, VPWMTimer* modelp);
    ~VPWMTimer__Syms();

    // METHODS
    const char* name() { return TOP.name(); }
};

#endif  // guard
