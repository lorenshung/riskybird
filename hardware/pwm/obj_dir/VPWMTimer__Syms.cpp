// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Symbol table implementation internals

#include "VPWMTimer__pch.h"
#include "VPWMTimer.h"
#include "VPWMTimer___024root.h"

// FUNCTIONS
VPWMTimer__Syms::~VPWMTimer__Syms()
{
}

VPWMTimer__Syms::VPWMTimer__Syms(VerilatedContext* contextp, const char* namep, VPWMTimer* modelp)
    : VerilatedSyms{contextp}
    // Setup internal state of the Syms class
    , __Vm_modelp{modelp}
    // Setup module instances
    , TOP{this, namep}
{
        // Check resources
        Verilated::stackCheck(53);
    // Configure time unit / time precision
    _vm_contextp__->timeunit(-12);
    _vm_contextp__->timeprecision(-12);
    // Setup each module's pointers to their submodules
    // Setup each module's pointer back to symbol table (for public functions)
    TOP.__Vconfigure(true);
}
