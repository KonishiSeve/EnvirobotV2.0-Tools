/*
 * python_link.cpp
 * C++ wrapper for the C++ CPG class, used as an interface to be called by the python code (CPP_CPG.py file)
 *
 *  Created on: Oct 29, 2024
 *      Author: Séverin Konishi
*/

#include <stdint.h>
#include <stdio.h>
#include "CPG.hpp"
#include <string.h>

CPG cpg;

//Initialized the CPG controller
extern "C" void python_cpg_init(uint8_t nb_modules,
                            float frequency,
                            float direction,
                            float amplc,
                            float amplh,
                            float nwave,
                            float coupling_strength,
                            float a_r) {
    cpg.init(nb_modules,frequency,direction,amplc,amplh,nwave,coupling_strength,a_r);
}

//Compute a step of the CPG controller and returns the joint setpoints
extern "C" void python_cpg_step(int8_t *output, float delta_ms) {
    cpg.step(output, delta_ms);
}

//Return the all the states of the CPG controller
extern "C" void python_cpg_states(float* osc_r, float* osc_dr, float* osc_ddr, float* osc_theta, float* osc_dtheta) {
    for(uint8_t i=0;i<cpg.number_oscillators;i++) {
        osc_r[i] = cpg.osc_r[i];
        osc_dr[i] = cpg.osc_dr[i];
        osc_ddr[i] = cpg.osc_ddr[i];
        osc_theta[i] = cpg.osc_theta[i];
        osc_dtheta[i] = cpg.osc_dtheta[i];
    }
}

extern "C" void python_cpg_reset() {
    cpg.reset();
}

extern "C" void python_cpg_number_modules(uint8_t value) {
    cpg.set_number_modules(value);
}

extern "C" void python_cpg_frequency(float value) {
    cpg.set_frequency(value);
}

extern "C" void python_cpg_direction(float value) {
    cpg.set_direction(value);
}

extern "C" void python_cpg_amplc(float value) {
    cpg.set_amplc(value);
}

extern "C" void python_cpg_amplh(float value) {
    cpg.set_amplh(value);
}

extern "C" void python_cpg_nwave(float value) {
    cpg.set_nwave(value);
}

extern "C" void python_cpg_coupling_strength(float value) {
    cpg.set_coupling_strength(value);
}

extern "C" void python_cpg_a_r(float value) {
    cpg.set_a_r(value);
}