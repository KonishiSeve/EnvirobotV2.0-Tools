#include "CPG.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))

void CPG::init(uint8_t nb_modules,
    float frequency,
    float direction,
    float amplc,
    float amplh,
    float nwave,
    float coupling_strength,
    float a_r) {

    //store the number of modules and oscillators
    number_modules = nb_modules;
    number_oscillators = number_modules*2;

    //store the CPG parameters
    param_frequency = frequency;
    param_direction = direction;
    param_amplc = amplc;
    param_amplh = amplh;
    param_nwave = nwave;
    param_coupling_strength = coupling_strength;
    param_a_r = a_r;

    //initialize the coupling and the phase shift matrices
    update_matrices();
    reset();
}

void CPG::step(int8_t* output, float delta_ms) {
    float coupling_term;
    //Update CPG oscillators amplitude and phase
    for(uint8_t i=0;i<number_oscillators;i++) {
        // = compute dtheta = //
        coupling_term = 0;
        for(uint8_t j=0;j<number_oscillators;j++) {
            coupling_term += param_coupling_strength*osc_w[i][j]*osc_r[j]*sin(osc_theta[j]-osc_theta[i]-osc_phi[i][j]);
        }
        osc_dtheta[i] = (2*M_PI*param_frequency + coupling_term);

        // = compute ddr = //
        float ampl;
        float ampl_r;
        //make the amplitude higher for modules further from the head
        //the first module will have "amplc" amplitude and last module will have "amplh"
        if(number_modules > 1) {
            if(i < number_modules) {
                ampl = param_amplh + (param_amplc - param_amplh)/(number_modules-1)*(number_modules-i-1);
            }
            else {
                ampl = param_amplh + (param_amplc - param_amplh)/(number_modules-1)*(2*number_modules-i-1);
            }
        }
        else {
            ampl=param_amplh;
        }
        //change amplitude of left and right oscillators depending on direction
        if(i < number_modules) {
            ampl_r=(ampl-ampl*param_direction)/2.0;
        }
        else {
            ampl_r=(ampl+ampl*param_direction)/2.0;
        }
        osc_ddr[i]  = param_a_r * (0.25*param_a_r * (ampl_r - osc_r[i]) - osc_dr[i]);
    }
    for(uint8_t i=0;i<number_oscillators;i++) {
        //Euler integration
        osc_theta[i] += osc_dtheta[i]*delta_ms/1000.0;
        osc_dr[i] +=  osc_ddr[i]*delta_ms/1000.0;
        osc_r[i] +=   osc_dr[i]*delta_ms/1000.0;
    }
    //Compute joint positions by using left and right oscillators (and convert from radian to degree)
    for(uint8_t i=0;i<number_modules;i++) {
        float setpoint = (osc_r[i+number_modules]*(1.0+cos(osc_theta[i+number_modules])) - osc_r[i]*(1.0+cos(osc_theta[i])))*180/M_PI;
        //set a max angle for each joint
        setpoint = (MAX(setpoint, (-60)));
        output[i] = (int8_t)(MIN(setpoint, (60)));
    }
}

void CPG::reset(void) {
    //reset the oscillators states
    for(uint8_t i=0;i<MAX_OSCILLATORS;i++) {
        osc_r[i] = 0;
        osc_dr[i] = 0;
        osc_ddr[i] = 0;
        osc_theta[i] = 0;
        osc_dtheta[i] = 0;
    }
}

void CPG::set_number_modules(uint8_t nb_modules) {
    number_modules = nb_modules;
    number_oscillators = number_modules*2;
    update_matrices();
    reset();
}

void CPG::set_frequency(float frequency) {
    param_frequency = frequency;
}

void CPG::set_direction(float direction) {
    param_direction = direction;
}

void CPG::set_amplc(float amplc) {
    param_amplc = amplc;
}

void CPG::set_amplh(float amplh) {
    param_amplh = amplh;
}

void CPG::set_nwave(float nwave) {
    param_nwave = nwave;
    update_matrices();
}

void CPG::set_coupling_strength(float coupling_strength) {
    param_coupling_strength = coupling_strength;
}

void CPG::set_a_r(float a_r) {
    param_a_r = a_r;
}

//function to update the phi matrix called when the value of param_nwave or number_modules changes
void CPG::update_matrices(void) {
    //determines the phase shifts between modules to have the desired phase shift between head and end of tail
    float dphi = (param_nwave*2.0*M_PI)/(number_modules);
    //fill the coupling matrix and the phase shift matrix
    for(uint8_t i=0;i<number_oscillators;i++) {
        for(uint8_t j=0;j<number_oscillators;j++) {
            //if the modules of the oscillators are neighbors
            if ((j==(i+1)) && (j!=number_modules)) {
                osc_w[i][j] = 1;
                osc_phi[i][j] = -dphi;
            }
            //if the modules of the oscillators are neighbors
            else if((j==(i-1)) && (j!=number_modules)) {
                osc_w[i][j] = 1;
                osc_phi[i][j] = dphi;
            }
            //if both oscillators are on the same joint
            else if((j==(number_modules+i))) {
                osc_w[i][j] = 1;
                osc_phi[i][j] = M_PI;
            }
            //if both oscillators are on the same joint
            else if((j==(i-number_modules))) {
                osc_w[i][j] = 1;
                osc_phi[i][j] = M_PI;
            }
            else {
                osc_w[i][j] = 0;
                osc_phi[i][j] = 0;
            }
        }
    }
}