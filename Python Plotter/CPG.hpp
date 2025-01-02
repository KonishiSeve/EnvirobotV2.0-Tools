#pragma once
#include <stdint.h>
//defines the max number of modules
#define MAX_MODULES     20
#define MAX_OSCILLATORS (MAX_MODULES*2)

class CPG {
    public:
        void init(uint8_t nb_modules,
            float frequency,
            float direction,
            float amplc,
            float amplh,
            float nwave,
            float coupling_strength,
            float a_r
        );
        void step(int8_t* output, float delta_ms);

        void reset();   //Resets all the oscillators states

        //update CPG parameters
        void set_number_modules(uint8_t nb_modules);
        void set_frequency(float frequency);
        void set_direction(float direction);
        void set_amplc(float amplc);
        void set_amplh(float amplh);
        void set_nwave(float nwave);
        void set_coupling_strength(float coupling_strength);
        void set_a_r(float a_r);

        //Oscillator variables, put as public to allow logging
        //osc_xxx[i] and osc_xxx[i + number_modules] contain the state of oscillators controlling the same joint
        float osc_r[MAX_OSCILLATORS] = {0};   //amplitude of the oscillators
        float osc_dr[MAX_OSCILLATORS] = {0};  //amplitude derivative
        float osc_ddr[MAX_OSCILLATORS] = {0}; //amplitude float derivative

        float osc_theta[MAX_OSCILLATORS] = {0};   //phase of the oscillators
        float osc_dtheta[MAX_OSCILLATORS] = {0};  //phase derivative

        uint8_t osc_w[MAX_OSCILLATORS][MAX_OSCILLATORS] = {{0}};   //binary coupling matrix (the coupling strength is taken into account in the step function)
        float osc_phi[MAX_OSCILLATORS][MAX_OSCILLATORS] = {{0}};   //phase shift between oscillators

    private:
        void update_matrices(void);

        //Radio parameters
        float param_frequency;
        float param_direction;
        float param_amplc;
        float param_amplh;
        float param_nwave;                //how many wave peaks are visible on the robot at the same time
        float param_coupling_strength;    //speed at which the phase difference between oscillators converge
        float param_a_r;                  //speed at which the amplitude of the oscillators converge

        //Variables for computation
        uint8_t number_modules;
        uint8_t number_oscillators;
};