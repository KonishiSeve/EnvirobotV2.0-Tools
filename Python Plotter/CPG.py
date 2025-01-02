"""
 * CPG.py
 * Python implementation of the CPG script from Envirobot V1
 *
 *  Created on: Oct 29, 2024
 *      Author: Séverin Konishi
"""
import numpy as np

#Python implementation of the CPG script from Envirobot V1
class CPG():
    def __init__(self, number_modules, frequency, direction, amplc, amplh, nwave, coupling_strength, a_r):
        # == parameters == #
        self.number_modules = number_modules        #number of modules
        self.number_oscillators = 2*number_modules  #number of oscillators (2 per joint)
        self.frequency = frequency                  #oscillator frequency
        self.direction = direction                  #turning direction (positive/negative) and amplitude
        self.amplc = amplc                          #defines the amplitude of the first joint (closest to head)
        self.amplh = amplh                          #defines the amplitude of the last joint (furthest from head)
        self.nwave = nwave                          #number of waves peaks that can be seen on the robot at the same time
        self.coupling_strength = coupling_strength  #speed at which the phase difference between coupled oscillators converges
        self.a_r = a_r                              #speed at which the amplitude of the oscillators converge

        # == oscillators amplitude and derivatives == #
        self.reset()
        self.update_matrices()

        # == joint angle setpoints == #
        self.output = np.zeros(self.number_modules)

    #compute a discrete step of the CPG controller
    def step(self, delta_ms):
        #Update state of each oscillator
        for i in range(self.number_oscillators):
            coupling = 0
            # = compute dtheta (phase derivative) = #
            for j in range(self.number_oscillators):
                coupling += self.coupling_strength*self.osc_w[i][j]*self.osc_r[j]*np.sin(self.osc_theta[j]-self.osc_theta[i]-self.osc_phi[i][j])
            self.osc_dtheta[i] = (2.0*np.pi*self.frequency + coupling)

            # = compute ddr (amplitude double derivative) = #
            #First module will have an amplitude of "amplc" and last module of "amplh"
            if self.number_modules > 1:
                if i < self.number_modules:
                    ampl = self.amplh + (self.amplc-self.amplh)/(self.number_modules-1)*(self.number_modules-i-1)
                else:
                    ampl = self.amplh + (self.amplc-self.amplh)/(self.number_modules-1)*(2*self.number_modules-i-1)
            else:
                ampl=self.amplh
            #adapt amplitude depending on direction 
            if i < self.number_modules:
                ampl_r=(ampl-ampl*self.direction)/2.0
            else:
                ampl_r=(ampl+ampl*self.direction)/2.0
            self.osc_ddr[i]  = self.a_r * ( 0.25*self.a_r * (ampl_r - self.osc_r[i]) - self.osc_dr[i])

        #Discrete integration
        for i in range(self.number_oscillators):
            self.osc_theta[i] += self.osc_dtheta[i]*(delta_ms/1000.0)
            self.osc_dr[i] += self.osc_ddr[i]*(delta_ms/1000.0)
            self.osc_r[i] += self.osc_dr[i]*(delta_ms/1000.0)

        #Compute joint position
        for i in range(self.number_modules):
            self.output[i] = (self.osc_r[i+self.number_modules]*(1.0+np.cos(self.osc_theta[i+self.number_modules])) - self.osc_r[i]*(1.0+np.cos(self.osc_theta[i])))*180/np.pi
        self.output = np.clip(self.output, a_min=-60, a_max=60)   #limit angle to +- 60 degrees
        return self.output
    
    def reset(self):
        self.osc_r = np.zeros(self.number_oscillators)
        self.osc_dr = np.zeros(self.number_oscillators)
        self.osc_ddr = np.zeros(self.number_oscillators)
        self.osc_theta = np.zeros(self.number_oscillators)
        self.osc_dtheta = np.zeros(self.number_oscillators)

    def update_matrices(self):
        # == coupling weights and phase shift == #
        self.osc_w = np.zeros((self.number_oscillators,self.number_oscillators))    #coupling matrix
        self.osc_phi = np.zeros((self.number_oscillators,self.number_oscillators))  #phase shift matrix
        dphi = self.nwave*2*np.pi/(self.number_modules) #phase shift between oscillators of neighbor modules (to have the desired number of simultaneous wave "peaks" on the robot)
        for i in range(self.number_oscillators):
            for j in range(self.number_oscillators):
                #if the oscillators are from neighbor modules
                if ((j==(i+1)) and (j!=self.number_modules)):
                    self.osc_w[i][j] = 1
                    self.osc_phi[i][j] = -dphi
                #if the oscillators are from neighbor modules
                elif ((j==(i-1)) and (j!=self.number_modules)):
                    self.osc_w[i][j] = 1
                    self.osc_phi[i][j] = dphi
                #if the oscillators are on the same joint
                elif ((j==(self.number_modules+i))):
                    self.osc_w[i][j] = 1
                    self.osc_phi[i][j] = np.pi
                #if the oscillators are on the same joint
                elif ((j==(i-self.number_modules))):
                    self.osc_w[i][j] = 1
                    self.osc_phi[i][j] = np.pi

    def set_number_modules(self, value):
        self.number_modules = value
        self.number_oscillators = value*2
        self.update_matrices()
        self.reset()

    def set_frequency(self, value):
        self.frequency = value

    def set_direction(self, value):
        self.direction = value

    def set_amplc(self, value):
        self.amplc = value

    def set_amplh(self, value):
        self.amplh = value

    def set_nwave(self, value):
        self.nwave = value
        self.update_matrices()

    def set_coupling_strength(self, value):
        self.coupling_strength = value

    def set_a_r(self, value):
        self.a_r = value