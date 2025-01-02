"""
 * CPP_CPG.py
 * Python wrapper for the C++ implementation of the CPG script from Envirobot V1, for compatilibily with the plotter
 *
 *  Created on: Oct 29, 2024
 *      Author: Séverin Konishi
"""

import ctypes
import numpy as np
from os.path import abspath
import os

class CPP_CPG():
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
        self.osc_r = np.zeros(self.number_oscillators)
        self.osc_dr = np.zeros(self.number_oscillators)
        self.osc_ddr = np.zeros(self.number_oscillators)

        # == oscillators phase and derivative == #
        self.osc_theta = np.zeros(self.number_oscillators)
        self.osc_dtheta = np.zeros(self.number_oscillators)

        # == joint setpoints == #
        self.output = []

        #Auto-compile the C++ part into a dll file (if it doesn't work, remove the line and compile manually)
        os.system("g++ -shared -o CPG.dll CPG.cpp python_link.cpp")

        #Import the dll file
        self.dll = ctypes.CDLL(abspath('CPG.dll'))
        #define the argument types of the C++ functions
        self.dll.python_cpg_init.argtypes = (ctypes.c_uint8,
                                         ctypes.c_float,
                                         ctypes.c_float,
                                         ctypes.c_float,
                                         ctypes.c_float,
                                         ctypes.c_float,
                                         ctypes.c_float,
                                         ctypes.c_float)
        self.dll.python_cpg_step.argtypes = (ctypes.POINTER(ctypes.c_int8),
                                         ctypes.c_float)
        
        self.dll.python_cpg_states.argtypes = (ctypes.POINTER(ctypes.c_float),
                                           ctypes.POINTER(ctypes.c_float),
                                           ctypes.POINTER(ctypes.c_float),
                                           ctypes.POINTER(ctypes.c_float),
                                           ctypes.POINTER(ctypes.c_float))
        
        self.dll.python_cpg_number_modules.argtypes = (ctypes.c_uint8,)
        self.dll.python_cpg_frequency.argtypes = (ctypes.c_float,)
        self.dll.python_cpg_direction.argtypes = (ctypes.c_float,)
        self.dll.python_cpg_amplc.argtypes = (ctypes.c_float,)
        self.dll.python_cpg_amplh.argtypes = (ctypes.c_float,)
        self.dll.python_cpg_nwave.argtypes = (ctypes.c_float,)
        self.dll.python_cpg_coupling_strength.argtypes = (ctypes.c_float,)
        self.dll.python_cpg_a_r.argtypes = (ctypes.c_float,)

        #initialize the CPG controller on the C++ side
        self.dll.python_cpg_init(self.number_modules,
                             self.frequency, self.direction, self.amplc,
                             self.amplh, self.nwave, self.coupling_strength,
                             self.a_r)

    def step(self, delta_ms):
        #compute a step of the CPG controller by calling the C++ function
        array_type = ctypes.c_int8 * self.number_modules
        array_out = []
        output = array_type(*array_out)
        self.dll.python_cpg_step(output, delta_ms)
        self.output = np.array(output[:])

        #update the oscillator states
        array_type = ctypes.c_float * (self.number_modules*2)
        array_r = []
        array_dr = []
        array_ddr = []
        array_theta = []
        array_dtheta = []
        self.dll.python_cpg_states(array_type(*array_r), array_type(*array_dr), array_type(*array_ddr), array_type(*array_theta), array_type(*array_dtheta))
        self.osc_r = np.array(array_r)
        self.osc_dr = np.array(array_dr)
        self.osc_ddr = np.array(array_ddr)
        self.osc_theta = np.array(array_theta)
        self.osc_dtheta = np.array(array_dtheta)
        return self.output
    
    def reset(self):
        self.osc_r = np.zeros(self.number_oscillators)
        self.osc_dr = np.zeros(self.number_oscillators)
        self.osc_ddr = np.zeros(self.number_oscillators)
        self.osc_theta = np.zeros(self.number_oscillators)
        self.osc_dtheta = np.zeros(self.number_oscillators)
        self.dll.python_cpg_reset()

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
        self.dll.python_cpg_number_modules(value)
        self.update_matrices()
        self.reset()

    def set_frequency(self, value):
        self.frequency = value
        self.dll.python_cpg_frequency(value)

    def set_direction(self, value):
        self.direction = value
        self.dll.python_cpg_direction(value)

    def set_amplc(self, value):
        self.amplc = value
        self.dll.python_cpg_amplc(value)

    def set_amplh(self, value):
        self.amplh = value
        self.dll.python_cpg_amplh(value)

    def set_nwave(self, value):
        self.nwave = value
        self.update_matrices()
        self.dll.python_cpg_nwave(value)

    def set_coupling_strength(self, value):
        self.coupling_strength = value
        self.dll.python_cpg_coupling_strength(value)

    def set_a_r(self, value):
        self.a_r = value
        self.dll.python_cpg_a_r(value)