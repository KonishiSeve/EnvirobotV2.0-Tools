import pyvisa as visa
import numpy as np

#Class used to retrieve the Tektronix TDS2022B oscilloscope readings
class Oscilloscope:
    def __init__(self, address):
        self.rm = visa.ResourceManager()
        self.scope = self.rm.open_resource(address)
        self.scope.timeout = 5000
        self.scope.encoding = "latin_1"
        self.scope.read_termination = "\n"
        self.scope.write_termination = None
        self.scope.write("*cls")
        print("opened: " + self.scope.query("*idn?"))

    def __del__(self):
        self.scope.close()
        self.rm.close()

    def close(self):
        self.scope.close()
        self.rm.close()

    #Returns the voltage values from the oscilloscope (as a numpy float64 array)
    def get_wave(self, channel):
        try:
            self.scope.write('data:encdg SRIBINARY')
            self.scope.write('data:source CH{0}'.format(channel)) # channel
            self.scope.write('header 0')
            self.scope.write('data:start 1')
            record = int(self.scope.query('horizontal:recordlength?'))
            self.scope.write('data:stop {0}'.format(record)) # last sample
            self.scope.write('wfmpre:byt_nr 1') # 1 byte per sample
            #retrieve wave
            bin_wave = self.scope.query_binary_values('curve?', datatype='b', container=np.array)
            # retrieve scaling factors
            vscale = float(self.scope.query('wfmpre:ymult?')) # volts / level
            voff = float(self.scope.query('wfmpre:yzero?')) # reference voltage
            vpos = float(self.scope.query('wfmpre:yoff?')) # reference position (level)
            #convert wave to voltages
            unscaled_wave = np.array(bin_wave, dtype='double') # data type conversion
            scaled_wave = (unscaled_wave - vpos) * vscale + voff
            return scaled_wave
        except:
            print(" == Oscilloscope error == ")
            r = int(self.scope.query('*esr?'))
            print('event status register: 0b{:08b}'.format(r))
            r = self.scope.query('allev?').strip()
            print('all event messages: {}'.format(r))

    #Returns the time values from the oscilloscope as a numpy array 
    #can be used with the result of get_wave for plotting
    def get_time(self):
        try:
            self.scope.write('data:encdg SRIBINARY')
            self.scope.write('header 0')
            self.scope.write('data:start 1')
            record = int(self.scope.query('horizontal:recordlength?'))
            self.scope.write('data:stop {0}'.format(record)) # last sample
            self.scope.write('wfmpre:byt_nr 1') # 1 byte per sample
            # retrieve scaling factors
            tscale = float(self.scope.query('wfmpre:xincr?'))
            tstart = float(self.scope.query('wfmpre:xzero?'))

            total_time = tscale*record
            tstop = tstart + total_time
            return np.linspace(tstart, tstop, num=record, endpoint=False)
        except:
            print(" == Oscilloscope error == ")
            r = int(self.scope.query('*esr?'))
            print('event status register: 0b{:08b}'.format(r))
            r = self.scope.query('allev?').strip()
            print('all event messages: {}'.format(r))