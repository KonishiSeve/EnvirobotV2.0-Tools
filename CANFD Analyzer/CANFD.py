import pyvisa as visa
import numpy as np
import matplotlib.pyplot as plt
import csv
import sys
from datetime import datetime
import struct

#Class used to retrieve the oscilloscope readings
class Analyser:
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
            self.scope.write('data:stop {}'.format(record)) # last sample
            self.scope.write('wfmpre:byt_nr 1') # 1 byte per sample
            #retrieve wave
            print(self.scope.query("horizontal:recordlength?"))
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
            self.scope.write('data:stop {}'.format(record)) # last sample
            self.scope.write('wfmpre:byt_nr 1') # 1 byte per sample
            # retrieve scaling factors
            tscale = float(self.scope.query('wfmpre:xincr?'))
            tstart = float(self.scope.query('wfmpre:xzero?'))

            total_time = tscale*record
            tstop = tstart + total_time
            return np.linspace(tstart, tstop, num=record, endpoint=False)
        except:
            r = int(self.scope.query('*esr?'))
            print('event status register: 0b{:08b}'.format(r))
            r = self.scope.query('allev?').strip()
            print('all event messages: {}'.format(r))

#Class used to retrieve CANFD packets from oscilloscope readings
class CANFD:
    def __init__(self):
        self.diff_threshold = 2
        self.std_bitrate = 1000000
        self.fd_bitrate = 4000000
        self.sample_point = 0.75
        self.dlc_fd_map = [0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64]

    #the inputs are the results of get_time and get_wave of the Analyzer class
    def parse(self, time, ch1, ch2):
        #initialize variables
        self.time = time
        self.ch1 = ch1
        self.ch2 = ch2
        self.seek_index = 0
        self.seek_time = time[0]
        self.buffer = []
        self.buffer_time = []   #stores at which timesteps the samples in the buffer where taken
        self.edges_time = []    #stores the times of edges
        packet = {}

        #sample sync bit
        self.seek_edge()
        self.seek_sample(self.sample_point/(self.std_bitrate))
        packet["sync"] = self.buffer[-1]

        #sample 11 bits identifier
        packet["identifier"] = ""
        for i in range(11):
            self.seek_sample_unstuffed(1/(self.std_bitrate))
            packet["identifier"] += str(self.buffer[-1])
        
        #sample RTR, IDE, FDF, BRS bits
        self.seek_sample_unstuffed(1/(self.std_bitrate))
        packet["RTR"] = self.buffer[-1]
        self.seek_sample_unstuffed(1/(self.std_bitrate))
        packet["IDE"] = self.buffer[-1]
        self.seek_sample_unstuffed(1/(self.std_bitrate))
        packet["FDF"] = self.buffer[-1]
        self.seek_sample_unstuffed(1/(self.std_bitrate))
        packet["res"] = self.buffer[-1]
        self.seek_sample_unstuffed(1/(self.std_bitrate))
        packet["BRS"] = self.buffer[-1]

        #look for ESI indicator
        self.seek_sample(self.sample_point/(self.fd_bitrate))
        packet["ESI"] = self.buffer[-1]

        #sample data lenght code (4 bits)
        packet["DLC"] = ""
        for i in range(4):
            self.seek_sample_unstuffed(1/(self.fd_bitrate))
            packet["DLC"] += str(self.buffer[-1])

        #sample the data frame
        packet["Data"] = ""
        for i in range(self.dlc_fd_map[int(packet["DLC"], 2)] * 8):
            self.seek_sample_unstuffed(1/(self.fd_bitrate))
            packet["Data"] += str(self.buffer[-1])

        #sample fixed stuffed bit
        self.seek_sample(1/(self.fd_bitrate))

        #sample the bit stuffing + its parity
        packet["StuffCount"] = ""
        for i in range(3):
            self.seek_sample(1/(self.fd_bitrate))
            packet["StuffCount"] += str(self.buffer[-1])
        self.seek_sample(1/(self.fd_bitrate))
        packet["StuffCountParity"] = self.buffer[-1]

        #sample the CRC
        packet["CRC"] = ""
        for i in range(26 if len(packet["Data"]) > 8*16 else 22):
            self.seek_sample(1/(self.fd_bitrate))
            if i%5 != 0:
                packet["CRC"] += str(self.buffer[-1])

        #sample the CRC delimiter
        self.seek_sample(1/(self.fd_bitrate))
        packet["CRCdelim"] = self.buffer[-1]

        self.print_packet(packet)
        print(self.buffer)
        return packet

    def print_packet(self, packet):
        for key in packet:
            if type(packet[key]) is str:
                print("{0}: {1} , {2}".format(key, hex(int(packet[key], 2)),packet[key]))
            else:
                print("{0}: {1}".format(key, packet[key]))

    def seek_edge(self, type="rising"):
        last_state = 0 if abs(self.ch1[max(0,self.seek_index-1)] - self.ch2[max(0,self.seek_index-1)]) > self.diff_threshold else 1
        for i in range(max(0,self.seek_index-1), self.time.shape[0]):
            state = 0 if abs(self.ch1[i] - self.ch2[i]) > self.diff_threshold else 1
            if type=="rising" and state==0 and state!=last_state:
                self.seek_index = i
                self.seek_time = self.time[i]
                self.edges_time.append(self.seek_time)
                print("rising edge found")
                return
            elif type=="falling" and state==1 and state!=last_state:
                self.seek_index = i
                self.seek_time = self.time[i]
                self.edges_time.append(self.seek_time)
                return
            last_state = state

    def seek_sample(self, time_delta):
        for i in range(max(0,self.seek_index-1), self.time.shape[0]):
            if self.time[i] > self.seek_time + time_delta:
                self.seek_index = i
                self.seek_time += time_delta
                self.buffer.append(0 if abs(self.ch1[i] - self.ch2[i]) > self.diff_threshold else 1)
                self.buffer_time.append(self.seek_time)
                #don't save the bit if it's a stuffed bit
                if len(self.buffer) >= 6:
                    #the last 5 bits are the same
                    if len(set(self.buffer[-6:-1])) == 1:
                        return False
                return True
    
    def seek_sample_unstuffed(self, time_delta):
        if not self.seek_sample(time_delta):
            if not self.seek_sample(time_delta):
                print(self.buffer)
                raise Exception("Bit suffing not respected")

def ENVI_protocol(packet):
    raw_bytes = [int(packet["Data"][i*8:i*8+8],2) for i in range(int(len(packet["Data"])/8))]
    print(raw_bytes)
    out = {}
    out["source"] = hex(raw_bytes[0])
    out["length"] = int(raw_bytes[1])
    out["payload"] = raw_bytes[2:2+out["length"]]

    out["ack"] = out["payload"][0] & (1 << 15)
    out["cmd"] = out["payload"][0] & (1 << 14)
    out["w/r"] = out["payload"][0] & (1 << 13)
    out["register_address"] = hex((out["payload"][0]<<8) + out["payload"][1])
    out["data0"] = struct.unpack('>f', bytes(out["payload"][2:6]))
    out["data1"] = struct.unpack('>f', bytes(out["payload"][6:10]))

    for key in out:
        print("{0}: {1}".format(key, out[key]))


ch1 = []
ch2 = []
time = []
if len(sys.argv) == 1:
    analyser = Analyser("USB::0x0699::0x0369::C040262::INSTR")
    ch1 = analyser.get_wave(1)
    ch2 = analyser.get_wave(2)
    time = analyser.get_time()
else:
    file = open(sys.argv[1], "r")
    reader = csv.reader(file)
    next(reader)
    for line in reader:
        time.append(float(line[0]))
        ch1.append(float(line[1]))
        ch2.append(float(line[2]))
    time = np.array(time)
    ch1 = np.array(ch1)
    ch2 = np.array(ch2)

canfd = CANFD()

ENVI_protocol(canfd.parse(time, ch1, ch2))

plt.plot(time, ch1)
plt.plot(time, ch2)
for i in canfd.buffer_time:
    plt.scatter(i, 2.6, color="red")
for i in canfd.edges_time:
    plt.scatter(i, 2.5, color="green")
plt.show()

if len(sys.argv) == 1:
    now = datetime.now()
    name = now.strftime("%d/%m/%Y %H:%M:%S")
    name = name.replace("/", "_")
    name = name.replace(":", "_")
    name = name.replace(" ", "_")
    file = open("logs/{0}.csv".format(name), "w", newline="")
    file_dig = open("logs/{0}_digital.csv".format(name), "w", newline="")
    writer = csv.writer(file)
    writer_dig = csv.writer(file_dig)
    #write to csv as digial
    writer.writerow(["Time[s]", "Ch1", "Ch2"])
    writer_dig.writerow(["Time[s]", "Ch1", "Ch2"])
    for i in range(len(time)):
        writer_dig.writerow([time[i], 1 if abs(ch1[i] - ch2[i]) > 2 else 0, 0 if abs(ch1[i] - ch2[i]) > 2 else 1])
        writer.writerow([time[i], ch1[i], ch2[i]])
    file.close()
    file_dig.close()