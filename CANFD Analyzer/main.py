from Oscilloscope import Oscilloscope
from CANParser import CANFDParser
import sys
import matplotlib.pyplot as plt
import csv
import numpy as np
from datetime import datetime
import struct

#save the oscilloscope readings to a csv file
save_file = False

#get the oscilloscope readings
ch1 = []
ch2 = []
time = []
#read the oscilloscope wave
if len(sys.argv) == 1:
    #opening the device, this ID can be found by right clicking the oscilloscope in Windows Device Manager --> properties --> Details --> Device instance path
    #The found path needs to be modified to the format found right below
    analyser = Oscilloscope("USB::0x0699::0x0369::C040262::INSTR")
    ch1 = analyser.get_wave(1)
    ch2 = analyser.get_wave(2)
    time = analyser.get_time()
#get the readings from a file
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

#Parse the readings
canfd = CANFDParser()
canfd.parse(time, ch1, ch2)

#Plot the oscilloscope waves with the sampling times and edges detected by the CANFD parser
plt.plot(time, ch1)
plt.plot(time, ch2)
for i in canfd.buffer_time:
    plt.scatter(i, 2.6, color="red")
for i in canfd.edges_time:
    plt.scatter(i, 2.5, color="green")
plt.show()

#Save oscilloscope data to a file if enabled
if len(sys.argv) == 1 and save_file:
    now = datetime.now()
    name = now.strftime("%d/%m/%Y %H:%M:%S")
    name = name.replace("/", "_")
    name = name.replace(":", "_")
    name = name.replace(" ", "_")
    file = open("logs/{0}.csv".format(name), "w", newline="")
    writer = csv.writer(file)
    #write to csv as digial
    writer.writerow(["Time[s]", "Ch1", "Ch2"])
    for i in range(len(time)):
        writer.writerow([time[i], ch1[i], ch2[i]])
    file.close()