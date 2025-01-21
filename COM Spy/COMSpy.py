"""
 * COMSpy.py
 *
 *  Created on: Jan 21, 2025
 *      Author: Severin Konishi
"""
import serial
import time
baudrate = 57600

bus_one = serial.Serial('COM3', baudrate=baudrate)
bus_two = serial.Serial('COM4', baudrate=baudrate)

start = time.time()
if __name__ == "__main__":
    try:
        while(True):
            #if there is a byte in the first COM buffer, print it to CMD and send it through the second COM port
            if bus_one.in_waiting > 0:
                data = bus_one.read()
                bus_two.write(data)
                print("[{0}: {1}->{2}]: 0x{3}".format(round((time.time()-start),2), bus_one.name, bus_two.name, data.hex()))
            #if there is a byte in the second COM buffer, print it to CMD and send it through the first COM port
            if bus_two.in_waiting > 0:
                data = bus_two.read()
                bus_one.write(data)
                print("[{0}: {1}->{2}]: 0x{3}".format(round((time.time()-start),2), bus_two.name, bus_one.name, data.hex()))
    except KeyboardInterrupt:
        print("Stopped")
        bus_one.close()
        bus_two.close()