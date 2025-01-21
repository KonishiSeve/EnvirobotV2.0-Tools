"""
 * RadioClient.py
 *
 *  Created on: Jan 21, 2025
 *      Author: Severin Konishi
"""

import serial
import sys
import time
import struct
# -c : channel
# -p : COM port

#Registers that are read when the "getall" command is executed
favourite_registers = [{"address":0x3E0, "size":1, "name": "version"},
          {"address":0x3E1, "size":1, "name": "channel" },
          {"address":0x3F0, "size":1, "name": "map status"},
          {"address":0x3F1, "size":1, "name": "map index"},
          {"address":0x3F2, "size":2, "name": "map radio address"},
          {"address":0x3F3, "size":2, "name": "map framework address"},
          {"address":0x3F4, "size":2, "name": "map length"}
          ]

#Informations to print for the "help" command
help_print = [{"command": "getb ADDRESS", "description": "Reads an 8 bits register"},
              {"command": "getw ADDRESS", "description": "Reads a 16 bits register"},
              {"command": "getdw ADDRESS", "description": "Reads a 32 bits register"},
              {"command": "getall", "description": "Reads all registers of the favourite list (set in the python file)"},
              {"command": "setb ADDRESS VALUE", "description": "Write an 8 bits register"},
              {"command": "setw ADDRESS VALUE", "description": "Write a 16 bits register"},
              {"command": "setdw ADDRESS VALUE", "description": "Write a 32 bits register"},
              {"command": "mapr", "description": "Read all existing mappings"},
              {"command": "mapnew RADIO_ADDRESS FRAMEWORK_ADDRESS LENGTH", "description": "Create a new mapping"},
              {"command": "mapmod MAPPING_INDEX RADIO_ADDRESS FRAMEWORK_ADDRESS LENGTH", "description": "Modifies an existing mapping"},
              {"command": "mapreset", "description": "Deletes all existing mappings"},
              {"command": "monitorb ADDRESS FREQUENCY", "description": "Reads the 8 bits register at the given frequency"},
              {"command": "monitorw ADDRESS FREQUENCY", "description": "Reads the 16 bits register at the given frequency"},
              {"command": "monitordw ADDRESS FREQUENCY", "description": "Reads the 32 bits register at the given frequency"},
              {"command": "robot start", "description": "Enable the motors"},
              {"command": "robot stop", "description": "Disable the motors"},
              {"command": "cpg start", "description": "Start the CPG controller"},
              {"command": "cpg stop", "description": "Stop the CPG controller"},
              {"command": "cpg freq VALUE", "description": "Set the CPG frequency"},
              {"command": "cpg dir VALUE", "description": "Set the CPG direction"},
              {"command": "cpg amplc VALUE", "description": "Set the CPG amplc"},
              {"command": "cpg amplh VALUE", "description": "Set the CPG amplh"},
              {"command": "cpg nwave VALUE", "description": "Set the CPG nwave"},
              {"command": "cpg coupling VALUE", "description": "Set the CPG coupling strength"},
              {"command": "cpg ar VALUE", "description": "Set the CPG ar"},
              {"command": "cpg dirmax VALUE", "description": "Set the CPG dirmax"},
              {"command": "cpg amplcmax VALUE", "description": "Set the CPG amplcmax"},
              {"command": "cpg amplhmaxVALUE", "description": "Set the CPG amplhmax"},
              {"command": "stmreset", "description": "Resets the STM32"},
              {"command": "watercheck FREQUENCY", "description": "monitors the water alert register and tell the user if a leak is detected"},
              {"command": "exit", "description": "Close the shell"},
              ]

#Class interfacing with the USB radio dongle
class PCRadio:
    def __init__(self, port):
        self.serial = serial.Serial(port, 56800, timeout=1.0)
        #synchronize with USB radio dongle
        while(1):
            for i in range(16):
                self.serial.write(0xFF.to_bytes(1))
            self.serial.flush()
            self.serial.write(0xAA.to_bytes(1))
            response = self.serial.read(1)
            if response[0] == 0xAA:
                print("synchronized")
                return
            print("sync failed {0}".format(response[0]))
            time.sleep(5)
            
    def __del__(self):
        self.serial.close()
    
    #change the USB dongle radio channel
    def set_channel(self, channel):
        #write the channel
        self.serial.write(0x13.to_bytes(1))
        self.serial.write(0xc1.to_bytes(1))
        self.serial.write(channel.to_bytes(1))
        if self.serial.read(1)[0] != 0x06:
            return False
        #read the channel back
        self.serial.write(0x03.to_bytes(1))
        self.serial.write(0xc1.to_bytes(1))
        response = self.serial.read(2)
        if(response[0] == 0x06 and response[1] == channel):
            return True
        return False
    
    #read an 8 bits register
    def reg_read_8(self, address):
        op = 0x00
        self.serial.write((op<<2 | address>>8).to_bytes(1))
        self.serial.write((address & 0xFF).to_bytes(1))
        if self.serial.read(1)[0] != 0x06:
            return None
        else:
            return self.serial.read(1)[0]
    
    #read a 16 bits register as signed int
    def reg_read_16(self, address):
        op = 0x01
        self.serial.write((op<<2 | address>>8).to_bytes(1))
        self.serial.write((address & 0xFF).to_bytes(1))
        if self.serial.read(1)[0] != 0x06:
            return None
        else:
            return int.from_bytes(self.serial.read(2), byteorder='little', signed=True)
    
    #read a 32 bits register as signed int
    def reg_read_32(self, address):
        op = 0x02
        self.serial.write((op<<2 | address>>8).to_bytes(1))
        self.serial.write((address & 0xFF).to_bytes(1))
        if self.serial.read(1)[0] != 0x06:
            return None
        else:
            return int.from_bytes(self.serial.read(4), byteorder='little', signed=True)
    
    #read a 32 bits register as float
    def reg_read_float(self, address):
        op = 0x02
        self.serial.write((op<<2 | address>>8).to_bytes(1))
        self.serial.write((address & 0xFF).to_bytes(1))
        if self.serial.read(1)[0] != 0x06:
            return None
        else:
            return struct.unpack('f', self.serial.read(4))[0]
    
    #write an 8 bits register
    def reg_write_8(self, address, value):
        op = 0x04
        self.serial.write((op<<2 | address>>8).to_bytes(1))
        self.serial.write((address & 0xFF).to_bytes(1))
        self.serial.write(value.to_bytes(1))
        if self.serial.read(1)[0] != 0x06:
            return None
        else:
            return True
    
    #write a 16 bits register
    def reg_write_16(self, address, value):
        op = 0x05
        self.serial.write((op<<2 | address>>8).to_bytes(1))
        self.serial.write((address & 0xFF).to_bytes(1))
        self.serial.write(value.to_bytes(2, "little"))
        if self.serial.read(1)[0] != 0x06:
            return None
        else:
            return True
    
    #write a 32 bits register
    def reg_write_32(self, address, value):
        op = 0x06
        self.serial.write((op<<2 | address>>8).to_bytes(1))
        self.serial.write((address & 0xFF).to_bytes(1))
        self.serial.write(value.to_bytes(4, "little"))
        if self.serial.read(1)[0] != 0x06:
            return None
        else:
            return True
    
    #write a float to a 32 bits register
    def reg_write_float(self, address, value):
        op = 0x06
        self.serial.write((op<<2 | address>>8).to_bytes(1))
        self.serial.write((address & 0xFF).to_bytes(1))
        self.serial.write(struct.pack("f", value))
        if self.serial.read(1)[0] != 0x06:
            return None
        else:
            return True

if __name__ == "__main__":
    #parse the startup command
    channel = None
    port = None
    segments = " ".join(sys.argv[1:])
    segments = segments.split("-")
    parameters = []
    for segment in segments[1:]:
        instruction = {"command":segment.split(" ")[0], "value":segment.split(" ")[1]}
        if instruction["command"] == "c":
            channel = int(instruction["value"])
            print("channel: {0}".format(channel))
        elif instruction["command"] == "p":
            port = instruction["value"]
            print("port: {0}".format(port))

    if (port is None) or (channel is None):
        print("[ERROR] Set the channel number and port with -c CHANNEL and -p COMPORT")
        exit()

    #Setting up the radio dongle
    radio = PCRadio(port)
    radio.set_channel(channel)

    #Start the shell
    stop_shell = False
    try:
        print('Shell started, type "help" to get a list of supported commands')
        while(not stop_shell):
            command = input("> ")
            command = command.split(" ")
            if(command[0] == "exit"):
                stop_shell = True
                break
            
            #check if the UART buffer is still synchronized (for debug)
            elif(command[0] == "sync"):
                value = radio.serial.in_waiting
                if(value > 0):
                    print("bad sync: {0} bytes behind".format(value))
                    print(", ".join([hex(i) for i in radio.serial.read(value)]))
                    continue
                else:
                    print("good sync")
                    continue

            #Read all registers of the "favourite" list
            elif(command[0] == "getall"):
                for i in favourite_registers:
                    if(i["size"] == 1):
                        value = radio.reg_read_8(int(i["address"]))                    
                    elif(i["size"] == 2):
                        value = radio.reg_read_16(int(i["address"]))
                    else:
                        print("{0}({1}): unsupported size".format(i["name"], hex(i["address"])))
                        continue
                    print("{0}({1}): {2}".format(i["name"], hex(i["address"]), "Error" if value is None else hex(value)))

            #Read a register
            elif(command[0][:3] == "get"):
                if(len(command) != 2) or (len(command[0]) <= 3):
                    print('usage (type "help" for more details): getb ADDRESS | getw ADDRESS | getdw ADDRESS | getf ADDRESS')
                    continue
                elif(command[0][3] == "b"): #read 1 byte register
                    value = radio.reg_read_8(int(command[1], 0))
                elif(command[0][3] == "w"): #read 2 bytes register
                    value = radio.reg_read_16(int(command[1], 0))
                elif(command[0][3:5] == "dw"):
                    value = radio.reg_read_32(int(command[1], 0))
                elif(command[0][3] == "f"):
                    value = radio.reg_read_float(int(command[1], 0))
                    print(value)
                    #print("{0} ({1})".format("Error" if value is None else value, "Error" if value is None else hex(value)))
                    continue
                else:
                    print('usage (type "help" for more details): getb ADDRESS | getw ADDRESS | getdw ADDRESS | getf ADDRESS')
                    continue
                print("{0} ({1})".format("Error" if value is None else value, "Error" if value is None else hex(value)))

            #Write a register
            elif(command[0][:3] == "set"):
                if(len(command) != 3):
                    print('usage (type "help" for more details): setb ADDRESS VALUE | setw ADDRESS VALUE | setdw ADDRESS VALUE')
                    continue
                elif(command[0][3] == "b"):
                    radio.reg_write_8(int(command[1], 0), int(command[2], 0))
                elif(command[0][3] == "w"):
                    radio.reg_write_16(int(command[1], 0), int(command[2], 0))
                elif(command[0][3:5] == "dw"):
                    radio.reg_write_32(int(command[1], 0), int(command[2], 0))
                elif(command[0][3] == "f"):
                    radio.reg_write_float(int(command[1], 0), float(command[2]))
                else:
                    print('usage (type "help" for more details): setb ADDRESS VALUE | setw ADDRESS VALUE | setdw ADDRESS VALUE')
                    continue
            
            #Read all mappings
            elif(command[0] == "mapr"): #read all the mappings
                map_count = radio.reg_read_8(0x3F0)
                if(map_count>=0xe0):
                    print("[Error {0}] Mapping access error, please retry".format(hex(map_count)))
                    continue
                print("mappings: {0}".format(map_count))
                for i in range(map_count):
                    radio.reg_write_8(0x3F0, i)
                    radio_addr = radio.reg_read_16(0x3F2)
                    frame_addr = radio.reg_read_16(0x3F3)
                    length = radio.reg_read_16(0x3F4)
                    index = radio.reg_read_8(0x3F1)
                    print("{4}: (radio) {0}-{1} <--> {2}-{3} (framework)".format(hex(radio_addr), hex(radio_addr+length-1), hex(frame_addr), hex(frame_addr+length-1), hex(index)))
                continue
            
            #Create a new mapping
            elif(command[0] == "mapnew"):
                if(len(command) != 4):
                    print('usage (type "help" for more details): mapnew RADIO_ADDRESS FRAMEWORK_ADDRESS LENGTH')
                    continue
                map_count = radio.reg_read_8(0x3F0)
                radio.reg_write_8(0x3F1, 0xFF) #index (new mapping)
                radio.reg_write_16(0x3F2, int(command[1], 0)) #radio addresss
                radio.reg_write_16(0x3F3, int(command[2], 0)) #framework address
                radio.reg_write_16(0x3F4, int(command[3], 0)) #length
                radio.reg_write_8(0x3F0, 0xAA) #confirm new mapping
                new_map_count = radio.reg_read_8(0x3F0)
                if(map_count+1 != new_map_count):
                    print("[Error {0}]: Could not confirm that the mapping was set".format(hex(new_map_count)))
                else:
                    print("Mapping {0} registered".format(hex(map_count)))

            #Modify an existing mapping
            elif(command[0] == "mapmod"):
                if(len(command) != 5):
                    print('usage (type "help" for more details): mapmod MAPPING_INDEX RADIO_ADDRESS FRAMEWORK_ADDRESS LENGTH'.format(command[0]))
                    continue
                map_count = radio.reg_read_8(0x3F0)
                if(int(command[1], 0) >= map_count):
                    print("[Error] mapping with index {0} does not exist".format(int(command[1], 0)))
                    continue
                radio.reg_write_8(0x3F1, int(command[1], 0)) #index (new mapping)
                radio.reg_write_16(0x3F2, int(command[2], 0)) #radio addresss
                radio.reg_write_16(0x3F3, int(command[3], 0)) #framework address
                radio.reg_write_16(0x3F4, int(command[4], 0)) #length
                radio.reg_write_8(0x3F0, 0xAA) #confirm new mapping

            #Delete all the mappings
            elif(command[0] == "mapreset"):
                radio.reg_write_8(0x3F0, 0xAB)
                radio.reg_write_8(0x3F0, 0xAC)

            #Read a register at a fixed frequency
            elif(command[0][:7] == "monitor"):
                if(len(command) != 3):
                    print('usage (type "help" for more details): monitorb ADDRESS FREQUENCY | monitorw ADDRESS FREQUENCY | monitordw ADDRESS FREQUENCY'.format(command[0]))
                    continue
                try:
                    print("Monitoring of {0} started at {1}Hz (Ctrl+c to stop)".format(command[1], command[2]))
                    while(True):
                        start_time = time.time()
                        if(command[0][7] == "b"): #read 1 byte register
                            value = radio.reg_read_8(int(command[1], 0))
                        elif(command[0][7] == "w"):
                            value = radio.reg_read_16(int(command[1], 0))
                        elif(command[0][7:9] == "dw"):
                            value = radio.reg_read_32(int(command[1], 0))
                        else:
                            print('usage (type "help" for more details): monitorb ADDRESS FREQUENCY | monitorw ADDRESS FREQUENCY | monitordw ADDRESS FREQUENCY')
                            break
                        print("{0} ({1})".format(value, hex(value)))
                        time.sleep((1/float(command[2])) - (time.time()-start_time))
                except KeyboardInterrupt:
                    pass
                print("Monitoring stopped")

            #Starting and stopping the robot
            elif(command[0] == "robot"):
                if(command[1] == "start"):
                    radio.reg_write_8(0x00, 1)
                elif(command[1] == "stop"):
                    radio.reg_write_8(0x00, 0)
            
            #Change the CPG parameters
            elif(command[0] == "cpg"):
                if(command[1] == "start"):
                    radio.reg_write_8(0x101, 2)
                elif(command[1] == "stop"):
                    radio.reg_write_8(0x101, 0)
                elif(command[1] == "freq"):
                    radio.reg_write_float(0x110,float(command[2]))
                elif(command[1] == "dir"):
                    radio.reg_write_float(0x111,float(command[2]))
                elif(command[1] == "amplc"):
                    radio.reg_write_float(0x112,float(command[2]))
                elif(command[1] == "amplh"):
                    radio.reg_write_float(0x113,float(command[2]))
                elif(command[1] == "nwave"):
                    radio.reg_write_float(0x114,float(command[2]))
                elif(command[1] == "coupling"):
                    radio.reg_write_float(0x115,float(command[2]))
                elif(command[1] == "ar"):
                    radio.reg_write_float(0x116,float(command[2]))
                elif(command[1] == "dirmax"):
                    radio.reg_write_float(0x120,float(command[2]))
                elif(command[1] == "amplcmax"):
                    radio.reg_write_float(0x121,float(command[2]))
                elif(command[1] == "amplhmax"):
                    radio.reg_write_float(0x122,float(command[2]))

            #Trigger the reset pin of the STM32
            elif(command[0] == "stmreset"):
                radio.reg_write_8(0x3E2, 0xAA)
                continue
            
            #Read the water leak register and checks that the response is correct
            elif(command[0] == "watercheck"):
                if(len(command) != 2):
                    print('usage (type "help" for more details): watercheck FREQUENCY')
                    continue
                try:
                    print("Monitoring started at {0}Hz (Ctrl + c to stop)".format(command[1]))
                    while(True):
                        value = radio.reg_read_32(0x201)
                        value = 0 if value is None else value
                        packet_lost = 0
                        start_time = time.time()
                        while((value&0xFFFFFF00) != 0x65432100):    #retry until a valid packet is received
                            packet_lost += 1
                            value = radio.reg_read_32(0x201)
                            value = 0 if value is None else value
                            time.sleep(0.1)
                        if(value&0xFF == 0):
                            print("No leak detected ({0}) [packets lost: {1}]".format(hex(value), packet_lost))
                        else:
                            print("[ALERT] LEAK IN MODULE {0} ({1}) [packets lost: {2}]".format(value&0xFF, hex(value), packet_lost))
                        time.sleep(max((1/float(command[1])) - (time.time()-start_time), 0))
                except KeyboardInterrupt:
                    pass
                print("Monitoring stopped")
                continue
                
            elif(command[0] == "help"):
                #get the longest command length to pad the others
                max_len = 0
                for i in help_print:
                    max_len = max(max_len, len(i["command"]))
                for i in help_print:
                    print("{0} : {1}".format(i["command"].ljust(max_len, " "), i["description"]))
            else:
                print("Invalid command")
    except KeyboardInterrupt:
        print("Stopped")
