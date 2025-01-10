#Class used to retrieve CANFD packets from oscilloscope readings
class CANFDParser:
    def __init__(self):
        self.diff_threshold = 2     #differential voltage threshold
        self.std_bitrate = 1000000  #standard CAN speed
        self.fd_bitrate = 4000000   #FD CAN speed
        self.sample_point = 0.75    #when to sample in % (50% for the middle of the bit)

        #to decode FD packet length
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
            #ignore the fixed stuff bits of the CRC
            if i%5 != 0:
                packet["CRC"] += str(self.buffer[-1])

        #sample the CRC delimiter
        self.seek_sample(1/(self.fd_bitrate))
        packet["CRCdelim"] = self.buffer[-1]

        print("===== Decoded bytes =====")
        self.print_packet(packet)
        print("===== Raw bit buffer =====")
        print(self.buffer)
        return packet
    
    #print what was returned by parse()
    def print_packet(self, packet):
        for key in packet:
            if type(packet[key]) is str:
                print("{0}: {1} , {2}".format(key, hex(int(packet[key], 2)),packet[key]))
            else:
                print("{0}: {1}".format(key, packet[key]))

    #advances the seek point on the next edge
    def seek_edge(self, type="rising"):
        last_state = 0 if abs(self.ch1[max(0,self.seek_index-1)] - self.ch2[max(0,self.seek_index-1)]) > self.diff_threshold else 1
        for i in range(max(0,self.seek_index-1), self.time.shape[0]):
            state = 0 if abs(self.ch1[i] - self.ch2[i]) > self.diff_threshold else 1
            if type=="rising" and state==0 and state!=last_state:
                self.seek_index = i
                self.seek_time = self.time[i]
                self.edges_time.append(self.seek_time)
                return
            elif type=="falling" and state==1 and state!=last_state:
                self.seek_index = i
                self.seek_time = self.time[i]
                self.edges_time.append(self.seek_time)
                return
            last_state = state

    #sample the bit time_delta after the current seek point, return False if it is a stuffed bit
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
    
    #sample the bit time_delta after the current seek point, ignores stuffed bits
    def seek_sample_unstuffed(self, time_delta):
        if not self.seek_sample(time_delta):
            if not self.seek_sample(time_delta):
                print(self.buffer)
                raise Exception("Bit suffing not respected")