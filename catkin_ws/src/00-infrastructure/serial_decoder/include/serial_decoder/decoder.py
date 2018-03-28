import serial
import struct

class Decoder():
    def __init__(self, baudrate, master=None):

        self.line = []
        #self._test()
        # 0:idle -- 1:original -- 2:binary -- 3: threshold -- 4: center -- 5:indicator -- 6:finish -- 0
        self.state = 0
        self.select = 0
        self.update_flag = 0
        self.initThread(baudrate=baudrate)
        
    def initThread(self, com='/dev/ttyUSB0', baudrate=115200):
        #baudrate = 230400
        self.ser = serial.Serial(com, baudrate=baudrate, timeout=1)

    def read_debug(self, length=6):
        data_list = []
        while(True):
            if len(data_list) < length:
                try:
                    msg = self.ser.read()
                    msg = struct.unpack('B', msg)[0]
                    data_list.append(msg)
                except:
                    fake_data = []
                    for i in range(length):
                        fake_data.append(0)
                    return fake_data
            else:
                print (data_list)
                #time.sleep(0.5)
                return data_list
