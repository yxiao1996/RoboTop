import serial
import struct
import time

class ArduinoDecoder():
    def __init__(self, baudrate, master=None):

        self.line = []
        #self._test()
        # 0:idle -- 1:original -- 2:binary -- 3: threshold -- 4: center -- 5:indicator -- 6:finish -- 0
        self.state = 0
        self.select = 0
        self.update_flag = 0
        self.initThread(baudrate=baudrate)
        #time.sleep(1)
        
    def initThread(self, com='/dev/ttyUSB1', baudrate=115200):
        #baudrate = 230400
        #try:
        self.ser = serial.Serial(com, baudrate=baudrate, timeout=1)
        #except:
        #    com = '/dev/ttyUSB2'
        #    self.ser = serial.Serial(com, baudrate=baudrate, timeout=1)

    def read_debug(self, length=8):
        self.ser.flushInput()
        #msg = self.ser.read(length)
        #return msg
        data_list = []
        while(True):
            #if len(data_list) < length:
            msg = self.ser.read()
            print "***" + msg + "***"
            print msg
            data = struct.unpack('B', msg)[0]
            #if data != 5:
            #    continue
            #else:
                #print "*"
                #data_list.append(msg)
            #    msg = self.ser.read()
            #    data = struct.unpack('B', msg)[0]
            #    msg = self.ser.read()
            #    data = struct.unpack('B', msg)[0]
            if data != 165:
                continue
            else:
                #print "**"
                #data_list.append(msg)
                # first sensor
                msg = self.ser.read(1)
                high = struct.unpack('B', msg)[0]
                msg = self.ser.read(1)
                low = struct.unpack('B', msg)[0]
                data = high * 256 + low
                data_list.append(data)
                # second sensor
                msg = self.ser.read(1)
                high = struct.unpack('B', msg)[0]
                msg = self.ser.read(1)
                low = struct.unpack('B', msg)[0]
                data = high * 256 + low
                data_list.append(data)
                # third sensor
                msg = self.ser.read(1)
                high = struct.unpack('B', msg)[0]
                msg = self.ser.read(1)
                low = struct.unpack('B', msg)[0]
                data = high * 256 + low
                data_list.append(data)
                # fourth sensor
                msg = self.ser.read(1)
                high = struct.unpack('B', msg)[0]
                msg = self.ser.read(1)
                low = struct.unpack('B', msg)[0]
                data = high * 256 + low
                data_list.append(data)
                msg = self.ser.read()
                data = struct.unpack('B', msg)[0]
                #print data
                if data != 90:
                    continue
                else:
                    #print "***"
                    #data_list.append(msg)
                #    msg = self.ser.read(1)
                #    data = struct.unpack('B', msg)[0]
                #if data != 5:
                #    continue
                #else:
                    #print "****"
                    #data_list.append(msg)
                    return data_list


        msg = self.ser.read(length)
        return msg
        while(True):
            if len(data_list) < length:
                try:
                    msg = self.ser.read()
                    #msg = struct.unpack('B', msg)[0]
                    data_list.append(msg)
                except:
                    fake_data = []
                    for i in range(length/4):
                        fake_data.append('A')
                        fake_data.append('D')
                        fake_data.append('D')
                        fake_data.append('A')
                    return fake_data
            else:
                #print (data_list)
                #time.sleep(0.5)
                return data_list

#decoder = ArduinoDecoder(115200)
#print decoder.read_debug()