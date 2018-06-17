import os
import serial
import struct
import time
import rospy

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
        
    def initThread(self, com='/dev/ttyUSB-arduino', baudrate=115200):
        #baudrate = 230400
        #try:
        os.chdir("/home/robocon/RoboTop")
        os.system("./set_serial.sh")
        self.ser = serial.Serial(com, baudrate=baudrate, timeout=1)
        #except:
        #    com = '/dev/ttyUSB2'
        #    self.ser = serial.Serial(com, baudrate=baudrate, timeout=1)

    def write(self, data):
        #try:
        self.ser.write(chr(data))
        #except:
        #    return

    def read_debug(self, length=8):
        self.ser.flushInput()
        #msg = self.ser.read(length)
        #return msg
        data_list = []
        #start = rospy.Time.now()
        delta_t = 0.0
        while(delta_t < 0.05):
            #delta_t = (rospy.Time.now()-start).to_sec()
            #if len(data_list) < length:
            buf_num = self.ser.in_waiting
            if  buf_num > 0:
                time.sleep(0.001)
            else:
                continue
            msg = self.ser.read()
            #print "***" + msg + "***"
            #print msg
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
                # first button
                msg = self.ser.read(1)
                data = struct.unpack('B', msg)[0]
                data_list.append(data)
                # second button
                msg = self.ser.read(1)
                data = struct.unpack('B', msg)[0]
                data_list.append(data)
                # tail
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

        return None
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
if __name__ == '__main__':
    decoder = ArduinoDecoder(19200)
    #print decoder.read_debug()
    count = 0
    while(1):
        print decoder.read_debug()
        count += 1
        if count == 10:
            decoder.write(76)
        if count == 20:
            decoder.write(88)
            count = 0
    while(1):
        decoder.write(76)
        time.sleep(0.01)




