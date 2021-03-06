import os
import rospy
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
        
    def initThread(self, com='/dev/ttyUSB-mcu', baudrate=115200):
        #baudrate = 230400
        #try:
        os.chdir("/home/robocon/RoboTop")
        os.system("./set_serial.sh")
        self.ser = serial.Serial(com, baudrate=baudrate, timeout=1)
        print "use serial port ", com
        #except:
        #    com = '/dev/ttyUSB1'
        #    self.ser = serial.Serial(com, baudrate=baudrate, timeout=1)
        #    print "use serial port ", com

    def read_debug(self, length=6):
        self.ser.flushInput()
        #msg = self.ser.read(length)
        #return msg
        data_list = []
        start = rospy.Time.now()
        delta_t = 0.0
        while(delta_t < 0.01):
            delta_t = (rospy.Time.now()-start).to_sec()
            #if len(data_list) < length:
            if self.ser.in_waiting == 0:
                continue
            #print self.ser.in_waiting
            msg = self.ser.read()
            data = struct.unpack('B', msg)[0]
            if data != 13:
                continue
            else:
                #print "*"
                data_list.append(msg)
                msg = self.ser.read()
                data = struct.unpack('B', msg)[0]
                msg = self.ser.read()
                data = struct.unpack('B', msg)[0]
                if data != 10:
                    continue
                else:
                    #print "**"
                    data_list.append(msg)
                    msg = self.ser.read(length)
                    data_list.append(msg)
                    msg = self.ser.read()
                    data = struct.unpack('B', msg)[0]
                    #print data
                    if data != 10:
                        continue
                    else:
                        #print "***"
                        data_list.append(msg)
                        msg = self.ser.read(1)
                        data = struct.unpack('B', msg)[0]
                        if data != 13:
                            continue
                        else:
                            #print "****"
                            data_list.append(msg)
                            return data_list

        return [[0], [0, 0, 0], [0]]
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
if __name__ == "__main__":
    d = Decoder(57600)
    print d.read_debug()
