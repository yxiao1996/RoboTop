import serial

class SerialEncoder():
    def __init__(self, baudrate):
        self.initThread(baudrate=baudrate)

    def initThread(self, com='/dev/ttyUSB0', baudrate=115200):
        #baudrate = 230400
        try:
            self.ser = serial.Serial(com, baudrate=baudrate, timeout=1)
        except:
            self.ser = serial.Serial('/dev/ttyUSB1', baudrate=baudrate, timeout=1)
        # Debug
        print "Serial Port Configuration"
        print "bytesize: ", self.ser.bytesize
        print "baudrate: ", self.ser.baudrate
        print "port: ", self.ser.port
        print "parity: ", self.ser.parity
        print "stopbits: ",  self.ser.stopbits

    def write(self, data):
        try:
            self.ser.write(chr(data))
        except:
            return

if __name__ == '__main__':
    e = SerialEncoder()
    cmd_list = [chr(128), chr(127), chr(126), chr(125), chr(124), chr(123)]
    for cmd in cmd_list:
        print "writing: ", cmd
        e.write(cmd)
