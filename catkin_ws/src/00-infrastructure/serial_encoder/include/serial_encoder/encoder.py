import serial

class SerialEncoder():
    def __init__(self):
        self.initThread()

    def initThread(self, com='/dev/ttyUSB0', baudrate=115200):
        self.ser = serial.Serial(com, baudrate=baudrate, timeout=1)
        
        # Debug
        print "Serial Port Configuration"
        print "bytesize: ", self.ser.bytesize
        print "baudrate: ", self.ser.baudrate
        print "port: ", self.ser.port
        print "parity: ", self.ser.parity
        print "stopbits: ",  self.ser.stopbits

    def write(self, data):
        self.ser.write(chr(data))

if __name__ == '__main__':
    e = SerialEncoder()
    cmd_list = [chr(128), chr(127), chr(126), chr(125), chr(124), chr(123)]
    for cmd in cmd_list:
        print "writing: ", cmd
        e.write(cmd)