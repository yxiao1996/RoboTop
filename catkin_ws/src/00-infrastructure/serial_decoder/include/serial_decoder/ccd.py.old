
from __future__ import print_function
import numpy as np
from Tkinter import *
from PIL import Image, ImageTk
import ttk
import serial
import time
import threading
import struct

class CCD(Frame):
    def __init__(self, master=None):
        Frame.__init__(self, master, width=720, height=500)
        self.pack()
        self.initFig()
        self.line = []
        self.threshold = 130
        self._test()
        self.state = 0
        self.select = 0
        self.update_flag = 0
        self.initThread()
        
    def initThread(self, com='/dev/ttyUSB0', baudrate=115200):
        self.ser = serial.Serial(com, baudrate=baudrate, timeout=10)
        #self.worker = threading.Thread(target=self.read)
        #self.worker.setDaemon(True)
        #self.worker.start()

    def on_update_com(self):
        self.state = 1
        if self.baudrate.get() != '':
            baudrate = eval(self.baudrate.get())
        else:
            baudrate = 115200
        self.ser = serial.Serial(baudrate=baudrate, timeout=1)

    def on_stop_com(self):
        self.state = 0

    def initFig(self):
        self.com = LabelFrame(self, text='Com Setting')
        self.com.grid(row=0, column=0)
        self.baudrateLabel = Label(self.com, text='Baudrate')
        self.baudrateLabel.pack(side=LEFT)
        self.baudrate = StringVar()
        self.baud = Entry(self.com, textvariable=self.baudrate)
        self.baud.pack(side=LEFT)
        self.updateCom = Button(self.com, text='Start Com', command=self.on_update_com)
        self.updateCom.pack(side=LEFT)
        self.stopCom = Button(self.com, text='Stop Com', command=self.on_stop_com)
        self.stopCom.pack(side=LEFT)
        #self.com.pack(fill=BOTH)

        self.canvas_1 = LabelFrame(self, text='Channel 1')
        self.num1 = Label(self.canvas_1, text='0                              25                               50                              75                              100                             125   ')
        self.num1.pack(side=TOP)
        self.canvas_1_0 = Label(self.canvas_1)
        self.canvas_1_1 = Label(self.canvas_1)
        self.canvas_1_0.pack(side=TOP)
        self.canvas_1_1.pack(side=TOP)
        self.canvas_1.grid(row=2, column=0)
        #self.canvas_1.pack(fill=BOTH)
        self.canvas_2 = LabelFrame(self, text='Channel 2')
        self.num2 = Label(self.canvas_2, text='0                              25                               50                              75                              100                             125   ')
        self.num2.pack(side=TOP)
        self.canvas_2_0 = Label(self.canvas_2)
        self.canvas_2_1 = Label(self.canvas_2)
        self.canvas_2_0.pack(side=TOP)
        self.canvas_2_1.pack(side=TOP)
        self.canvas_2.grid(row=3, column=0)
        #self.canvas_2.pack(fill=BOTH)
        self.canvas_3 = LabelFrame(self, text='Channel 3')
        self.num3 = Label(self.canvas_3, text='0                              25                               50                              75                              100                             125   ')
        self.num3.pack(side=TOP)
        self.canvas_3_0 = Label(self.canvas_3)
        self.canvas_3_1 = Label(self.canvas_3)
        self.canvas_3_0.pack(side=TOP)
        self.canvas_3_1.pack(side=TOP)
        self.canvas_3.grid(row=4, column=0)
        #self.canvas_3.pack(fill=BOTH)
        self.canvas_4 = LabelFrame(self, text='Channel 4')
        self.num3 = Label(self.canvas_4, text='0                              25                               50                              75                              100                             125   ')
        self.num3.pack(side=TOP)
        self.canvas_4_0 = Label(self.canvas_4)
        self.canvas_4_1 = Label(self.canvas_4)
        self.canvas_4_0.pack(side=TOP)
        self.canvas_4_1.pack(side=TOP)
        self.canvas_4.grid(row=5, column=0)
        #self.canvas_4.pack(fill=BOTH)

    def read(self):
        #while(True):
            # 1 byte a time
            msg = self.ser.read()
            print (msg)
            msg = struct.unpack('B', msg)[0]
            #print (hex(msg))
            # S: 250 128      E: 251
            # time.sleep(0.001)
            if self.state:
                self.line.append(msg)
                print("Current length: " + str(len(self.line))+" : "+str(msg))
                # return msg
                if len(self.line) == 128:
                    temp = self.ser.read()
                    temp = struct.unpack('B', temp)[0]
                    if temp == 251:
                        print("1 Update.")
                        #self.plot(np.uint8(np.array([self.line])), self.select)
                    if temp == 253:
                        print("2 Update.")
                        #self.plot(np.uint8(np.array([self.line])), self.select)
                    if temp == 255:
                        print("3 Update.")
                        #self.plot(np.uint8(np.array([self.line])), self.select)
                    self.line = []
                    self.state = 0
                        
            else:
                # time.sleep(0.1)
                if msg == 250:
                    self.select = 1
                    self.state = 1
                    print("Start.")
                if msg == 252:
                    self.select = 2
                    self.state = 1
                    print("Start.")
                if msg == 254:
                    self.select = 3
                    self.state = 1
                    print("Start.")
            return self.line

    def plot(self, array, select=0):
        img = Image.fromarray(array).convert('L')
        img = img.resize((720, 50))
        img = ImageTk.PhotoImage(image=img)
        for i in range(128):
            if array[0][i] > self.threshold:
                array[0][i] = 255
            else:
                array[0][i] = 0
        img_1 = Image.fromarray(array)
        img_1 = img_1.resize((720, 50))
        img_1 = ImageTk.PhotoImage(image=img_1)
        if select == 1:
            self.canvas_1_0.configure(image=img)
            self.canvas_1_0.image = img
            self.canvas_1_1.configure(image=img_1)
            self.canvas_1_1.image = img_1
        elif select == 2:
            self.canvas_2_0.configure(image=img)
            self.canvas_2_0.image = img
            self.canvas_2_1.configure(image=img_1)
            self.canvas_2_1.image = img_1
        elif select == 3:
            self.canvas_3_0.configure(image=img)
            self.canvas_3_0.image = img
            self.canvas_3_1.configure(image=img_1)
            self.canvas_3_1.image = img_1
        elif select == 4:
            self.canvas_4_0.configure(image=img)
            self.canvas_4_0.image = img
            self.canvas_4_1.configure(image=img_1)
            self.canvas_4_1.image = img_1

    def _test(self):
        test_arr = np.uint8(np.random.randint(255,size=(1,128)))
        self.plot(test_arr, 1)
        test_arr = np.uint8(np.random.randint(255,size=(1,128)))
        self.plot(test_arr, 2)
        test_arr = np.uint8(np.random.randint(255,size=(1,128)))
        self.plot(test_arr, 3)
        test_arr = np.uint8(np.random.randint(255,size=(1,128)))
        self.plot(test_arr, 4)

if __name__ == '__main__':
    root = Tk()
    root.title("CCD Helper")
    ccd = CCD(root)
    ccd.mainloop()
