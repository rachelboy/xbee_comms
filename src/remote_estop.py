#!/usr/bin/env python

import rospy
from Tkinter import *
import tkFont
import serial
import time

class EStop():

    def __init__(self, serial):
        rospy.init_node('remote_estop', anonymous=True)

        self.estop_engaged = False
        self.serial = serial
        self.gui = Tk()

        self.OK = rospy.get_param("OK_msg", "A")
        self.STOP = rospy.get_param("STOP_msg", "B")

        self.gui.after(200, self.send_command)
        self.b = Button(self.gui, 
            text="Engage E-Stop", 
            command=self.change_status, 
            bg="red",
            activebackground="red",
            height=2,
            width=15,
            font=tkFont.Font(family="Times", size="60"))
        self.b.pack()

    def change_status(self):
        if self.estop_engaged:
            self.disengage_estop()
        else:
            self.engage_estop()

    def disengage_estop(self):
        self.estop_engaged = False
        self.b["text"] = "Engage E-Stop"
        self.b["bg"] = "red"
        self.b["activebackground"]="red",
        print "estop engaged: False"
        self.b.pack()

    def engage_estop(self):
        self.estop_engaged = True
        self.serial.write(self.STOP)
        self.b["text"] = "Disengage E-Stop"
        self.b["bg"] = "green"
        self.b["activebackground"]="green",
        print "estop engaged: True"
        self.b.pack()


    def send_command(self):
        resp =  ser.read(100)
        if 'STOP' in resp:
            self.engage_estop()
        print resp

        if not self.estop_engaged:
            ser.write(self.OK)
        else:
            ser.write(self.STOP)

        self.gui.after(200, self.send_command)

    def execute(self):
        try:
            self.gui.mainloop()
        except KeyboardInterrupt:
            self.serial.close()

if __name__ == "__main__":
    ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=0)
    estop = EStop(ser)
    estop.execute()