#!/usr/bin/env python

import rospy
from Tkinter import *
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
        b = Button(self.gui, text = "E-Stop", command=self.change_status)
        b.pack()

    def change_status(self):
        self.estop_engaged = (not self.estop_engaged)
        if not self.estop_engaged:
            ser.write(self.STOP)
        print "estop engaged:", self.estop_engaged

    def send_command(self):
        resp =  ser.read(100)
        if 'STOP' in resp:
            self.estop_engaged = True
            ser.write(self.STOP)
            print "estop engaged: True"
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