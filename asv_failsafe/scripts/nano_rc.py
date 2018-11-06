#!/usr/bin/python
import rospy
import serial
import struct
import numpy as np
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool, Trigger

class nano_rc():
    def __init__(self):
        rospy.init_node("rc_read")
        self._PORT = rospy.get_param("~device","/dev/ttyUSB0")
        self._BAUD = rospy.get_param("~baud",9600)
        self._TIMEOUT = rospy.get_param("~timeout",1.0)
        self.joy_msg = Joy()
        self.pub = rospy.Publisher("joy",Joy,queue_size=10)
        self.main()

    def main(self):
        try:
            fmt = ">BBBBBB"
            size = struct.calcsize(fmt)
            with serial.Serial(self._PORT,self._BAUD,timeout=self._TIMEOUT) as ser:
                while not rospy.is_shutdown():
                    line = ser.readline().rstrip()
                    if not line is None and len(line)==size:
                        data = struct.unpack(fmt,line)
                        rospy.logdebug(data)
                        self.parse_joystick(data)
        except rospy.ROSInterruptException:
            pass

    def parse_joystick(self,data):
        data = np.array(data)
        data = data/255.0
        ax = data[0:4]
        but = data[4:]
        but[but<0.5]=0
        but[but>0.5]=1
        self.joy_msg.axes = ax
        self.joy_msg.buttons = but
        rospy.logdebug(data)
        self.joy_msg.header.stamp = rospy.Time.now()
        self.joy_msg.header.frame_id = "override"
        self.pub.publish(self.joy_msg)

if __name__=="__main__":
    nano_rc()