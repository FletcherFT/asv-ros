#!/usr/bin/python
import rospy
import serial
import struct
from std_srvs.srv import Trigger
from asv_control_msgs.msg import Thrusters
from asv_control_msgs.srv import ConfigureSteppers

class ardudriver:
    def __init__(self):
        rospy.init_node('ardudriver')
        mega_dev=rospy.get_param("~mega_device","/dev/ttyACM0")
        uno_dev = rospy.get_param("~uno_device","/dev/ttyACM1")
        mega_baud=rospy.get_param("~mega_baud",115200)
        uno_baud =rospy.get_param("~uno_baud",115200)
        self.mega = serial.Serial(mega_dev,mega_baud,timeout=0)
        self.uno = serial.Serial(uno_dev,uno_baud,timeout=0)
        self.stepper_server = rospy.Service('stepperconfig',ConfigureSteppers,self.steppers)
        rospy.Subscriber("thrusters",Thrusters,self.update)

    def steppers(self,request):
        """
        Handles ROS service requests for zeroing, enabling/disabling, and changing stepper mode.
        Returns success flag and message.
        """
        formatspec="c???"
        try:
            data = ("s",request.zero,request.mode,request.enable)
            data_msg = struct.pack(formatspec,data)
            with self.uno as ser:
                ser.write(data_msg)
        except Exception as exc:
            response=[False,exc]
        else:
            response=[True,"Stepper configuration message sent to drivers."]
        return response

    def update(self,msg):
        try:
            data_msg="s,{},{},{}".format(*msg.pwm)
            rospy.logdebug(data_msg)
            with self.mega as ser:
                ser.write(data_msg)
        except Exception as e:
            rospy.logerr(e)

if __name__=="__main__":
    try:
        h = ardudriver()
    except rospy.ROSInterruptException:
        if h.mega.is_open():
            h.mega.close()
        if h.uno.is_open():
            h.uno.close()