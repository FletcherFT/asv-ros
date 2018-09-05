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
        self.test = rospy.get_param("~test",False)
        mega_dev=rospy.get_param("~mega_device","/dev/ttyACM0")
        uno_dev = rospy.get_param("~uno_device","/dev/ttyACM1")
        mega_baud=rospy.get_param("~mega_baud",115200)
        uno_baud =rospy.get_param("~uno_baud",115200)
        if not self.test:
            self.mega = serial.Serial(mega_dev,mega_baud,timeout=0)
            self.uno = serial.Serial(uno_dev,uno_baud,timeout=0)
        self.stepper_server = rospy.Service('~stepperconfig',ConfigureSteppers,self.steppers)
        if not self.test and not self.mega.is_open:
            self.mega.open()
        if not self.test and not self.uno.is_open:
            self.uno.open()
        rospy.Subscriber("thrusters",Thrusters,self.update)
        rospy.spin()

    def steppers(self,request):
        """
        Handles ROS service requests for zeroing, enabling/disabling, and changing stepper mode.
        Returns success flag and message.
        """
        formatspec="cccc"
        try:
            data = ("s",str(int(request.zero)),str(int(request.mode)),str(int(request.enable)))
            data_msg = struct.pack(formatspec,*data)
            if not self.test:
                self.uno.write(data_msg)
            rospy.logdebug("Uno: sending "+data_msg)
        except Exception as exc:
            response=[False,str(exc)]
        else:
            response=[True,"Stepper configuration message sent to drivers."]
        return response

    def update(self,msg):
        try:
            data_msg="s,{},{},{}".format(*msg.pwm)
            rospy.logdebug("Mega: sending "+data_msg)
            if not self.test:
                self.mega.write(data_msg)
        except Exception as e:
            rospy.logerr(e)

if __name__=="__main__":
    try:
        h = ardudriver()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr(exc)
    finally:
        if not h.test and h.mega.is_open:
            h.mega.close()
        if not h.test and h.uno.is_open:
            h.uno.close()
