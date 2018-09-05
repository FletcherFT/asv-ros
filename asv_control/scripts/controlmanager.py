#!/usr/bin/python
import rospy
from std_msgs.msg import String
from std_srvs.srv import SetBool
from asv_msgs.srv import ConfigureSteppers
import re

def selectCallback(msg):
    token=msg.split("/")[-1]
    if token=="AP":
        # disable DP controller
        try:
            rospy.wait_for_service('dynamic_position/enable',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('dynamic_position/enable', SetBool)
        response=service_handle(False)
        # reconfigure steppers for AP mode
        try:
            rospy.wait_for_service('ardudriver/stepperconfig',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('ardudriver/stepperconfig', ConfigureSteppers)
        response=service_handle(True,True,True)
        # reconfigure allocator for AP mode
        try:
            rospy.wait_for_service('allocator/modeconfig',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('allocator/modeconfig', ConfigureSteppers)
        response=service_handle(True,True,True)
        # enable AP controller
        try:
            rospy.wait_for_service('autopilot/enable',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('autopilot/enable', SetBool)
        response=service_handle(True)
    elif token=="override":
        # disable DP controller
        try:
            rospy.wait_for_service('dynamic_position/enable',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('dynamic_position/enable', SetBool)
        response=service_handle(False)
        # disable AP controller
        try:
            rospy.wait_for_service('autopilot/enable',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('autopilot/enable', SetBool)
        response=service_handle(False)
        # reconfigure steppers for AP mode
        try:
            rospy.wait_for_service('ardudriver/stepperconfig',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('ardudriver/stepperconfig', ConfigureSteppers)
        response=service_handle(True,True,True)
        # reconfigure allocator for AP mode
        try:
            rospy.wait_for_service('allocator/modeconfig',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('allocator/modeconfig', ConfigureSteppers)
        response=service_handle(True,True,True)
    elif token=="DP":
        # disable AP controller
        try:
            rospy.wait_for_service('autopilot/enable',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('autopilot/enable', SetBool)
        response=service_handle(False)
        # reconfigure steppers for DP mode
        try:
            rospy.wait_for_service('ardudriver/stepperconfig',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('ardudriver/stepperconfig', ConfigureSteppers)
        response=service_handle(True,False,True)
        # reconfigure allocator for DP mode
        try:
            rospy.wait_for_service('allocator/modeconfig',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('allocator/modeconfig', ConfigureSteppers)
        response=service_handle(True,False,True)
        # enable DP controller
        try:
            rospy.wait_for_service('dynamic_position/enable',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('dynamic_position/enable', SetBool)
        response=service_handle(True)

def main():
    rospy.init_node("control_manager")
    rospy.Subscriber("mux/select",String,selectCallback)
    rospy.spin()

if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass