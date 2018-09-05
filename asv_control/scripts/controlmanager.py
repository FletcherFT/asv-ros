#!/usr/bin/python
import rospy
from std_msgs.msg import String
from std_srvs.srv import SetBool
from asv_control_msgs.srv import ConfigureSteppers

def selectCallback(msg):
    token=msg.data.split("/")[-1]
    if token=="AP":
        # disable DP controller
        try:
            rospy.wait_for_service('dynamic_position/enable',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('dynamic_position/enable', SetBool)
        response=service_handle(False)
        rospy.logdebug(response.message)
        # reconfigure steppers for AP mode
        try:
            rospy.wait_for_service('ardudriver/stepperconfig',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('ardudriver/stepperconfig', ConfigureSteppers)
        response=service_handle(True,True,True)
        rospy.logdebug(response.message)
        # reconfigure allocator for AP mode
        try:
            rospy.wait_for_service('allocator/modeconfig',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('allocator/modeconfig', ConfigureSteppers)
        response=service_handle(True,True,True)
        rospy.logdebug(response.message)
        # enable AP controller
        try:
            rospy.wait_for_service('autopilot/enable',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('autopilot/enable', SetBool)
        response=service_handle(True)
        rospy.logdebug(response.message)
    elif token=="override":
        # disable DP controller
        try:
            rospy.wait_for_service('dynamic_position/enable',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('dynamic_position/enable', SetBool)
        response=service_handle(False)
        rospy.logdebug(response.message)
        # disable AP controller
        try:
            rospy.wait_for_service('autopilot/enable',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('autopilot/enable', SetBool)
        response=service_handle(False)
        rospy.logdebug(response.message)
        # reconfigure steppers for AP mode
        try:
            rospy.wait_for_service('ardudriver/stepperconfig',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('ardudriver/stepperconfig', ConfigureSteppers)
        response=service_handle(True,True,True)
        rospy.logdebug(response.message)
        # reconfigure allocator for AP mode
        try:
            rospy.wait_for_service('allocator/modeconfig',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('allocator/modeconfig', ConfigureSteppers)
        response=service_handle(True,True,True)
        rospy.logdebug(response.message)
    elif token=="DP":
        # disable AP controller
        try:
            rospy.wait_for_service('autopilot/enable',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('autopilot/enable', SetBool)
        response=service_handle(False)
        rospy.logdebug(response.message)
        # reconfigure steppers for DP mode
        try:
            rospy.wait_for_service('ardudriver/stepperconfig',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('ardudriver/stepperconfig', ConfigureSteppers)
        response=service_handle(True,False,True)
        rospy.logdebug(response.message)
        # reconfigure allocator for DP mode
        try:
            rospy.wait_for_service('allocator/modeconfig',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('allocator/modeconfig', ConfigureSteppers)
        response=service_handle(True,False,True)
        rospy.logdebug(response.message)
        # enable DP controller
        try:
            rospy.wait_for_service('dynamic_position/enable',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('dynamic_position/enable', SetBool)
        response=service_handle(True)
        rospy.logdebug(response.message)

def main():
    rospy.init_node("control_manager")
    rospy.Subscriber("mux/select",String,selectCallback)
    rospy.spin()

if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
