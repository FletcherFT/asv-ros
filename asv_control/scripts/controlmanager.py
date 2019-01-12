#!/usr/bin/python
import rospy
from std_msgs.msg import String
from std_srvs.srv import SetBool
from asv_messages.srv import ConfigureSteppers
from asv_messages.msg import Float64Stamped

def selectCallback(msg,pub):
    token=msg.data.split("/")[-1]
    mode_msg = Float64Stamped()
    mode_msg.header.frame_id = 'base_link'
    mode_msg.header.stamp = rospy.Time.now()
    if token=="AP":
        mode_msg.data = 1
        # disable DP controller
        try:
            rospy.wait_for_service('dynamic_position/enable',15.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('dynamic_position/enable', SetBool)
        response=service_handle(False)
        rospy.logdebug(response.message)
        # reconfigure steppers for AP mode
        try:
            rospy.wait_for_service('ardudriver/stepperconfig',15.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('ardudriver/stepperconfig', ConfigureSteppers)
        response=service_handle(True,True,True)
        rospy.logdebug(response.message)
        # reconfigure allocator for AP mode
        try:
            rospy.wait_for_service('allocator/modeconfig',15.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('allocator/modeconfig', ConfigureSteppers)
        response=service_handle(True,True,True)
        rospy.logdebug(response.message)
        # enable AP controller
        try:
            rospy.wait_for_service('autopilot/enable',15.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('autopilot/enable', SetBool)
        response=service_handle(True)
        rospy.logdebug(response.message)
        pub.publish(mode_msg)
    elif token=="override":
        mode_msg.data = 0
        # disable DP controller
        try:
            rospy.wait_for_service('dynamic_position/enable',15.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('dynamic_position/enable', SetBool)
        response=service_handle(False)
        rospy.logdebug(response.message)
        # disable AP controller
        try:
            rospy.wait_for_service('autopilot/enable',15.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('autopilot/enable', SetBool)
        response=service_handle(False)
        rospy.logdebug(response.message)
        # reconfigure steppers for AP mode
        try:
            rospy.wait_for_service('ardudriver/stepperconfig',15.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('ardudriver/stepperconfig', ConfigureSteppers)
        response=service_handle(True,True,True)
        rospy.logdebug(response.message)
        # reconfigure allocator for AP mode
        try:
            rospy.wait_for_service('allocator/modeconfig',15.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('allocator/modeconfig', ConfigureSteppers)
        response=service_handle(True,True,True)
        rospy.logdebug(response.message)
        pub.publish(mode_msg)
    elif token=="DP":
        mode_msg.data = 2
        # disable AP controller
        try:
            rospy.wait_for_service('autopilot/enable',15.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('autopilot/enable', SetBool)
        response=service_handle(False)
        rospy.logdebug(response.message)
        # reconfigure steppers for DP mode
        try:
            rospy.wait_for_service('ardudriver/stepperconfig',15.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('ardudriver/stepperconfig', ConfigureSteppers)
        response=service_handle(True,False,True)
        rospy.logdebug(response.message)
        # reconfigure allocator for DP mode
        try:
            rospy.wait_for_service('allocator/modeconfig',15.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('allocator/modeconfig', ConfigureSteppers)
        response=service_handle(True,False,True)
        rospy.logdebug(response.message)
        # enable DP controller
        try:
            rospy.wait_for_service('dynamic_position/enable',15.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('dynamic_position/enable', SetBool)
        response=service_handle(True)
        rospy.logdebug(response.message)
        pub.publish(mode_msg)

def main():
    rospy.init_node("control_manager")
    pub = rospy.Publisher("asv/mode",Float64Stamped,queue_size=1)
    rospy.Subscriber("mux/select",String,selectCallback,pub)
    rospy.spin()

if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
