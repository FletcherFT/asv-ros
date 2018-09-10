#!/usr/bin/python
import rospy
from asv_mission.msg import Plan, Task
from sensor_msgs.msg import Joy
from geographic_msgs.msg import GeoPoseStamped
from topic_tools.srv import MuxSelect
from missionplanning import hierarchy

class Supervisor:
    def __init__(self):
        self.paused = False
        self.manual = False
        rospy.init_node("supervisor")
        rospy.Subscriber("mission/plan",Plan,self.receivePlanCallback)
        rospy.Subscriber("guidance/operator",GeoPoseStamped,self.receiveOperatorCallback)
        rospy.Subscriber("joy",Joy,self.receiveManualCallback)
        rospy.spin()
    
    def receiveManualCallback(self,joy_msg):
        # if we've received a joystick message and the mission isn't paused, pause the mission and configure for override mode.
        flag = False
        if not self.paused:
            self.manual = True
            self.paused = True
            flag = True
        elif self.paused and not self.manual:
            # joystick received, but vehicle in operator override mode, so override that with manual control.
            self.manual = True
            flag = True
        if flag:
            try:
                rospy.wait_for_service('control_topic_mux/select',5.0)
            except Exception as exc:
                rospy.logerr(exc)
                return
            service_handle = rospy.ServiceProxy('control_topic_mux/select', MuxSelect)
            response=service_handle("tau_com/override")
            rospy.logdebug("{}: Switching to manual override, now publishing {} from {}".format(rospy.get_name(),rospy.resolve_name("override"),response.prev_topic))

    def receiveOperatorCallback(self,geo_msg):
        # if we've receive an operator override message and the mission isn't paused or in manual override, pause it.
        flag = False
        if not self.paused and not self.manual:
            self.paused = True
            flag = True
        elif self.paused and not self.manual:
            flag = True
        elif self.paused and self.manual:
            rospy.logwarn("In manual override mode, you must call resume service to get out of it.")
        if flag:
            try:
                rospy.wait_for_service('control_topic_mux/select',5.0)
            except Exception as exc:
                rospy.logerr(exc)
                return
            service_handle = rospy.ServiceProxy('control_topic_mux/select', MuxSelect)
            response=service_handle("tau_com/AP")
            rospy.logdebug("{}: Switching to operator override, now publishing {} from {}".format(rospy.get_name(),rospy.resolve_name("AP"),response.prev_topic))

    def newTask(self,task):
        if task.action == "wp":
            # waypoint task, configure for AP mode to a GPS waypoint.
            pass
        elif task.action == "hp":
            # hold position task, configure DP mode at a GPS waypoint and orientation for a set time.
            pass
        elif task.action == "root":
            # root task reached, notify the operator that the mission is complete.
            pass
        else:
            print("Task of type {} is not supported, getting next task.".format(task.action))

    def receivePlanCallback(self,plan_msg):
        # parse the received Plan message
        self.plan = hierarchy.Hierarchy(plan_msg)

if __name__=="__main__":
    try:
        supervisor = Supervisor()
    except rospy.ROSInterruptException:
        pass