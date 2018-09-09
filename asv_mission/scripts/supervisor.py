#!/usr/bin/python
import rospy
from asv_mission.msg import Plan, Task
from sensor_msgs.msg import Joy
from topic_tools.srv import MuxSelect
from missionplanning import hierarchy

class Supervisor:
    def __init__(self):
        self.manual = False
        rospy.init_node("supervisor")
        rospy.Subscriber("plan",Plan,self.receivePlanCallback)
        rospy.Subscriber("joy",Joy,self.overridePlanCallback)
        #TODO supervisor advertises the following services:
        # 1. manual override service that's called from xbee_read.py
        # 2. 

    def doPlan(self):
        while True:
            pass
    
    def overridePlanCallback(self,joy_msg):
        if not manual:
            try:
                rospy.wait_for_service('control_topic_mux/select',5.0)
            except Exception as exc:
                rospy.logerr(exc)
                return
            service_handle = rospy.ServiceProxy('control_topic_mux/select', MuxSelect)
            response=service_handle("tau_com/override")
            rospy.logdebug(response.prev_topic)
            manual = True

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
        # while True:
        #     task = plan.getTask()
        #     print(task)
        #     if task.action == "root":
        #         break
        # print "Plan complete."

def main():
    rospy.init_node("testmissionlistener")
    rospy.Subscriber("plan",Plan,receivePlanCallback)
    rospy.spin()

if __name__=="__main__":
    main()