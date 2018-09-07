#!/usr/bin/python
import sys
print(sys.path)

import rospy
from asv_mission.msg import Plan, Task
from missionplanning import hierarchy

def receivePlanCallback(plan_msg):
    plan = hierarchy.Hierarchy(plan_msg)
    print plan.getTask()

def main():
    rospy.init_node("testmissionlistener")
    rospy.Subscriber("plan",Plan,receivePlanCallback)
    rospy.spin()

if __name__=="__main__":
    main()