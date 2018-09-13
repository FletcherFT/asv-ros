#!/usr/bin/python
import rospy
from asv_mission.msg import Plan, Task
from missionplanning import hierarchy

def receivePlanCallback(plan_msg):
    rospy.loginfo("Plan received: {}".format(plan_msg))
    plan = hierarchy.Hierarchy(plan_msg)
    while True:
        task = plan.getTask()
        print(task)
        if task.action == "root":
            break
        plan.taskDone()
    print "Plan complete."

def main():
    rospy.init_node("testmissionlistener")
    rospy.Subscriber("mission/plan",Plan,receivePlanCallback)
    rospy.spin()

if __name__=="__main__":
    main()