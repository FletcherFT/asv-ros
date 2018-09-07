#!/usr/bin/python
import rospy
from asv_mission.msg import Plan, Task

def main():
    rospy.init_node("testmissionpublisher")
    pub = rospy.Publisher("plan",Plan,queue_size=1)
    plan=Plan()
    plan.header.stamp=rospy.Time.now()
    plan_container=[]
    plan_container.append(Task())
    plan_container[0].parent=-1
    plan_container[0].children=[1,2]
    plan_container[0].energy=20
    plan_container[0].action="root"
    plan_container[0].data=[]
    plan_container.append(Task())
    plan_container[1]=Task()
    plan_container[1].parent=0
    plan_container[1].children=[]
    plan_container[1].energy=15
    plan_container[1].action="waypoint"
    plan_container[1].data=[-41.400419,147.125098]
    plan_container.append(Task())
    plan_container[2].parent=0
    plan_container[2].children=[]
    plan_container[2].energy=5
    plan_container[2].action="dp"
    plan_container[2].data=[-41.400419,147.125098,0.0,30.0]
    plan.plan=plan_container
    hz = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        pub.publish(plan)
        hz.sleep()

if __name__=="__main__":
    main()