#!/usr/bin/python
import rospy
from asv_mission.msg import Plan, Task

def main():
    rospy.init_node("testmissionpublisher")
    pub = rospy.Publisher("mission/plan",Plan,queue_size=10)
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
    plan_container[1].action="wp"
    plan_container[1].data=[-41.400675,147.121411]
    plan_container.append(Task())
    plan_container[2].parent=0
    plan_container[2].children=[]
    plan_container[2].energy=5
    plan_container[2].action="hp"
    plan_container[2].data=[-41.400675,147.121411,-1.57,30.0]
    plan.plan=plan_container
    hz = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        pub.publish(plan)
        hz.sleep()

if __name__=="__main__":
    main()