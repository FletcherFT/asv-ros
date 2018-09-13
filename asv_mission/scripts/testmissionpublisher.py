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
    plan_container[-1].parent=-1
    plan_container[-1].children=[1,2,3,4,5,6,7,8]
    plan_container[-1].energy=40
    plan_container[-1].action="root"
    plan_container[-1].data=[]
    plan_container.append(Task())
    plan_container[-1]=Task()
    plan_container[-1].parent=0
    plan_container[-1].children=[]
    plan_container[-1].energy=15
    plan_container[-1].action="wp"
    plan_container[-1].data=[-41.398475, 147.119728]
    plan_container.append(Task())
    plan_container[-1].parent=0
    plan_container[-1].children=[]
    plan_container[-1].energy=5
    plan_container[-1].action="hp"
    plan_container[-1].data=[-41.398475, 147.119728,1.57,10.0]
    plan_container.append(Task())
    plan_container[-1]=Task()
    plan_container[-1].parent=0
    plan_container[-1].children=[]
    plan_container[-1].energy=15
    plan_container[-1].action="wp"
    plan_container[-1].data=[-41.398395, 147.119747]
    plan_container.append(Task())
    plan_container[-1].parent=0
    plan_container[-1].children=[]
    plan_container[-1].energy=5
    plan_container[-1].action="hp"
    plan_container[-1].data=[-41.398395, 147.119747,0.0,10.0]
    plan_container.append(Task())
    plan_container[-1]=Task()
    plan_container[-1].parent=0
    plan_container[-1].children=[]
    plan_container[-1].energy=15
    plan_container[-1].action="wp"
    plan_container[-1].data=[-41.398385, 147.119822]
    plan_container.append(Task())
    plan_container[-1].parent=0
    plan_container[-1].children=[]
    plan_container[-1].energy=5
    plan_container[-1].action="hp"
    plan_container[-1].data=[-41.398385, 147.119822,-1.57,10.0]
    plan_container.append(Task())
    plan_container[-1]=Task()
    plan_container[-1].parent=0
    plan_container[-1].children=[]
    plan_container[-1].energy=15
    plan_container[-1].action="wp"
    plan_container[-1].data=[-41.398489, 147.119799]
    plan_container.append(Task())
    plan_container[-1].parent=0
    plan_container[-1].children=[]
    plan_container[-1].energy=5
    plan_container[-1].action="hp"
    plan_container[-1].data=[-41.398489, 147.119799,0,10.0]
    plan.plan=plan_container
    hz = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        pub.publish(plan)
        hz.sleep()

if __name__=="__main__":
    main()