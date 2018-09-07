#!/usr/bin/python
import rospy
from asv_mission.msg import Mission, Task

def main():
    pub = rospy.Publisher("plan",Mission,queue_size=1)
    mission=Mission()
    mission.header.stamp=rospy.Time.now()
    mission.tasks[0]=Task()
    mission.tasks[0].children=[1,2]
    mission.tasks[0].energy=20
    mission.tasks[0].action=""
    mission.tasks[0].data=[]
    mission.tasks[1]=Task()
    mission.tasks[1].children=[]
    mission.tasks[1].energy=15
    mission.tasks[1].action="waypoint"
    mission.tasks[1].data=[-41.400419,147.125098]
    mission.tasks[2]=Task()
    mission.tasks[2].children=[]
    mission.tasks[2].energy=5
    mission.tasks[2].action="dp"
    mission.tasks[2].data=[-41.400419,147.125098,0.0,30.0]
    pub.publish(mission)
    rospy.spin()

if __name__=="__main__":
    main()