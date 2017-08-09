#!/usr/bin/python
#NAME:  control_allocation.py
#AUTHOR:  Fletcher Thompson
import rospy
from geometry_msgs.msg import Wrench
from asv_control_msgs.msg import ThrusterComponents
import numpy

class allocator:
    def __init__(self):
        rospy.init_node('control_allocator')
        rospy.Subscriber("cmd_wrench",Wrench,self.callback)
        self.pub = rospy.Publisher("thrusts",ThrusterComponents,queue_size=10)
    
    def callback(self,msg):
        x = numpy.array([0,msg.force.x,msg.force.x])
        y = numpy.array([msg.force.y, msg.force.y, msg.force.y])
        thruster_components_msg = ThrusterComponents()
        thruster_components_msg.x = x.tolist()
        thruster_components_msg.y = y.tolist()
        self.pub.publish(thruster_components_msg)

def main():
    a = allocator()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
