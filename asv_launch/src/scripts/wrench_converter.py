#!/usr/bin/python
#NAME:  wrench_converter.py
#AUTHOR:  Fletcher Thompson

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Wrench

class wrench_converter:
    def __init__(self):
        rospy.init_node('cmd_wrench_conv')
        rospy.Subscriber("cmd_vel",Twist,self.callback)
        self.pub = rospy.Publisher("cmd_wrench",Wrench,queue_size=10)
    
    def callback(self,msg):
        wrench_msg = Wrench()
        wrench_msg.force.x = msg.linear.x
        wrench_msg.force.y = msg.linear.y
        wrench_msg.torque.z = msg.angular.z
        self.pub.publish(wrench_msg)

def main():
    w = wrench_converter()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

