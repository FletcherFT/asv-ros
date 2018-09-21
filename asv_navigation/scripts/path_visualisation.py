#!/usr/bin/env python
# license removed for brevity
# This script calculates the standard deviation of an IMU recording.
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

class path_publisher:
    def __init__(self):
        rospy.init_node("path_publisher")
        rate=rospy.get_param("~rate",1)
        self.history = rospy.get_param("~length",1000)
        T = rospy.Duration.from_sec(1.0/rate)
        self.msg = Path()
        self.msg.poses = []
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = "odom"
        self.pub = rospy.Publisher("path/filtered",Path,queue_size=10)
        rospy.Timer(T,self.publish)
        rospy.Subscriber("odometry/filtered",Odometry,self.update)
        try:
            rospy.spin()
        except ROSTimeMovedBackwardsException:
            self.msg.poses=[]

    def update(self,msg):
        #rospy.loginfo(self.msg.poses)
        if (rospy.Time.now()-self.msg.header.stamp).to_sec()>1:
            self.msg.header.stamp = rospy.Time.now()
            P = PoseStamped()
            P.header=msg.header
            P.pose=msg.pose.pose
            self.msg.poses.append(P)
            if len(self.msg.poses)>self.history:
                self.msg.poses = self.msg.poses[1:]


    def publish(self,event):
        if self.msg.poses:
            self.pub.publish(self.msg)

if __name__=="__main__":
    path_publisher()
