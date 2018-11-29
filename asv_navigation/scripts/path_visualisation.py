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
        self.previous_time = rospy.Time.now()
        T = rospy.Duration.from_sec(1.0/rate)
        self.msg = Path()
        self.msg.poses = []
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = "odom"
        self.pub = rospy.Publisher("path/filtered",Path,queue_size=10)
        rospy.Subscriber("odometry/filtered",Odometry,self.update)
        rospy.spin()

    def update(self,msg):
        #rospy.loginfo(self.msg.poses)
        try:
            if (msg.header.stamp-self.previous_time).to_sec()>1:
                self.msg.header.stamp = msg.header.stamp
                self.previous_time=msg.header.stamp
                P = PoseStamped()
                P.header=msg.header
                P.pose=msg.pose.pose
                self.msg.poses.append(P)
                if len(self.msg.poses)>self.history:
                    self.msg.poses = self.msg.poses[1:]
                self.pub.publish(self.msg)
        except rospy.ROSTimeMovedBackwardsException:
            self.msg.poses=[]
            self.msg.header.stamp = msg.header.stamp
            self.previous_time = msg.header.stamp
            P = PoseStamped()
            P.header=msg.header
            P.pose=msg.pose.pose
            self.msg.poses.append(P)
            if len(self.msg.poses)>self.history:
                self.msg.poses = self.msg.poses[1:]
            self.pub.publish(self.msg)


if __name__=="__main__":
    path_publisher()
