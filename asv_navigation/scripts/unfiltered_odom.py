#!/usr/bin/env python
import rospy
import numpy as np
from geodesy import utm
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TransformStamped, QuaternionStamped, PointStamped
from nav_msgs.msg import Odometry
import tf2_ros
from tf_conversions import transformations as tf
import tf
import message_filters
from scipy.linalg import block_diag

def qv_mult(q1, v1):
    q2 = list(v1)
    q2.append(0.0)
    return tf.quaternion_multiply(
        tf.quaternion_multiply(q1, q2), 
        tf.quaternion_conjugate(q1)
    )[:3]

class unfiltered_odom:
    def __init__(self):
        rospy.init_node("unfiltered_odom")
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf.TransformListener()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.pub = rospy.Publisher("odometry/unfiltered",Odometry,queue_size=10)
        imu_sub = message_filters.Subscriber('imu/data',Imu)
        nav_sub = message_filters.Subscriber("odometry/gps",Odometry)
        ts = message_filters.ApproximateTimeSynchronizer([imu_sub, nav_sub], 10, 0.25)
        ts.registerCallback(self.publish)
        rospy.spin()

    def publish(self,imu_msg,nav_msg):
        if self.tfBuffer.can_transform(nav_msg.header.frame_id,"odom",rospy.Time.now(),rospy.Duration.from_sec(0.5)):
            if self.tfBuffer.can_transform(imu_msg.header.frame_id,"base_link",rospy.Time.now(),rospy.Duration.from_sec(0.5)):
                unfiltered_msg = Odometry()
                unfiltered_msg.header.frame_id="odom"
                unfiltered_msg.child_frame_id="base_link"
                imu_q = QuaternionStamped()
                imu_q.quaternion = imu_msg.orientation
                imu_q.header = imu_msg.header
                unfiltered_msg.pose.pose.orientation = self.tfListener.transformQuaternion("base_link",imu_q).quaternion #TF2 FOR KINETIC JUST AIN'T WORKING
                nav_p = PointStamped()
                nav_p.point = nav_msg.pose.pose.position
                nav_p.header = nav_msg.header
                unfiltered_msg.pose.pose.position = self.tfListener.transformPoint("odom",nav_p).point
                self.pub.publish(unfiltered_msg)
            else:
                rospy.logwarn("{} to base_link tf unavailable!".format(imu_msg.header.frame_id))
        else:
            rospy.logwarn("{} to odom tf unavailable!".format(nav_msg.header.frame_id))

if __name__ == '__main__':
    unfiltered_odom()