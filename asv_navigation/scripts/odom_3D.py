#!/usr/bin/env python
import rospy
import tf_conversions
import tf2_ros
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from math import sqrt, pi
from geometry_msgs.msg import Vector3, TransformStamped, Quaternion, Point, TwistWithCovarianceStamped
import message_filters

def qv_mult(q1, v1):
    v1 = tf_conversions.transformations.unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    return tf_conversions.transformations.quaternion_multiply(
        tf_conversions.transformations.quaternion_multiply(q1, q2), 
        tf_conversions.transformations.quaternion_conjugate(q1)
    )[:3]

class odom3d:
    def __init__(self):
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_link"

        self.br = tf2_ros.TransformBroadcaster()
        self.T = TransformStamped()

        rospy.init_node("odom_unfiltered")
        self.pub = rospy.Publisher("odometry/unfiltered",Odometry,queue_size=10)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        imu_sub = message_filters.Subscriber('imu/data',Imu)
        gps_sub = message_filters.Subscriber("odometry/gps",Odometry)
        vel_sub = message_filters.Subscriber("ublox_gps/fix_velocity",TwistWithCovarianceStamped)
        ts = message_filters.ApproximateTimeSynchronizer([imu_sub, gps_sub, vel_sub], 10, 0.1)
        ts.registerCallback(self.publish)
        rospy.spin()

    def publish(self,imu_msg,gps_msg,vel_msg):
        try: 
            imu_trans = self.tfBuffer.lookup_transform(self.odom_msg.child_frame_id, imu_msg.header.frame_id, imu_msg.header.stamp)
            gps_trans = self.tfBuffer.lookup_transform(self.odom_msg.child_frame_id, "gps_link", gps_msg.header.stamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)
            return

        q_rot = (imu_trans.transform.rotation.x,imu_trans.transform.rotation.y,imu_trans.transform.rotation.z,imu_trans.transform.rotation.w)
        q_new = tf_conversions.transformations.quaternion_multiply(q_rot, (imu_msg.orientation.x,imu_msg.orientation.y,imu_msg.orientation.z,imu_msg.orientation.w))
        self.odom_msg.header.stamp = rospy.Time.now()

        self.odom_msg.pose.pose.orientation.x = q_new[0]
        self.odom_msg.pose.pose.orientation.y = q_new[1]
        self.odom_msg.pose.pose.orientation.z = q_new[2]
        self.odom_msg.pose.pose.orientation.w = q_new[3]

        self.odom_msg.pose.pose.position.x = gps_msg.pose.pose.position.x - gps_trans.transform.translation.x
        self.odom_msg.pose.pose.position.y = gps_msg.pose.pose.position.y - gps_trans.transform.translation.y
        self.odom_msg.pose.pose.position.z = gps_msg.pose.pose.position.z - gps_trans.transform.translation.z

        vel = qv_mult(q_new,(vel_msg.twist.twist.linear.x,vel_msg.twist.twist.linear.y,vel_msg.twist.twist.linear.z))
        self.odom_msg.twist.twist.linear.x = vel[0]
        self.odom_msg.twist.twist.linear.y = vel[1]
        self.odom_msg.twist.twist.linear.z = vel[2]
        
        self.pub.publish(self.odom_msg)

        self.header.stamp = rospy.Time.now()
        self.header.frame_id = "odom"
        self.child_frame_id = "base_link"
        self.transform.translation = self.odom_msg.pose.pose.position
        self.transform.rotation = self.odom_msg.pose.pose.orientation
        self.br.sendTransform(T)

if __name__ == '__main__':
    handle = odom3d()