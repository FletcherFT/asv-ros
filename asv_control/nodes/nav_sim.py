#!/usr/bin/env python
# license removed for brevity
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import cos, sin
import tf
import numpy as np

inv = np.linalg.inv
mul = np.matmul

class NavSimNode():
    
    def __init__(self):
        self.inertia_mat = np.diag([100.0,200.0,1.0,1.0,1.0,300.0])
        self.a = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.damping_mat = np.diag([10.33,10.7,0.0,0.0,0.0,10.9])
        self.v = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.p = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        rospy.Subscriber('wrench',WrenchStamped,self.wrenchCallback)
        rospy.Subscriber('initialpose',PoseWithCovarianceStamped,self.setCallback)
        self.odom_pub = rospy.Publisher('odometry', Odometry, queue_size=1)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.period = rospy.rostime.Duration.from_sec(1.0/10.0)
        self.timer = rospy.Timer(self.period, self.kinematics)

    def setCallback(self,data):
        self.a = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.v = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        quaternion = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)        
        self.p = np.array(
                [data.pose.pose.position.x,
                data.pose.pose.position.y,
                data.pose.pose.position.z,
                euler[0],
                euler[1],
                euler[2]])

    def wrenchCallback(self,wrench):
        #calculate acceleration from model
        tau = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        tau[0] = wrench.wrench.force.x
        tau[1] = wrench.wrench.force.y
        tau[2] = wrench.wrench.force.z
        tau[3] = wrench.wrench.torque.x
        tau[4] = wrench.wrench.torque.y
        tau[5] = wrench.wrench.torque.z
        #body fixed acceleration
        self.a = mul(inv(self.inertia_mat),(tau-mul(self.damping_mat,self.v)))

    def kinematics(self,event):
        #calculate the time interval
        dt = self.period.to_sec()
        #update the velocity vector (body fixed)
        self.v = self.v+self.a*dt
        #calculate the new orientation
        self.p[3:] = self.p[3:]+self.v[3:]*dt
        #update the rotation matrix
        R=tf.transformations.euler_matrix(self.p[5],self.p[4],self.p[3],'szyx')
        #update the position
        self.p[0:3]=self.p[0:3]+(mul(R[0:3,0:3],self.v[0:3]))*dt
        odom_quat = tf.transformations.quaternion_from_euler(self.p[3],self.p[4],self.p[5])
        #The odometry transformation (odom -> base_link)
        self.odom_broadcaster.sendTransform((self.p[0],self.p[1],self.p[2]),odom_quat,rospy.Time.now(),'base_link','odom')

        #The odometry message
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        #set odometry position
        odom.pose.pose.position.x = self.p[0]
        odom.pose.pose.position.y = self.p[1]
        odom.pose.pose.position.z = self.p[2]
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]

        #set odometry velocity
        odom.twist.twist.linear.x = self.v[0]
        odom.twist.twist.linear.y = self.v[1]
        odom.twist.twist.linear.z = self.v[2]
        odom.twist.twist.angular.x = self.v[3]
        odom.twist.twist.angular.y = self.v[4]
        odom.twist.twist.angular.z = self.v[5]

        #publish odometry
        self.odom_pub.publish(odom)

if __name__ == '__main__':
    try:
        rospy.init_node('nav_sim')
        n = NavSimNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
