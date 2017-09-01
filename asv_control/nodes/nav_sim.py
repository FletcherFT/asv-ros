#!/usr/bin/env python
# license removed for brevity
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import cos, sin
import tf
import fossen
import numpy as np
inv = np.linalg.inv
mul = np.matmul

class NavSimNode():
    
    def __init__(self):
        #inertial matrix
        self.M_iner = np.diag([15.0,15.0,15.0,100.0,100.0,100.0])
        if rospy.has_param('Mi'):
            self.M_iner = rospy.get_param('Mi')
            if len(self.M_iner)<7:
                self.M_iner = np.diag(self.M_iner)
            else:
                self.M_iner = np.reshape(self.M_iner,(6,6))
        self.M_addm = np.zeros([6,6])
        #added mass matrix
        if rospy.has_param('Ma'):
            self.M_addm = rospy.get_param('Ma')
            if len(self.M_addm)<7:
                self.M_addm = np.diag(self.M_addm)
            else:
                self.M_addm = np.reshape(self.M_addm,(6,6))
        #initialise acceleration vector
        self.a = np.zeros(6)

        #velocity terms
        #coriolis and centripetal matrix due to inertia (this is a function of the inertial matrix and velocity)
        self.C_iner = np.zeros([6,6])
        #coriolis and centripetal matrix due to added mass (this is a function of the added mass matrix and vel)
        self.C_addm = np.zeros([6,6])
        #total damping matrix (potential + skin + wave damping coefficients)
        self.D = np.diag([10.33,10.7,0.0,0.0,0.0,10.9])
        if rospy.has_param('D'):
            self.D = rospy.get_param('D')
            if len(self.D)<7:
                self.D = np.diag(self.D)
            else:
                self.D = np.reshape(self.D,(6,6))
        #initialise velocity vector
        self.v = np.zeros(6)

        #position terms
        self.G = np.zeros(6)
        self.p = np.zeros(6)

        #external force terms
        self.tau = np.zeros(6)

        #rospy subscribers + publisher
        rospy.Subscriber('wrench',WrenchStamped,self.wrenchCallback)
        rospy.Subscriber('initialpose',PoseWithCovarianceStamped,self.setCallback)
        self.odom_pub = rospy.Publisher('odometry', Odometry, queue_size=1)
        self.odom_broadcaster = tf.TransformBroadcaster()
        frequency = 10.0
        if rospy.has_param('frequency'):
            frequency = rospy.get_param('frequency')
        self.period = rospy.rostime.Duration.from_sec(1.0/frequency)
        self.timer = rospy.Timer(self.period, self.kinematics)

    def setCallback(self,data):
        self.a = np.zeros(6)
        self.v = np.zeros(6)
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
        self.tau[0] = wrench.wrench.force.x
        self.tau[1] = wrench.wrench.force.y
        self.tau[2] = wrench.wrench.force.z
        self.tau[3] = wrench.wrench.torque.x
        self.tau[4] = wrench.wrench.torque.y
        self.tau[5] = wrench.wrench.torque.z

        #update the coriolis matrix from the old velocity
        self.C_iner = fossen.m2c(self.M_iner,self.v)
        #update the added mas matrix from the old velocity
        self.C_addm = fossen.m2c(self.M_addm,self.v)
        #update Gvector from old position TODO:  see fossen.gvect for what needs to be calculated
        #self.G = fossen.gvect(

    def kinematics(self,event):
        #consolidate velocity and acceleration terms
        M = self.M_iner+self.M_addm
        C = self.C_addm + self.C_iner + self.D
        G = self.G
        #calculate the fixed acceleration (based off old velocity and position)
        self.a = mul(inv(M),self.tau-mul(C,self.v)-mul(G,self.p))

        #calculate the time interval
        dt = self.period.to_sec()
        #update the velocity vector (body fixed) from new acceleration
        self.v = self.v+self.a*dt

        #calculate the new orientation
        self.p[3:] = self.p[3:]+self.v[3:]*dt

        #obtain the rotation matrix by rotating through the euler angles
        R=tf.transformations.euler_matrix(self.p[5],self.p[4],self.p[3],'szyx')

        #update the position
        self.p[0:3]=self.p[0:3]+(mul(R[0:3,0:3],self.v[0:3]))*dt
        #get the orientation as a quaternion
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
