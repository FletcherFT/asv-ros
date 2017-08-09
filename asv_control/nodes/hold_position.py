#!/usr/bin/python
import message_filters
import rospy
import asv_control_msgs.srv
from tf import transformations as transform
from pid import Pid
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import numpy as np

mul = np.matmul

class PoseControllerNode():
    """
    Node for controlling the pose (position and orientation) of a robot,
    using a simple PID scheme. 
    ECEF positions X,Y,Z are converted to body fixed errors (dx,dy,dz).
    These errors are inputted as the state for surge, sway, heave PIDs
    (only surge, sway right now).
    Euler angle of vessel is inputted as the state for roll, pitch,
    yaw PIDs (only yaw right now).
    Desired euler angles are the setpoint for rpy PIDs.
    Setpoints for surge, sway, heave PIDs are always 0.
    Output is a message of type geometry_msgs/WrenchStamped on topic 
    'wrench' containing force and torque values that are
    necessary to maintain the position.
    """

    def __init__(self, frequency):
        self.setpoint_valid = False
        self.enabled = False
        self.enable_server = rospy.Service('~enable', asv_control_msgs.srv.EnableControl, self.enable)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.surgeFlag=False
        self.swayFlag=False
        self.yawFlag = False

        self.end_pose = PoseStamped()
        
        self.setpoint_x_pub = rospy.Publisher('~surge/setpoint', Float64,queue_size=1)
        self.setpoint_y_pub = rospy.Publisher('~sway/setpoint', Float64,queue_size=1)
        self.setpoint_yaw_pub = rospy.Publisher('~yaw/setpoint', Float64,queue_size=1)
        self.state_x_pub = rospy.Publisher('~surge/state', Float64,queue_size=1)
        self.state_y_pub = rospy.Publisher('~sway/state', Float64,queue_size=1)
        self.state_yaw_pub = rospy.Publisher('~yaw/state', Float64,queue_size=1)

        self.wrench_pub = rospy.Publisher('~wrench',WrenchStamped,queue_size=1)

        rospy.Subscriber('~surge/control_effort',Float64,self.surgeCallback)
        rospy.Subscriber('~sway/control_effort',Float64,self.swayCallback)
        rospy.Subscriber('~yaw/control_effort',Float64,self.yawCallback)

        rospy.Subscriber('~pose_request', PoseStamped, self.setpointCallback)
        rospy.Subscriber('odometry', Odometry, self.odometryCallback)
        
        rospy.loginfo('Listening for pose feedback to be published on '
                      '%s...', rospy.resolve_name('odometry'))
        rospy.loginfo('Waiting for setpoint to be published on '
                      '%s...', rospy.resolve_name('~pose_request'))

    def surgeCallback(self,x):
        self.surgeFlag=True
        self.x = x.data
        if self.surgeFlag and self.swayFlag and self.yawFlag:
            self.surgeFlag=False
            self.swayFlag=False
            self.yawFlag=False
            self.wrenchCallback()
        
    def swayCallback(self,y):
        self.swayFlag=True
        self.y = y.data
        if self.surgeFlag and self.swayFlag and self.yawFlag:
            self.surgeFlag=False
            self.swayFlag=False
            self.yawFlag=False
            self.wrenchCallback()

    def yawCallback(self,yaw):
        self.yawFlag=True
        self.yaw = yaw.data
        if self.surgeFlag and self.swayFlag and self.yawFlag:
            self.surgeFlag=False
            self.swayFlag=False
            self.yawFlag=False
            self.wrenchCallback()

    def wrenchCallback(self):
        wrench_msg = WrenchStamped()
        wrench_msg.wrench.force.x = self.x
        wrench_msg.wrench.force.y = self.y
        wrench_msg.wrench.torque.z = self.yaw
        wrench_msg.header.stamp = rospy.Time.now()
        wrench_msg.header.frame_id = 'base_link'
        self.wrench_pub.publish(wrench_msg)

    def enable(self, request):
        """
        Handles ROS service requests for enabling/disabling control.
        Returns current enabled status and setpoint.
        """
        response = asv_control_msgs.srv.EnableControlResponse()
        if request.enable:
            if self.isFeedbackValid():
                self.enabled = True
                self.setpoint_valid = True
                response.enabled = True
            else:
                rospy.logerr("Cannot enable pose control without valid feedback!")
                response.enabled = False
        else:
            self.enabled = False
            response.enabled = False
        return response
    
    def setpointCallback(self,setpoint):
        """
        Change the setpoint of the controller.
        """
        if not self.setpoint_valid:
            rospy.loginfo("First setpoint received.")
            self.setpoint_valid = True
        #save the desired pose
        self.end_pose = setpoint
        
        self.setSetpoint(setpoint.pose)
        rospy.loginfo('Changed setpoint to: %s', setpoint.pose)

    def setSetpoint(self, pose):
        #publish the setpoints to the pid nodes
        quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
        euler = transform.euler_from_quaternion(quaternion)
        self.setpoint_x_pub.publish(0.0)
        self.setpoint_y_pub.publish(0.0)
        self.setpoint_yaw_pub.publish(euler[2])

    def odometryCallback(self, odometry_msg):
        #publish the odometry to the pid nodes
        quaternion = (
        odometry_msg.pose.pose.orientation.x,
        odometry_msg.pose.pose.orientation.y,
        odometry_msg.pose.pose.orientation.z,
        odometry_msg.pose.pose.orientation.w)
        euler = transform.euler_from_quaternion(quaternion)

        #get the earth fixed distances as a vector from the current position
        dX = odometry_msg.pose.pose.position.x-self.end_pose.pose.position.x
        dY = odometry_msg.pose.pose.position.y-self.end_pose.pose.position.y
        dZ = odometry_msg.pose.pose.position.z-self.end_pose.pose.position.z
        dP = np.array([dX,dY,dZ])
        
        #get the rotation matrix from the quaternion
        R = transform.quaternion_matrix(quaternion)[0:3,0:3].T

        #transform the earth fixed distances to body fixed distances
        dp = mul(R,dP)
        self.state_x_pub.publish(dp[0])
        self.state_y_pub.publish(dp[1])
        self.state_yaw_pub.publish(euler[2])

if __name__ == "__main__":
    rospy.init_node('pose_controller')
    try:
        frequency = rospy.get_param("~frequency", 10.0)
        rospy.loginfo('Starting pose control with %f Hz.\n', frequency)
        node = PoseControllerNode(frequency)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
