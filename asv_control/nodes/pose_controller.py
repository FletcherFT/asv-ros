#!/usr/bin/python
import rospy
import dynamic_reconfigure.server
import asv_control_msgs.srv
from asv_control.cfg import PoseControllerConfig
from tf import transformations as transform
from pid import Pid
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from math import pi
import numpy as np
mul=np.matmul

class PoseControllerNode():
    """
    Node for controlling the pose (position and orientation) of a robot,
    using a simple PID scheme. The input consists in a setpoint (required 
    position) given as geometry_msgs/PoseStamped on topic '~pose_request' 
    and of current odometry readings as nav_msgs/Odometry on topic 'odometry'.
    Every degree of freedom has a separate PID. The pose values given in
    the incoming odometry message are used as feedback for the PIDs.
    Output is a message of type geometry_msgs/TwistStamped on topic 
    '~twist_commanded' containing linear and angular velocity values that are
    necessary to maintain the position..
    """
    def __init__(self, frequency):
        self.P = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.FEEDBACK_TIMEOUT = 1.0
        self.setpoint_valid = False
        self.feedback_received = False
        self.enabled = False
        self.last_feedback = Odometry()
        self.last_feedback_time = rospy.Time.now()
        self.enable_server = rospy.Service('~enable', asv_control_msgs.srv.EnableControl, self.enable)
        self.pids = []
        for i in range(6):
            self.pids.append(Pid(0.0, 0.0, 0.0))
        self.server = dynamic_reconfigure.server.Server(PoseControllerConfig, self.reconfigure)
        
        self.pub = rospy.Publisher('~twist_commanded', TwistStamped,queue_size=1)
        rospy.Subscriber('~pose_request', PoseStamped, self.setpointCallback)
        
        period = rospy.rostime.Duration.from_sec(1.0/frequency)
        self.timer = rospy.Timer(period, self.updateOutput)
        
        rospy.Subscriber('odometry', Odometry, self.odometryCallback)
        rospy.loginfo('Listening for pose feedback to be published on '
                      '%s...', rospy.resolve_name('odometry'))
        rospy.loginfo('Waiting for setpoint to be published on '
                      '%s...', rospy.resolve_name('~pose_request'))

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

    def reconfigure(self, config, level):
        """
        Handles dynamic reconfigure requests.
        """
        rospy.loginfo("Reconfigure request...")
        self.pids[0].k_p = config['linear_x_Kp']
        self.pids[0].k_i = config['linear_x_Ki']
        self.pids[0].k_d = config['linear_x_Kd']
        self.pids[1].k_p = config['linear_y_Kp']
        self.pids[1].k_i = config['linear_y_Ki']
        self.pids[1].k_d = config['linear_y_Kd']
        self.pids[2].k_p = config['linear_z_Kp']
        self.pids[2].k_i = config['linear_z_Ki']
        self.pids[2].k_d = config['linear_z_Kd']
        self.pids[3].k_p = config['angular_x_Kp']
        self.pids[3].k_i = config['angular_x_Ki']
        self.pids[3].k_d = config['angular_x_Kd']
        self.pids[4].k_p = config['angular_y_Kp']
        self.pids[4].k_i = config['angular_y_Ki']
        self.pids[4].k_d = config['angular_y_Kd']
        self.pids[5].k_p = config['angular_z_Kp']
        self.pids[5].k_i = config['angular_z_Ki']
        self.pids[5].k_d = config['angular_z_Kd']
        return config # Returns the updated configuration.
    
    def setpointCallback(self,setpoint):
        """
        Change the setpoint of the controller.
        """
        if not self.setpoint_valid:
            rospy.loginfo("First setpoint received.")
            self.setpoint_valid = True
        self.setSetpoint(setpoint.pose)
        if not self.enabled:
            rospy.logwarn("PIDs not enabled, please call /pose_controller/enable service")
        rospy.loginfo('Changed setpoint to: %s', setpoint.pose)

    def setSetpoint(self, pose):
        #these are the setpoints
        self.pids[0].setSetpoint(0.0)
        self.pids[1].setSetpoint(0.0)
        self.pids[2].setSetpoint(0.0)
        quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
        euler = transform.euler_from_quaternion(quaternion)
        #roll pitch yaw pids (always set to 0)
        self.pids[3].setSetpoint(0)
        self.pids[4].setSetpoint(0)
        self.pids[5].setSetpoint(0)
        
        #set the global position vector
        self.P = np.array([pose.position.x,pose.position.y,pose.position.z,euler[0],euler[1],euler[2]])

    def odometryCallback(self, odometry_msg):
        #Obtain the inputs from the odometry
        #Orientation
        quaternion = (
        odometry_msg.pose.pose.orientation.x,
        odometry_msg.pose.pose.orientation.y,
        odometry_msg.pose.pose.orientation.z,
        odometry_msg.pose.pose.orientation.w)
        euler = transform.euler_from_quaternion(quaternion)

        #get the earth fixed position errors and euler angle errors
        dP = np.array([ odometry_msg.pose.pose.position.x-self.P[0],
                        odometry_msg.pose.pose.position.y-self.P[1],
                        odometry_msg.pose.pose.position.z-self.P[2],
                        euler[0]-self.P[3],
                        euler[1]-self.P[4],
                        euler[2]-self.P[5]])

        #get the rotation matrix from the quaternion
        R = transform.quaternion_matrix(quaternion)[0:3,0:3].T

        #ensure the discontinuous angle error measurements are bounded correctly
        for idx,val in enumerate(dP[3:]):
            #handle the angle wrap
            if abs(val)>pi:
                dP[idx+3]=-dP[idx+3]
                #the derivative and proportional errors should be reset here
                self.pids[idx+3].__integral=0.0
                self.pids[idx+3].__derivative=0.0
                self.pids[idx+3].__previouserror=0.0

        #transform the earth fixed distances to body fixed distances retain the euler angles
        errors = np.append(mul(R,dP[0:3]),dP[3:])

        self.last_feedback = errors
        self.last_feedback_time = odometry_msg.header.stamp
        
    def isFeedbackValid(self):
        feedback_in_time = (rospy.Time.now() - 
            self.last_feedback_time).to_sec() < self.FEEDBACK_TIMEOUT
        return feedback_in_time 
        
    def updateOutput(self, event):
        if self.setpoint_valid and self.enabled:
            twist_output = TwistStamped()
            if self.isFeedbackValid():
                dt = (event.current_real - event.last_real).to_sec()
                twist_output.twist.linear.x = self.pids[0].update(self.last_feedback[0], dt)
                twist_output.twist.linear.y = self.pids[1].update(self.last_feedback[1], dt)
                twist_output.twist.linear.z = self.pids[2].update(self.last_feedback[2], dt)
                twist_output.twist.angular.x = self.pids[3].update(self.last_feedback[3], dt)
                twist_output.twist.angular.y = self.pids[4].update(self.last_feedback[4], dt)
                twist_output.twist.angular.z = self.pids[5].update(self.last_feedback[5], dt)
            else:
                rospy.logwarn("Odometry feedback is invalid, setting wrench to zero.")
                twist_output.twist.linear.x = 0
                twist_output.twist.linear.y = 0
                twist_output.twist.linear.z = 0
                twist_output.twist.angular.x = 0
                twist_output.twist.angular.y = 0
                twist_output.twist.angular.z = 0
            twist_output.header.stamp = rospy.Time.now()
            twist_output.header.frame_id = 'base_link'
            self.pub.publish(twist_output)

if __name__ == "__main__":
    rospy.init_node('cascade_pose_controller')
    try:
        frequency = rospy.get_param("~frequency", 10.0)
        rospy.loginfo('Starting cascade pose control with %f Hz.\n', frequency)
        node = PoseControllerNode(frequency)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
