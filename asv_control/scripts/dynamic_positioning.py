#!/usr/bin/python
import rospy
import dynamic_reconfigure.server
from asv_control.cfg import PoseControllerConfig
from std_srvs.srv import SetBool
import tf2_ros
from tf_conversions import transformations as transform
import tf
from controllers.pid import Pid
from geometry_msgs.msg import PoseStamped, Quaternion, WrenchStamped
from nav_msgs.msg import Odometry
from math import pi, sin, cos
import numpy as np
mul=np.matmul

class PoseControllerNode():
    #Node for controlling the pose (position and orientation) of a robot,
    #using a simple PID scheme. The input consists in a setpoint (required 
    #position) given as geometry_msgs/PoseStamped on topic '~pose_com' 
    #and of current odometry readings as nav_msgs/Odometry on topic '~state'.
    #Every degree of freedom has a separate PID. The pose values given in
    #the incoming odometry message are used as feedback for the PIDs.
    #Output is a message of type geometry_msgs/WrenchStamped on topic 
    #'~tau_com' containing force and torque values that are
    #necessary to maintain the position..

    def __init__(self, frequency):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf.TransformListener()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.P = [0.0,0.0,0.0]
        self.FEEDBACK_TIMEOUT = 1.0
        self.setpoint_valid = False
        self.feedback_received = False
        self.enabled = False
        self.last_feedback = Odometry()
        self.last_feedback_time = rospy.Time.now()
        self.enable_server = rospy.Service('~enable', SetBool, self.enable)

        self.pids = []
        for i in range(3):
            self.pids.append(Pid(0.0, 0.0, 0.0,integral_min=-0.1,integral_max=0.1))
            self.pids[i].setSetpoint(0.0)

        self.server = dynamic_reconfigure.server.Server(PoseControllerConfig, self.reconfigure)
        
        self.pub = rospy.Publisher('tau_com/DP', WrenchStamped,queue_size=1)
        rospy.Subscriber('pose_com', PoseStamped, self.setpointCallback)
        
        period = rospy.rostime.Duration.from_sec(1.0/frequency)
        self.timer = rospy.Timer(period, self.updateOutput)
        
        #rospy.Subscriber('odometry/filtered', Odometry, self.odometryCallback)
        rospy.loginfo('Waiting for setpoint to be published on '
                      '%s...', rospy.resolve_name('pose_com')) 

    def enable(self, request):
        """
        Handles ROS service requests for enabling/disabling control.
        Returns current enabled status and setpoint.
        """
        if request.data:
            if self.setpoint_valid:
                self.enabled = True
                self.setpoint_valid = True
                return [True,"DP Controller Enabled"]
            else:
                rospy.logwarn("No valid setpoint received yet.")
                return [False,"No valid setpoint received yet."]
        else:
            self.enabled = False
            return [False,"DP Controller Disabled"]

    def reconfigure(self, config, level):
        """
        Handles dynamic reconfigure requests.
        """
        rospy.loginfo("Reconfigure request...")
        self.pids[0].k_p = config['surge_Kp']
        self.pids[0].k_i = config['surge_Ki']
        self.pids[0].k_d = config['surge_Kd']
        self.pids[0].output_max = config['surge_max']
        self.pids[1].k_p = config['sway_Kp']
        self.pids[1].k_i = config['sway_Ki']
        self.pids[1].k_d = config['sway_Kd']
        self.pids[1].output_max = config['sway_max']
        self.pids[2].k_p = config['yaw_Kp']
        self.pids[2].k_i = config['yaw_Ki']
        self.pids[2].k_d = config['yaw_Kd']
        self.pids[2].output_max = config['yaw_max']
        return config # Returns the updated configuration.
    
    def setpointCallback(self,setpoint):
        """
        Change the setpoint of the controller.
        """
        if not self.setpoint_valid:
            rospy.loginfo("First setpoint received.")
            self.setpoint_valid = True
        self.set_pose = setpoint
        if not self.enabled:
            rospy.logwarn("{}: PIDs not enabled, please call 'rosservice call {} true'".format(rospy.get_name(),rospy.resolve_name('~enable')))
        rospy.loginfo('{}: Changed setpoint to: {}'.format(rospy.get_name(), setpoint.pose))
        
    def isFeedbackValid(self):
        feedback_in_time = (rospy.Time.now() - 
            self.tfListener.getLatestCommonTime(self.set_pose.header.frame_id,'base_link')) < self.FEEDBACK_TIMEOUT
        return feedback_in_time 
        
    def updateOutput(self, event):
        if self.setpoint_valid and self.enabled:
            if self.set_pose.header.frame_id != 'base_link':
                self.set_pose.header.stamp = rospy.Time.now()
                try:
                    self.tfListener.waitForTransform(self.set_pose.header.frame_id,"base_link",rospy.Time.now(),rospy.Duration(0.5))
                    error_vector = self.tfListener.transformPose("base_link",self.set_pose)
                except:
                    rospy.logwarn("No TF, setting feedback to all zeros")
                    error_vector = self.set_pose
                    error_vector.pose.position.x=0
                    error_vector.pose.position.y=0
                    error_vector.pose.orientation = Quaternion(0,0,0,1)
            else:
                error_vector = self.set_pose
            quat = (error_vector.pose.orientation.x,
                    error_vector.pose.orientation.y,
                    error_vector.pose.orientation.z,
                    error_vector.pose.orientation.w)
            eul = transform.euler_from_quaternion(quat)
            rospy.logdebug(error_vector)
            self.last_feedback = [-error_vector.pose.position.x,-error_vector.pose.position.y,-eul[2]]

            wrench_output = WrenchStamped()
            dt = (event.current_real - event.last_real).to_sec()
            wrench_output.wrench.force.x = self.pids[0].update(self.last_feedback[0], dt)
            wrench_output.wrench.force.y = self.pids[1].update(self.last_feedback[1], dt)
            wrench_output.wrench.torque.z = self.pids[2].update(self.last_feedback[2], dt)
            wrench_output.header.stamp = rospy.Time.now()
            wrench_output.header.frame_id = 'DP'
            self.pub.publish(wrench_output)

if __name__ == "__main__":
    rospy.init_node('dynamic_position')
    try:
        frequency = rospy.get_param("~frequency", 20.0)
        rospy.loginfo('Starting dynamic pose control with %f Hz.\n', frequency)
        node = PoseControllerNode(frequency)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
