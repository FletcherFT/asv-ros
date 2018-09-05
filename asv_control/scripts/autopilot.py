#!/usr/bin/python
import rospy
import dynamic_reconfigure.server
from asv_control.cfg import AutopilotConfig
from std_srvs.srv import SetBool
from pid import Pid
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from math import pi
import numpy as np
mul=np.matmul
import tf2_ros
from tf_conversions import transformations as transform
import tf

## TODO ##
## USE LOS GUIDANCE BETWEEN STATE AND SET POSITIONS (ATAN2) TO OBTAIN THE SET HEADING
## MAKE SURE THAT STATE ORIENTATION IS IN UTM FIRST!

class Autopilot():
    #Autopilot controls forward speed and heading of a robot.
    #All other degrees of freedom are uncontrolled.

    def __init__(self):
        # init the ros node
        rospy.init_node('autopilot')
        # setup the tf2/tf listeners for transforms
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf.TransformListener()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # get the desired control loop frequency
        frequency = rospy.get_param('~frequency',20)
        rospy.loginfo('Starting autopilot control with {} Hz.'.format(frequency))
        # get the desired cruising speed of the ASV
        self.U = rospy.get_param('~U',0.3)
        # init some pose messages
        self.pose_state = PoseStamped()
        self.pose_set = PoseStamped()
        # a timeout parameter for feedback from odometry/filtered (dont' want any out of date info)
        self.FEEDBACK_TIMEOUT = 1.0
        self.setpoint_valid = False
        self.feedback_received = False
        self.enabled = False
        self.last_feedback = Odometry()
        self.last_feedback_time = rospy.Time.now()
        self.enable_server = rospy.Service('~enable', SetBool, self.enable)
        self.pids = []
        for i in range(2):
            self.pids.append(Pid(0.0, 0.0, 0.0,integral_min=-0.1,integral_max=0.1,output_max=1.0))
        self.server = dynamic_reconfigure.server.Server(AutopilotConfig, self.reconfigure)
        self.pub = rospy.Publisher('tau_com/AP',WrenchStamped,queue_size=10)
        period = rospy.Duration.from_sec(1.0/frequency)
        self.timer = rospy.Timer(period,self.updateOutput)
        rospy.Subscriber('pose_com',PoseStamped,self.setpointCallback)
        rospy.Subscriber('odometry/filtered',Odometry,self.stateCallback)
        rospy.loginfo("""Listening for pose feedback to be published on {}...\n
        Waiting for setpoint to be published on {}...\n
        Enable/Disable controller via {}...""".format(rospy.resolve_name('odometry/filtered'),rospy.resolve_name('pose_com'),rospy.resolve_name('~enable')))    
        rospy.spin()

    def enable(self,request):
        """
        Handles ROS service requests for enabling/disabling control.
        Returns current enabled status and setpoint.
        """
        if request.data:
            if self.isFeedbackValid():
                self.enabled = True
                self.setpoint_valid = True
                response = [True,"Autopilot Enabled!"]
            else:
                rospy.logerr("Cannot enable pose control without valid feedback!")
                response = [False,"Cannot enable pose control without valid feedback!"]
        else:
            self.enabled = False
            response = [True,"Autopilot Disabled!"]
        return response
    
    def reconfigure(self,config,level):
        """
        Handles dynamic reconfigure requests.
        """
        rospy.loginfo("Reconfigure request...")
        self.pids[0].k_p = config['U_Kp']
        self.pids[0].k_i = config['U_Ki']
        self.pids[0].k_d = config['U_Kd']
        self.pids[1].k_p = config['psi_Kp']
        self.pids[1].k_i = config['psi_Ki']
        self.pids[1].k_d = config['psi_Kd']
        return config # Returns the updated configuration.
    
    def setpointCallback(self,setpoint):
        """
        Change the setpoint of the controller.
        """
        if not self.setpoint_valid:
            rospy.loginfo("First setpoint received.")
            self.setpoint_valid = True
        self.setSetpoint(setpoint)
        if not self.enabled:
            rospy.logwarn("Autopilot PIDs not enabled, please call 'rosservice call {} true'".format(rospy.resolve_name('enable')))
            return
        rospy.loginfo('{}: Changed setpoint to: {}'.format(rospy.get_name(),setpoint))

    def setSetpoint(self, pose):
        #these are the setpoints
        #setpoint 1: forward speed
        #setpoint 2: heading error
        self.pids[0].setSetpoint(self.U)
        self.pids[1].setSetpoint(0.0)
        #Need to ensure commanded pose is in UTM.
        if pose.header.frame_id != 'utm':
            if self.tfBuffer.can_transform(pose.header.frame_id,"utm",rospy.Time.now(),rospy.Duration.from_sec(0.5)):
                self.pose_set = self.tfListener.transformPose("utm",pose)
        else:
            self.pose_set = pose

    def stateCallback(self, odometry_msg):
        #Odometry.pose message comes in as an odom frame.
        #Need to convert to UTM for true North heading error.
        pose_in = PoseStamped()
        pose_in.header = odometry_msg.header
        pose_in.pose = odometry_msg.pose.pose
        if self.tfBuffer.can_transform(pose_in.header.frame_id,"utm",rospy.Time.now(),rospy.Duration.from_sec(0.5)):
            self.pose_state = self.tfListener.transformPose("utm",pose_in)
        else:
            rospy.logwarn("{}: No tf available at time requested.".format(rospy.get_name()))
            return
        #heading_error
        q_state = (self.pose_state.pose.orientation.x,
        self.pose_state.pose.orientation.y,
        self.pose_state.pose.orientation.z,
        self.pose_state.pose.orientation.w)
        eul_state = transform.euler_from_quaternion(q_state,'sxyz')
        q_set = (self.pose_set.pose.orientation.x,
        self.pose_set.pose.orientation.y,
        self.pose_set.pose.orientation.z,
        self.pose_set.pose.orientation.w)
        eul_set = transform.euler_from_quaternion(q_set,'sxyz')
        psi_err = eul_set[2]-eul_state[2]
        if abs(psi_err)>pi:
            if psi_err<0:
                psi_err+=2*pi
            else:
                psi_err-=2*pi
            self.pids[1].__integral=0.0
            self.pids[1].__derivative=0.0
            self.pids[1].__previouserror=0.0
        feedback = np.array([odometry_msg.twist.twist.linear.x,psi_err])
        self.last_feedback = feedback
        self.last_feedback_time = odometry_msg.header.stamp

    def isFeedbackValid(self):
        feedback_in_time = (rospy.Time.now() - 
            self.last_feedback_time).to_sec() < self.FEEDBACK_TIMEOUT
        return feedback_in_time

    def updateOutput(self, event):
        if self.setpoint_valid and self.enabled:
            wrench_output = WrenchStamped()
            if self.isFeedbackValid():
                dt = (event.current_real - event.last_real).to_sec()
                wrench_output.wrench.force.x = self.pids[0].update(self.last_feedback[0], dt)
                wrench_output.wrench.force.y = 0
                wrench_output.wrench.force.z = 0
                wrench_output.wrench.torque.x = 0
                wrench_output.wrench.torque.y = 0
                wrench_output.wrench.torque.z = self.pids[1].update(self.last_feedback[1], dt)
            else:
                rospy.logwarn("Odometry feedback is too old, setting wrench to zero.")
                wrench_output.wrench.force.x = 0
                wrench_output.wrench.force.y = 0
                wrench_output.wrench.force.z = 0
                wrench_output.wrench.torque.x = 0
                wrench_output.wrench.torque.y = 0
                wrench_output.wrench.torque.z = 0
            wrench_output.header.stamp = rospy.Time.now()
            wrench_output.header.frame_id = 'AP'
            self.pub.publish(wrench_output)

if __name__ == "__main__":
    try:
        Autopilot()
    except rospy.ROSInterruptException:
        pass
