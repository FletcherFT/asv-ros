#!/usr/bin/python
import rospy
import dynamic_reconfigure.server
from asv_control.cfg import AutopilotConfig
from std_srvs.srv import SetBool
from controllers.pid import Pid
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from asv_messages import Float64Stamped
from math import pi, atan2
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
        self.U = rospy.get_param('~U',0.7)
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
            self.pids.append(Pid(0.0, 0.0, 0.0,integral_min=-2.0,integral_max=2.0))
        self.server = dynamic_reconfigure.server.Server(AutopilotConfig, self.reconfigure)
        self.pub = rospy.Publisher('tau_com/AP',WrenchStamped,queue_size=10)
        self.u_pub = rospy.Publisher('u_error',Float64Stamped,queue_size=10)
        self.psi_pub = rospy.Publisher('psi_error',Float64Stamped,queue_size=10)
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
        self.pids[0].k_p = config['surge_Kp']
        self.pids[0].k_i = config['surge_Ki']
        self.pids[0].k_d = config['surge_Kd']
        self.pids[0].output_max = config['surge_max']
        self.pids[1].k_p = config['yaw_Kp']
        self.pids[1].k_i = config['yaw_Ki']
        self.pids[1].k_d = config['yaw_Kd']
        self.pids[1].output_max = config['yaw_max']
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
        dY = self.pose_set.pose.position.y-self.pose_state.pose.position.y
        dX = self.pose_set.pose.position.x-self.pose_state.pose.position.x
        psi_set = atan2(dY,dX)
        psi_err = -(psi_set-eul_state[2])
        # if the heading error is greater than pi/5 and the speed setpoint is U, set to 0.0.
        if abs(psi_err) > pi/5 and abs(self.pids[0].getSetpoint())>0:
            self.pids[0].setSetpoint(0.0)
        elif abs(psi_err) < pi/5 and abs(self.pids[0].getSetpoint())==0:
            self.pids[0].setSetpoint(self.U)
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
            u_output = Float64Stamped()
            psi_output = Float64Stamped()
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
            u_output.header.stamp = rospy.Time.now()
            u_output.header.frame_id = 'AP'
            psi_output.header.stamp = rospy.Time.now()
            psi_output.header.frame_id = 'AP'
            u_output.data = self.pids[0].__error
            psi_output.data = self.pids[1].__error
            self.u_pub.publish(u_output)
            self.psi_pub.publish(psi_output)

if __name__ == "__main__":
    try:
        Autopilot()
    except rospy.ROSInterruptException:
        pass
