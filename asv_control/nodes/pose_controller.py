#!/usr/bin/python
import rospy
import dynamic_reconfigure.server
import asv_control_msgs.srv
from asv_control.cfg import PoseControllerConfig
from tf import transformations as transform
from pid import Pid
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from math import pi

#TODO:  1.  Listen for /odometry and /move_base_simple/goal
    #       2.  Calculate dX, dY, dZ
    #       3.  Use orientation to transform dX, dY, dZ to dx, dy, dz
    #       4.  Feed dx, dy, dz as state input to surge, sway, heave, controllers
    #       5.  Setpoints for surge, sway, heave controllers is 0
    #       6.  State inputs for roll, pitch, yaw controllers are /odometry
    #       7.  Setpoints for roll, pitch, yaw controllers are from /move_base_simple/goal

class PoseControllerNode():
    """
    Node for controlling the pose (position and orientation) of a robot,
    using a simple PID scheme. The input consists in a setpoint (required 
    position) given as geometry_msgs/PoseStamped on topic 'pose_request' 
    and of current odometry readings as nav_msgs/Odometry on topic 'odometry'.
    Every degree of freedom has a separate PID. The pose values given in
    the incoming odometry message are used as feedback for the PIDs.
    Output is a message of type geometry_msgs/WrenchStamped on topic 
    'wrench' containing force and torque values that are
    necessary to maintain the position..
    """
    def __init__(self, frequency):
        self.setpoints = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.FEEDBACK_TIMEOUT = 1.0
        self.setpoint_valid = False
        self.feedback_received = False
        self.enabled = False
        self.last_feedback = Odometry()
        self.enable_server = rospy.Service('~enable', asv_control_msgs.srv.EnableControl, self.enable)
        self.pids = []
        for i in range(6):
            self.pids.append(Pid(0.0, 0.0, 0.0))
        self.server = dynamic_reconfigure.server.Server(PoseControllerConfig, self.reconfigure)
        
        self.pub = rospy.Publisher('~wrench_output', WrenchStamped,queue_size=1)
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
        rospy.loginfo('Changed setpoint to: %s', setpoint.pose)

    def setSetpoint(self, pose):
        self.pids[0].setSetpoint(pose.position.x)
        self.pids[1].setSetpoint(pose.position.y)
        self.pids[2].setSetpoint(pose.position.z)
        quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
        euler = transform.euler_from_quaternion(quaternion)

        #self.pids[3].setSetpoint(euler[0])
        #self.pids[4].setSetpoint(euler[1])
        #self.pids[5].setSetpoint(euler[2])
        self.pids[3].setSetpoint(0)
        self.pids[4].setSetpoint(0)
        self.pids[5].setSetpoint(0)
        self.setpoints[5]=euler[2]

    def odometryCallback(self, odometry_msg):
        self.last_feedback = odometry_msg
        
    def isFeedbackValid(self):
        feedback_in_time = (rospy.Time.now() - 
            self.last_feedback.header.stamp).to_sec() < self.FEEDBACK_TIMEOUT
        return feedback_in_time 
        
    def updateOutput(self, event):
       if self.setpoint_valid and self.enabled:
           wrench_output = WrenchStamped()
           if self.isFeedbackValid():
               dt = (event.current_real - event.last_real).to_sec()
               wrench_output.wrench.force.x = self.pids[0].update(self.last_feedback.pose.pose.position.x, dt)
               wrench_output.wrench.force.y = self.pids[1].update(self.last_feedback.pose.pose.position.y, dt)
               wrench_output.wrench.force.z = self.pids[2].update(self.last_feedback.pose.pose.position.z, dt)
               quaternion = (
               self.last_feedback.pose.pose.orientation.x,
               self.last_feedback.pose.pose.orientation.y,
               self.last_feedback.pose.pose.orientation.z,
               self.last_feedback.pose.pose.orientation.w)
               euler = transform.euler_from_quaternion(quaternion)
               wrench_output.wrench.torque.x = self.pids[3].update(euler[0], dt)
               wrench_output.wrench.torque.y = self.pids[4].update(euler[1], dt)
               error_z = -(self.setpoints[5] - euler[2])
               if abs(error_z)>pi:
                    error_z = -error_z
               wrench_output.wrench.torque.z = self.pids[5].update(error_z, dt)
           else:
               rospy.logwarn("Odometry feedback is invalid, setting wrench to zero.")
           wrench_output.header.stamp = rospy.Time.now()
           wrench_output.header.frame_id = 'base_link'
           self.pub.publish(wrench_output)

if __name__ == "__main__":
    rospy.init_node('pose_controller')
    try:
        frequency = rospy.get_param("~frequency", 10.0)
        rospy.loginfo('Starting pose control with %f Hz.\n', frequency)
        node = PoseControllerNode(frequency)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

