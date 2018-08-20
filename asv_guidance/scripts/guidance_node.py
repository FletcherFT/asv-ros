#!/usr/bin/env python
import rospy
import numpy as np
#from geodesy import utm
from pygeodesy import utm

from geometry_msgs.msg import TransformStamped, PoseStamped, Vector3
from geographic_msgs.msg import GeoPoseStamped

from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

import tf2_ros
import tf_conversions
import tf2_geometry_msgs
import tf

class guidance:
    def __init__(self):
        rospy.init_node("guidance")
        # Publishers
        self.marker_pub = rospy.Publisher("markers",Marker,queue_size=10)
        self.setpoint_pub = rospy.Publisher("guidance/setpoint",PoseStamped,queue_size=10)
        # TF listeners
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf.TransformListener()
        # Flow Control
        self.manual = True #switch for automated or manual override guidance
        # msg containers
        self.override = PoseStamped()
        self.automate = PoseStamped()

        self.resume = rospy.Service('guidance/resume', Trigger, self.handle_resume)
        # subscribers
        rospy.Subscriber("guidance/operator",PoseStamped,self.override_cb)
        rospy.Subscriber("guidance/task",GeoPoseStamped,self.task_cb)
        # fidget spinner
        rospy.spin()

    def handle_resume(self,req):
        res = Trigger()
        if self.manual:
            self.manual=False
            res.success=True
            res.message="Waypoint override disabled, resuming current waypoint."
        else:
            res.success=False
            res.message="Waypoint override already disabled!"
        return res


    def override_cb(self,msg):
        self.manual = True
        if self.tfBuffer.can_transform(msg.header.frame_id,"odom",rospy.Time.now(),rospy.Duration.from_sec(2.0)):
            rospy.loginfo("{}: New override waypoint at ({},{},{})".format(rospy.get_name(),msg.pose.pose.position.x,msg.pose.pose.position.y,msg.header.frame_id))
            self.override = self.tfListener.transformPose("odom",msg) #TF2 FOR KINETIC JUST AIN'T WORKING
            self.update_goal(self.override)
            self.setpoint_pub.publish(self.override)
        else:
            rospy.logwarn("{}: No TF between {} and odom!".format(rospy.get_name(),msg.header.frame_id))

    def task_cb(self,msg):
        # in: GeoPoseStamped message
        # CONVERT LAT, LONG TO UTM
        WGS84 = msg.position
        UTM = utm.toUtm(WGS84.latitude,WGS84.longitude)
        self.automate = PoseStamped()
        self.automate.header.frame_id="utm"
        self.automate.header.stamp=msg.header.stamp
        self.automate.pose.position.x=UTM.easting
        self.automate.pose.position.y=UTM.northing
        self.automate.pose.orientation=msg.orientation
        if not self.manual:
            self.setpoint_pub.publish(self.automate)
            self.update_goal(self.automate)
        else:
            rospy.logwarn("{}: Manual waypoint override in effect, call guidance/resume service to resume.")

    def update_goal(self,setpoint_msg):
        marker_msg = Marker()
        marker_msg.header = setpoint_msg.header
        marker_msg.ns = "guidance"
        marker_msg.id = 0
        marker_msg.type = 2
        marker_msg.action = 0
        marker_msg.pose = setpoint_msg.pose
        marker_msg.scale = Vector3(5,5,5)
        marker_msg.color = ColorRGBA(0.77432878,  0.31884126,  0.54658502,1.0) #HOT PINK
        self.marker_pub.publish(marker_msg)

if __name__ == '__main__':
    guidance()