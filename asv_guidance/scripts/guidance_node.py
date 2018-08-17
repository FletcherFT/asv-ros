#!/usr/bin/env python
import rospy
import numpy as np
from geodesy import utm
from geometry_msgs.msg import TransformStamped, PoseStamped, Vector3
from geographic_msgs.msg import GeoPointStamped
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
        self.marker_pub = rospy.Publisher("markers",Marker,queue_size=10)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.tfBufferInterface = tf2_ros.BufferInterface()
        #self.tfTransformer = tf.TransformerROS()
        self.tfListener = tf.TransformListener()
        rospy.Subscriber("guidance/operator",PoseStamped,self.override_cb)
        rospy.Subscriber("guidance/task",GeoPointStamped,self.task_cb)
        rospy.spin()

    def override_cb(self,msg):
        if self.tfBuffer.can_transform(msg.header.frame_id,"odom",rospy.Time.now(),rospy.Duration.from_sec(0.5)):
            #tfmsg = tf2_ros.Stamped(msg,msg.header.stamp,msg.header.frame_id)
            #test = self.tfBufferInterface.transform(tfmsg,"odom")
            setpoint_msg = self.tfListener.transformPose("odom",msg) #TF2 FOR KINETIC JUST AIN'T WORKING
            #try:
            #    trans = self.tfBuffer.lookup_transform(msg.header.frame_id,"odom",rospy.Time.now())
            #except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            #    rospy.logerr(e)
            #    return
            #setpoint_msg = tf2_geometry_msgs.do_transform_pose(msg,trans)
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
            self.set_point_pub.publish(setpoint_msg)
        else:
            pass

    def task_cb(self,msg):
        pass

if __name__ == '__main__':
    guidance()