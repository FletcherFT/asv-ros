#!/usr/bin/env python
import rospy
import tf
transform = tf.transformations
import numpy as np
from sensor_msgs.msg import Imu

class ImuDriverNode():
    def __init__(self):
        rospy.Subscriber('imu/data',Imu,self.callback)
        self.world_imu_broadcaster = tf.TransformBroadcaster()

    def callback(self,data):
        quaternion = (
                data.orientation.x,
                data.orientation.y,
                data.orientation.z,
                data.orientation.w)
        self.world_imu_broadcaster.sendTransform((0,0,0),quaternion,rospy.Time.now(),'imu_link','map')


if __name__=='__main__':
    try:
        rospy.init_node('imu_tf')
        h = ImuDriverNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
