#!/usr/bin/env python
import rospy
import numpy as np
from geodesy import utm
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import tf2_ros
import tf_conversions

def pose_cb(gps_msg,pub):
    lat = gps_msg.latitude
    lon = gps_msg.longitude
    alt = gps_msg.altitude

    utmPoint = utm.fromLatLong(lat,lon,alt).toPoint()

    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.stamp = gps_msg.header.stamp
    pose_msg.header.frame_id = 'utm'
    pose_msg.pose.pose.position = utmPoint
    cov = np.zeros(36)
    cov[0]=gps_msg.position_covariance[0]
    cov[7]=gps_msg.position_covariance[4]
    cov[14]=gps_msg.position_covariance[8]
    pose_msg.pose.covariance = cov
    pub.publish(pose_msg) 

def main():
    rospy.init_node("utm_pose_node")
    datum = rospy.get_param("~datum",False)
    mag_dec = rospy.get_param('~magnetic_declination_radians',0.0)
    yaw_offset = rospy.get_param('~yaw_offset',0.0)
    rotation_offset = yaw_offset+mag_dec
    rospy.loginfo("Magnetic Declination is {}\n IMU Reference Offset is:{}\nTotal Yaw Offset for UTM->World is {}".format(mag_dec,yaw_offset,rotation_offset))
    br = tf2_ros.StaticTransformBroadcaster()
    T = TransformStamped()
    if datum:
        rospy.loginfo("Setting datum to Latitude: {}, Longitude: {}".format(datum[0],datum[1]))
        datumPoint = utm.fromLatLong(datum[0],datum[1])
        rospy.loginfo(datumPoint)
        rospy.loginfo("UTM Zone: {}".format(datumPoint.gridZone()))
        datum = datumPoint.toPoint()
        rospy.loginfo("UTM Translation is: (X,Y,Z)=({},{},{})".format(datum.x,datum.y,datum.z))
        T.transform.translation.x = datum.x
        T.transform.translation.y = datum.y
        T.transform.translation.z = datum.z
    else:
        rospy.loginfo("Setting datum to current zone origin.")
    T.header.frame_id="utm"
    T.child_frame_id = "odom"
    quat=tf_conversions.transformations.quaternion_from_euler(rotation_offset,0,0,axes='sxyz')
    T.transform.rotation.x = quat[0]
    T.transform.rotation.y = quat[1]
    T.transform.rotation.z = quat[2]
    T.transform.rotation.w = quat[3]
    br.sendTransform(T)

    pub = rospy.Publisher("pose/gps",PoseWithCovarianceStamped,queue_size=10)

    rospy.Subscriber("gps/fix",NavSatFix,pose_cb,pub)
    rospy.spin()

if __name__ == '__main__':
    main()