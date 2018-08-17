#!/usr/bin/env python
import rospy
import numpy as np
from geodesy import utm
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
from tf_conversions import transformations as tf
import message_filters
from scipy.linalg import block_diag

def qv_mult(q1, v1):
    #v1 = tf.unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    return tf.quaternion_multiply(
        tf.quaternion_multiply(q1, q2), 
        tf.quaternion_conjugate(q1)
    )[:3]

class utm_odom:
    def __init__(self):
        rospy.init_node("utm_odom")
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        datum = rospy.get_param("~datum",False)
        mag_dec = rospy.get_param('~magnetic_declination_radians',0.0)
        yaw_offset = rospy.get_param('~yaw_offset',0.0)
        self.do_tf = rospy.get_param('~broadcast_odom_base',False)
        pub_gps = rospy.get_param('~pub_gps',False)
        rotation_offset = yaw_offset+mag_dec
        rospy.loginfo("Magnetic Declination is {}\n IMU Reference Offset is:{}\nTotal Yaw Offset for UTM->Odom is {}".format(mag_dec,yaw_offset,rotation_offset))
        self.odom_tf = tf2_ros.TransformBroadcaster()
        # Transform odom into UTM frame
        if datum:
            rospy.loginfo("Setting datum to Latitude: {}, Longitude: {}".format(datum[0],datum[1]))
            datumPoint = utm.fromLatLong(datum[0],datum[1],0.0) # get the 
            (zone,band)= datumPoint.gridZone()
            rospy.loginfo("UTM Zone: {}{}".format(zone,band))
            self.datum = datumPoint.toPoint()
            rospy.loginfo("UTM Translation is: (X,Y,Z)=({},{},{})".format(self.datum.x,self.datum.y,self.datum.z))
        else:
            self.datum= utm.UTMPoint(easting=0.0, northing=0.0, altitude=0.0, zone=zone, band=band).toPoint()
            rospy.logwarn("Setting datum to current zone origin. odom frame will have large values!")
        T = TransformStamped()
        T.header.frame_id="utm"
        T.header.stamp = rospy.Time.now()
        T.child_frame_id = "odom"
        quat=tf.quaternion_from_euler(0,0,rotation_offset,axes='sxyz')
        T.transform.rotation.x = quat[0]
        T.transform.rotation.y = quat[1]
        T.transform.rotation.z = quat[2]
        T.transform.rotation.w = quat[3]
        T.transform.translation.x = self.datum.x # Northing
        T.transform.translation.y = self.datum.y # Easting
        T.transform.translation.z = self.datum.z # Altitude
        br = tf2_ros.StaticTransformBroadcaster()
        br.sendTransform(T)
        self.pub = rospy.Publisher("odometry/gps",Odometry,queue_size=10)
        imu_sub = message_filters.Subscriber('imu/data',Imu)
        nav_sub = message_filters.Subscriber("gps/fix",NavSatFix)
        ts = message_filters.ApproximateTimeSynchronizer([imu_sub, nav_sub], 10, 0.25)
        ts.registerCallback(self.publish)
        if pub_gps:
            filt_sub = rospy.Subscriber("odometry/filtered",Odometry,self.filt_cb)
        rospy.spin()

    def filt_cb(self,msg):
        #self.tfBuffer.lookup_transform(msg.header.frame_id,"utm")
        pass

    def publish(self,imu_msg,nav_msg):
        # GET THE POSITION OF THE GPS IN UTM FRAME
        lat = nav_msg.latitude
        lon = nav_msg.longitude
        alt = nav_msg.altitude
        utmPoint = utm.fromLatLong(lat,lon,alt).toPoint()
        # CONVERT GPS IN UTM FRAME TO ODOM FRAME
        x = utmPoint.x-self.datum.x
        y = utmPoint.y-self.datum.y
        z = utmPoint.z-self.datum.z
        # GET THE ROTATION OF THE IMU FROM ODOM FRAME TO UTM FRAME
        q_imu=(imu_msg.orientation.x,imu_msg.orientation.y,imu_msg.orientation.z,imu_msg.orientation.w)
        # GET THE ROTATION/TRANSLATION FROM IMU/NAV TO BASE_LINK (BODY->BODY)
        try: 
            imu_trans = self.tfBuffer.lookup_transform(imu_msg.header.frame_id,"base_link", imu_msg.header.stamp)
            nav_trans = self.tfBuffer.lookup_transform(nav_msg.header.frame_id,"base_link", nav_msg.header.stamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)
            return
        # FIRST, ROTATE IMU_LINK TO BASE_LINK
        q_rot = (imu_trans.transform.rotation.x,imu_trans.transform.rotation.y,imu_trans.transform.rotation.z,imu_trans.transform.rotation.w)
        #
        q_base = tf.quaternion_multiply(q_rot, q_imu)
        # NEXT, ROTATE GPS_LINK TO BASE_LINK
        v_nav = qv_mult(q_base,[nav_trans.transform.translation.x,nav_trans.transform.translation.y,nav_trans.transform.translation.z])
        # FINALLY, APPLY TRANSLATION FROM GPS_LINK TO BASE_LINK        
        odom_msg = Odometry()
        odom_msg.header.frame_id="odom"
        odom_msg.child_frame_id="base_link"
        odom_msg.pose.pose.position.x=v_nav[0]+x
        odom_msg.pose.pose.position.y=v_nav[1]+y
        odom_msg.pose.pose.position.z=v_nav[2]+z
        odom_msg.pose.pose.orientation.x=q_base[0]
        odom_msg.pose.pose.orientation.y=q_base[1]
        odom_msg.pose.pose.orientation.z=q_base[2]
        odom_msg.pose.pose.orientation.w=q_base[3]
        nav_cov = np.reshape(nav_msg.position_covariance,[3,3])
        imu_cov = np.reshape(imu_msg.orientation_covariance,[3,3])
        cov = block_diag(nav_cov,imu_cov).flatten()
        odom_msg.pose.covariance=cov.tolist()
        odom_msg.header.stamp=rospy.Time.now()
        self.pub.publish(odom_msg)
        if self.do_tf:
            T = TransformStamped()
            T.header = odom_msg.header
            T.child_frame_id = odom_msg.child_frame_id
            T.transform.translation.x=odom_msg.pose.pose.position.x
            T.transform.translation.y=odom_msg.pose.pose.position.y
            T.transform.translation.z=odom_msg.pose.pose.position.z
            T.transform.rotation = odom_msg.pose.pose.orientation
            self.odom_tf.sendTransform(T)

if __name__ == '__main__':
    utm_odom()