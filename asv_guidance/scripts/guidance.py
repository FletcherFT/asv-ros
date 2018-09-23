#!/usr/bin/env python
import rospy
import numpy as np
from pygeodesy import utm

from geometry_msgs.msg import TransformStamped, PoseStamped, Vector3
from geographic_msgs.msg import GeoPoseStamped
from topic_tools.srv import MuxSelect

from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from asv_messages.srv import UTMService, UTMServiceResponse
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
import tf_conversions
import tf

#TODO ADD IN AUTPILOT AND DYNAMIC POSITIONING MODE CONTROL

class guidance:
    def __init__(self):
        rospy.init_node("guidance")

        # acceptance errors
        self.distance_error_acceptance = rospy.get_param("~distance_error_acceptance",2.5) # within 2.5 m
        self.yaw_error_acceptance = rospy.get_param("~yaw_error_acceptance",0.0872665) # five degrees

        # Publishers
        self.marker_pub = rospy.Publisher("guidance/current",MarkerArray,queue_size=10)
        self.setpoint_pub = rospy.Publisher("guidance/setpoint",PoseStamped,queue_size=10)

        # TF listeners
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.tfListener = tf.TransformListener()

        # Flow Control
        self.manual = True #switch for automated or manual override guidance
        self.notified = False #switch for notifying the operator/supervisor when the task is reached
        self.hp = False

        # msg containers
        self.override = PoseStamped()
        self.automate = PoseStamped()

        self.northing = None
        self.easting = None
        self.latitude = None
        self.longitude = None
        self.pose_meas = None

        self.resume = rospy.Service('guidance/resume', Trigger, self.handle_resume)
        self.utmrequest = rospy.Service('guidance/utmrequest', UTMService, self.handleUTMRequest)
        # subscribers
        rospy.Subscriber("guidance/operator",PoseStamped,self.override_cb)
        rospy.Subscriber("guidance/task",GeoPoseStamped,self.task_cb)
        rospy.Subscriber("odometry/filtered",Odometry,self.measure_cb)
        # fidget spinner
        rospy.spin()

    def handleUTMRequest(self,req):
        self.odom2utm()
        response = UTMServiceResponse()
        if not self.northing is None and not self.easting is None:
            response.easting = self.easting
            response.northing = self.northing
            response.latitude = self.latitude
            response.longitude = self.longitude
            response.success = True
            return response 
        else:
            response.easting = 0
            response.northing = 0
            response.latitude = 0
            response.longitude = 0
            response.success = False
            return [False,0,0]

    def handle_resume(self,req):
        if self.manual:
            self.manual=False
            res = [True,"Waypoint override disabled, resuming current waypoint."]
            self.update_goal(self.automate)
        else:
            res = [False,"Waypoint override already disabled!"]
        return res

    def request_new(self):
        # Service to call for new way point
        try:
            rospy.wait_for_service('supervisor/request_new',5.0)
        except:
            rospy.logerr("No supervisor/request_new service available.")
            self.notified=False
            return
        new_waypoint_req = rospy.ServiceProxy('supervisor/request_new', Trigger)
        try:
          resp = new_waypoint_req()
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: {}".format(str(exc)))
            self.notified=False

    def override_cb(self,msg):
        self.manual = True
        self.notified = False
        if self.tfBuffer.can_transform(msg.header.frame_id,"odom",rospy.Time.now(),rospy.Duration.from_sec(5)):
            rospy.loginfo("{}: New override waypoint at ({},{},{})".format(rospy.get_name(),msg.pose.position.x,msg.pose.position.y,msg.header.frame_id))
            self.override = self.tfListener.transformPose("odom",msg) #TF2 FOR KINETIC JUST AIN'T WORKING
            self.update_goal(self.override)
            self.setpoint_pub.publish(self.override)
        else:
            rospy.logerr("{}: No TF between {} and odom!".format(rospy.get_name(),msg.header.frame_id))

    def task_cb(self,msg):
        self.notified = False
        # in: GeoPoseStamped message
        # CONVERT LAT, LONG TO UTM
        WGS84 = msg.pose.position
        # check to see if GeoPoseStamped is for hold position or waypoint following..
        self.hp = WGS84.altitude>=0
        UTM = utm.toUtm(WGS84.latitude,WGS84.longitude)
        self.automate = PoseStamped()
        self.automate.header.frame_id="utm"
        self.automate.header.stamp=msg.header.stamp
        self.automate.pose.position.x=UTM.easting
        self.automate.pose.position.y=UTM.northing
        self.automate.pose.orientation=msg.pose.orientation
        if self.tfBuffer.can_transform(self.automate.header.frame_id,"odom",rospy.Time.now(),rospy.Duration.from_sec(5)):
            self.automate = self.tfListener.transformPose("odom",self.automate) #TF2 FOR KINETIC JUST AIN'T WORKING
        else:
            rospy.logerr("{}: No TF between {} and odom!".format(rospy.get_name(),msg.header.frame_id))
            return
        if not self.manual:
            rospy.loginfo("{}: New waypoint at ({},{},{}))".format(rospy.get_name(),self.automate.pose.position.x,self.automate.pose.position.y,self.automate.header.frame_id))
            self.setpoint_pub.publish(self.automate)
            self.update_goal(self.automate)
        else:
            rospy.logwarn("{}: Manual waypoint override in effect, call guidance/resume service to resume.".format(rospy.get_name()))

    def odom2utm(self):
        if self.pose_meas is None:
            return None
        pose_msg = PoseStamped()
        pose_msg.header.stamp=rospy.Time.now()-rospy.Duration(0.05)
        pose_msg.header.frame_id='odom'
        pose_msg.pose = self.pose_meas
        if self.tfBuffer.can_transform(pose_msg.header.frame_id,"utm",rospy.Time.now(),rospy.Duration.from_sec(5)):
            utm_msg = self.tfListener.transformPose("utm",pose_msg) #TF2 FOR KINETIC JUST AIN'T WORKING
            self.northing = utm_msg.pose.position.y
            self.easting = utm_msg.pose.position.x
            UTMpoint = utm.Utm(55,'S',self.easting,self.northing)
            (self.latitude, self.longitude, _, _, _) = UTMpoint.toLatLon(None)
        else:
            rospy.logerr("{}: No TF between {} and utm!".format(rospy.get_name(),pose_msg.header.frame_id))
            return None

    def update_goal(self,setpoint_msg):
        array_msg = MarkerArray()
        array_msg.markers = []
        marker_msg = Marker()
        marker_msg.header = setpoint_msg.header
        marker_msg.ns = "guidance/current"
        marker_msg.id = 0
        marker_msg.type = 2
        marker_msg.action = 0
        marker_msg.pose = setpoint_msg.pose
        marker_msg.scale = Vector3(2*self.distance_error_acceptance,2*self.distance_error_acceptance,2*self.distance_error_acceptance)
        marker_msg.color = ColorRGBA(0.77432878,  0.31884126,  0.54658502,1.0) #HOT PINK
        array_msg.markers.append(marker_msg)
        marker_msg = Marker()
        marker_msg.header = setpoint_msg.header
        marker_msg.ns = "guidance/current"
        marker_msg.id = 1
        marker_msg.type = 0
        marker_msg.action = 0
        marker_msg.pose = setpoint_msg.pose
        marker_msg.scale = Vector3(20,2,2)
        marker_msg.color = ColorRGBA(0.77432878,  0.31884126,  0.54658502,1.0) #HOT PINK
        array_msg.markers.append(marker_msg)
        self.marker_pub.publish(array_msg)

    def measure_cb(self,msg):
        self.pose_meas = msg.pose.pose
        pose_meas = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw_meas = tf_conversions.transformations.euler_from_quaternion((orientation.x,orientation.y,orientation.z,orientation.w))[2]

        if self.manual:
            pose_set = self.override.pose.position
            orientation = self.override.pose.orientation
            yaw_set = tf_conversions.transformations.euler_from_quaternion((orientation.x,orientation.y,orientation.z,orientation.w))[2]
        else:
            pose_set = self.automate.pose.position
            orientation = self.automate.pose.orientation
            yaw_set = tf_conversions.transformations.euler_from_quaternion((orientation.x,orientation.y,orientation.z,orientation.w))[2]

        distance_error_measured = np.linalg.norm([pose_set.x-pose_meas.x,pose_set.y-pose_meas.y,pose_set.z-pose_meas.z])
        yaw_error_measured = yaw_set-yaw_meas
        if yaw_error_measured > np.pi:
            yaw_error_measured-=2*np.pi
        elif yaw_error_measured < -np.pi:
            yaw_error_measured+=2*np.pi
        rospy.logdebug("Distance error:{}\tYaw error:{}".format(distance_error_measured,yaw_error_measured))
        if self.manual and distance_error_measured<self.distance_error_acceptance:
            if not self.notified:
                rospy.loginfo("{}: Override waypoint reached, holding position.".format(rospy.get_name()))
                try: 
                    rospy.wait_for_service("control_topic_mux/select",1.0)
                except:
                    pass
                else:
                    # call trigger service to switch to dynamic positioning
                    service_handle = rospy.ServiceProxy('control_topic_mux/select', MuxSelect)
                    service_handle("tau_com/DP")
                self.notified=True
        elif not self.manual: 
            if not self.hp and distance_error_measured<self.distance_error_acceptance:
                if not self.notified:
                    rospy.loginfo("{}: Automated waypoint reached, requesting next point.".format(rospy.get_name()))
                    self.notified=True
                    self.request_new()
            elif self.hp and abs(yaw_error_measured)<self.yaw_error_acceptance and distance_error_measured<self.distance_error_acceptance:
                if not self.notified:
                    rospy.loginfo("{}: Automated waypoint reached, requesting next point.".format(rospy.get_name()))
                    self.notified=True
                    self.request_new()

if __name__ == '__main__':
    guidance()
