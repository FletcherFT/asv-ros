#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix

def main():
	rospy.init_node("fake_gps")
	point=rospy.get_param("~coords",[-41.40155,147.12094,0])
	pub = rospy.Publisher("/ublox_gps/fix",NavSatFix,queue_size=10)
	rate = rospy.Rate(4)
	msg = NavSatFix()
	while not rospy.is_shutdown():
		msg.header.stamp=rospy.Time.now()
		msg.header.frame_id="gps_link"
		msg.latitude=point[0]
		msg.longitude=point[1]
		msg.altitude=point[2]
		pub.publish(msg)
		rate.sleep()

if __name__=="__main__":
	main()