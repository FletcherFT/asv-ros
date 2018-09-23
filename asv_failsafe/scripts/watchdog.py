#!/usr/bin/python
import rospy
from std_msgs.msg import Empty, Bool

class watchdog:
	def __init__(self):
		rospy.init_node("watchdog")
		self._robot = rospy.get_param('~robot',False)
		self._state = False
		self.pub=rospy.Publisher("out",Empty,queue_size=1)
		if self._robot:
			self.rangepub = rospy.Publisher("failsafe/inrange",Bool,queue_size=10)
		pubrate = rospy.get_param("~pubrate",1.0)
		checkrate = rospy.Rate(5)
		timeout = rospy.get_param("~timeout",10.0)
		self.start = rospy.Time.now()
		rospy.Timer(rospy.Duration(1.0/pubrate),self.out_cb)
		rospy.Subscriber("in",Empty,self.in_cb)
		while not rospy.is_shutdown():
			if (rospy.Time.now()-self.start).to_sec()>timeout:
				self.alert()
			checkrate.sleep()

	def alert(self):
		if self._state:
			self._state = False
			if self._robot:
				self.rangepub.publish(self._state)
		rospy.logwarn("{}: No response from other end for {} seconds!".format(rospy.get_name(),(rospy.Time.now()-self.start).to_sec()))

	def out_cb(self,event):
		self.pub.publish()

	def in_cb(self,msg):
		self.start = rospy.Time.now()
		rospy.logdebug("Response received, resetting watchdog.")
		if not self._state:
			self._state = True
			if self._robot:
				self.rangepub.publish(self._state)

if __name__=="__main__":
	watchdog()