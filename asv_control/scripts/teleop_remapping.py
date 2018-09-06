#!/usr/bin/python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import WrenchStamped

def interpolate(x,in_min,in_max,out_min,out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def joyCallback(msg,args):
    # map channel five of axis to commanded surge: +20:

    # map channel four of axis to commanded yaw.
    outputMsg = WrenchStamped()
    outputMsg.header.frame_id="override"
    outputMsg.header.stamp = rospy.Time.now()
    # map positive X to forward motion
    outputMsg.wrench.force.x = interpolate(-msg.axes[4],-1023,0,-20,20)
    # map positive Y to port strafing motion on slider 2
    outputMsg.wrench.force.y = interpolate(msg.axes[0],54,1024,-3,3)
    # map poistive Nz to CCW motion
    outputMsg.wrench.torque.z = interpolate(msg.axes[3],0,1023,-5,5)
    args.publish(outputMsg)

def main():
    rospy.init_node("teleop_remapper")
    pub = rospy.Publisher("tau_com/override",WrenchStamped,queue_size=10)
    rospy.Subscriber("joy",Joy,joyCallback,(pub))
    rospy.spin()

if __name__=="__main__":
    main()