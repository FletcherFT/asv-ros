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
    xorientation=-1
    xcenter = xorientation*515
    xmin=xorientation*1023
    xmax=xorientation*0
    x = xorientation*msg.axes[4]
    if x<xcenter:
        outputMsg.wrench.force.x = interpolate(x,xmin,xcenter,-20,0)
    else:
        outputMsg.wrench.force.x = interpolate(x,xcenter,xmax,0,20)
    # map poistive Nz to CCW motion
    zorientation=1
    zcenter = zorientation*499
    zmin=zorientation*0
    zmax=zorientation*1023
    z = zorientation*msg.axes[3]
    if z<zcenter:
        outputMsg.wrench.torque.z = interpolate(z,zmin,zcenter,-5,0)
    else:
        outputMsg.wrench.torque.z = interpolate(z,zcenter,zmax,0,5)
    args.publish(outputMsg)

def main():
    rospy.init_node("teleop_remapper")
    pub = rospy.Publisher("tau_com/override",WrenchStamped,queue_size=10)
    rospy.Subscriber("joy",Joy,joyCallback,(pub))
    rospy.spin()

if __name__=="__main__":
    main()