#!/usr/bin/python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import WrenchStamped

def interpolate(x,in_min,in_max,out_min,out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def joyCallback(msg,args):
    # map channel five of axis to commanded surge: +20:
#0.5, 0.5, 0.49000000953674316, 0.49000000953674316
    # map channel four of axis to commanded yaw.
    outputMsg = WrenchStamped()
    outputMsg.header.frame_id="override"
    outputMsg.header.stamp = rospy.Time.now()
    # map positive X to forward motion
    xreverse = True
    xcenter = 0.49000000953674316
    xmin=0
    xmax=1.0
    x = msg.axes[2]
    if xreverse:
        if x<xcenter:
            outputMsg.wrench.force.x = interpolate(x,xcenter,xmin,0,100)
        else:
            outputMsg.wrench.force.x = interpolate(x,xmax,xcenter,-100,0)
    else:
        if x<xcenter:
            outputMsg.wrench.force.x = interpolate(x,xmin,xcenter,-100,0)
        else:
            outputMsg.wrench.force.x = interpolate(x,xcenter,xmax,0,100)

    # map positive Y to port motion
    yreverse = False
    ycenter = 0.49000000953674316
    ymin=0
    ymax=1.0
    y = msg.axes[3]
    if yreverse:
        if y<ycenter:
            outputMsg.wrench.force.y = interpolate(y,ycenter,ymin,0,20)
        else:
            outputMsg.wrench.force.y = interpolate(y,ymax,ycenter,-20,0)
    else:
        if y<ycenter:
            outputMsg.wrench.force.y = interpolate(y,ymin,ycenter,-20,0)
        else:
            outputMsg.wrench.force.y = interpolate(y,ycenter,ymax,0,20)

    # map positive Nz to CCW motion
    zreverse = False
    zcenter = 0.5
    zmin=0
    zmax=1.0
    z = msg.axes[0]
    if zreverse:
        if z<zcenter:
            outputMsg.wrench.torque.z = interpolate(z,zcenter,zmin,0,20)
        else:
            outputMsg.wrench.torque.z = interpolate(z,zmax,zcenter,-20,0)
    else:
        if y<ycenter:
            outputMsg.wrench.torque.z = interpolate(z,zmin,zcenter,-20,0)
        else:
            outputMsg.wrench.torque.z = interpolate(z,zcenter,zmax,0,20)
    args.publish(outputMsg)

def main():
    rospy.init_node("teleop_remapper")
    pub = rospy.Publisher("tau_com/override",WrenchStamped,queue_size=10)
    rospy.Subscriber("joy",Joy,joyCallback,(pub))
    rospy.spin()

if __name__=="__main__":
    main()
