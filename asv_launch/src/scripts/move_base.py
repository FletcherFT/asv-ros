#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from navigation_msgs.msg import Odometry
from std_msgs.msg import String, Bool

bool auto_manual_flag

def auto_manual_toggle(data):
    global auto_manual_flag
    auto_manual_flag = data

def setpoints_publish(data):
    Vx = data.linear.x
    Vy = data.linear.y
    Vt = data.angular.z
    msg = 'vc'+str(Vx)+','+str(Vy)+','+str(Vt)
    toArduPub.publish(msg)

def inputs_publish(data):
    Vx = data.twist.twist.linear.x
    Vy = data.twist.twist.linear.y
    Vz = data.twist.twist.angular.z
    msg = 'vi'+str(Vx)+','+str(Vy)+','+str(Vt)
    toArduPub.publish(msg)

def main():
    rospy.init_node('move_base')
    toArduPub = rospy.Publisher('msg_topic',String,queue_size=1)
    rospy.Subscriber('auto_manual',Bool,auto_manual_toggle)
    rospy.Subscriber('cmd_vel',Twist,setpoints_publish)
    rospy.Subscriber('odometry/filtered',Odometry,inputs_publish)
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
