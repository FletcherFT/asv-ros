#!/usr/bin/env python
import rospy
import serial
import tf.transformations as transform
import math
import numpy as np
from sensor_msgs.msg import Imu

def imu_rx():
    if ser.inWaiting()>0:
        msg = Imu()
        reading = ser.readline().strip().strip('\x00')
        data = reading.split(',')
        if len(data)==9:
            try:
                tmp = [float(i) for i in data]
                tmp[0] = -tmp[0]
                tmp[2] = -tmp[2]
                tmp[3] = -tmp[3]
                tmp[5] = -tmp[5]
                tmp[7] = -tmp[7]
                vals = transform.quaternion_from_euler(tmp[2],tmp[1],tmp[0],'rzyx')
                msg.header.stamp=rospy.Time.now()
                msg.header.frame_id='imu_link'
                msg.orientation.x=vals[0]
                msg.orientation.y=vals[1]
                msg.orientation.z=vals[2]
                msg.orientation.w=vals[3]
                msg.angular_velocity.x=tmp[3]
                msg.angular_velocity.y=tmp[4]
                msg.angular_velocity.z=tmp[5]
                msg.linear_acceleration.x=tmp[6]
                msg.linear_acceleration.y=tmp[7]
                msg.linear_acceleration.z=tmp[8]
                msg.orientation_covariance = [  0.001520691, -0.001042596, 0.000183062,
                                                -0.001042596, 0.000745226, -0.000129729,
                                                0.000183062, 0.000129729, 0.000032987 ]
                msg.angular_velocity_covariance = [ 1e-9, 1e-9, 1e-9,
                                                    1e-9, 1e-9, 1e-9,
                                                    1e-9, 1e-9, 1e-9 ]
                msg.linear_acceleration_covariance = [  0.000081284, -0.000000467, -0.000001143,
                                                         -0.000000467, 0.000087904, 0.000000242,
                                                        -0.000001143, 0.000000242, 0.000174439 ]
                return msg
            except ValueError as e:
                rospy.logerr(e)
                return 0
        else:
            return 0

def main():
    pub = rospy.Publisher('imu/data', Imu, queue_size=10)
    rospy.init_node('imu_read')
    hz = rospy.Rate(50)
    while not rospy.is_shutdown():
        msg = imu_rx()
        if msg:
            pub.publish(msg)

if __name__=='__main__':
    try:
        ser = serial.Serial(
                            port='/dev/ttyO4',
                            baudrate = 115200,
                            bytesize = serial.EIGHTBITS,
                            parity = serial.PARITY_NONE,
                            stopbits = serial.STOPBITS_ONE,
                            timeout = 0.05,
                            )
        main()
    except rospy.ROSInterruptException:
        pass
