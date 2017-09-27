#!/usr/bin/env python
import rospy
import serial, select
import struct
import tf.transformations as transform
import math
import numpy as np
from sensor_msgs.msg import Imu

class ImuDriverNode():
    def __init__(self):
        self.ser = serial.Serial(
                            port='/dev/ttyACM0',
                            baudrate = 115200,
                            bytesize = serial.EIGHTBITS,
                            parity = serial.PARITY_NONE,
                            stopbits = serial.STOPBITS_ONE,
                            timeout = 0.5,
                            )
        self.pub = rospy.Publisher('imu/data', Imu, queue_size=1)
        hz = rospy.Rate(50)
        while not rospy.is_shutdown():
            msg = self.imu_rx()
            if msg:
                self.pub.publish(msg)
            hz.sleep()

    def imu_rx(self):
        if self.ser.inWaiting()>0:
            msg = Imu()
            reading = self.ser.readline().rstrip()
            fmt = 'BBhhhhhhhhhi'
            if len(reading)==struct.calcsize(fmt):
                data = struct.unpack(fmt,reading)
            else:
                return 0

            #euler angles were multiplied by 100 for int byte conversion
            quaternion = transform.quaternion_from_euler(data[2]/100.0,data[3]/100.0,data[4]/100.0,'sxyz')
            msg.orientation.x=quaternion[0]
            msg.orientation.y=quaternion[1]
            msg.orientation.z=quaternion[2]
            msg.orientation.w=quaternion[3]

            #gyros were multiplied by 100 for int byte conversion
            msg.angular_velocity.x=data[5]/100.0
            msg.angular_velocity.y=data[6]/100.0
            msg.angular_velocity.z=data[7]/100.0
            #accelerations were multiplied by 100 for int byte conversion
            msg.linear_acceleration.x=data[8]/100.0
            msg.linear_acceleration.y=data[9]/100.0
            msg.linear_acceleration.z=data[10]/100.0
            msg.orientation_covariance = [  0.001520691, -0.001042596, 0.000183062,
                                            -0.001042596, 0.000745226, -0.000129729,
                                            0.000183062, 0.000129729, 0.000032987 ]
            msg.angular_velocity_covariance = [ 1e-9, 1e-9, 1e-9,
                                                1e-9, 1e-9, 1e-9,
                                                1e-9, 1e-9, 1e-9 ]
            msg.linear_acceleration_covariance = [  0.000081284, -0.000000467, -0.000001143,
                                                     -0.000000467, 0.000087904, 0.000000242,
                                                    -0.000001143, 0.000000242, 0.000174439 ]
            msg.header.frame_id='imu_link'
            msg.header.stamp=rospy.Time.now()
            return msg
        else:
            return 0

if __name__=='__main__':
    try:
        rospy.init_node('imu_read')
        h = ImuDriverNode()
    except rospy.ROSInterruptException:
        pass
