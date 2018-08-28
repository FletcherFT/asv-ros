#!/usr/bin/python
import rospy
from digi.xbee.devices import ZigBeeDevice
from digi.xbee.models.mode import APIOutputMode
from digi.xbee.util import utils
import struct
from sensor_msgs.msg import Joy

# TODO: Replace with the serial port where your local module is connected to. 
PORT = "/dev/ttyUSB1"
# TODO: Replace with the baud rate of your local module.
BAUD_RATE = 115200
formatspec = 'cchhhhh????'
joy_msg = Joy()
device = ZigBeeDevice(PORT, BAUD_RATE)

def main():
    rospy.init_node("xbee_read")
    pub = rospy.Publisher("joy",Joy,queue_size=10)
    def explicit_data_callback(explicit_xbee_message):
        data = struct.unpack(formatspec,explicit_xbee_message.data)
        joy_msg.axes = data[5:7]
        joy_msg.buttons = data[7:]
        rospy.logdebug(data)
        joy_msg.header.stamp = rospy.Time.now()
        joy_msg.header.frame_id = "rc"
        pub.publish(joy_msg)
    try:
        device.open()
        device.set_api_output_mode(APIOutputMode.EXPLICIT)
        device.flush_queues()
        device.add_expl_data_received_callback(explicit_data_callback)
        print("Waiting for data in explicit format...\n")
        rospy.spin()
    finally:
        if device is not None and device.is_open():
            device.close()

if __name__ == '__main__':
    main()