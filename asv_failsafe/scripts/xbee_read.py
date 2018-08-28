#!/usr/bin/python
import rospy
from digi.xbee.devices import ZigBeeDevice
from digi.xbee.models.mode import APIOutputMode
#from digi.xbee.util import utils
import struct
from sensor_msgs.msg import Joy

class xbee_read():
    def __init__(self):
        rospy.init_node("xbee_read")
        PORT = rospy.get_param("~device","/dev/ttyUSB2")
        BAUD_RATE = rospy.get_param("~baud",115200)
        self.joy_msg = Joy()
        self.device = ZigBeeDevice(PORT, BAUD_RATE)
        self.pub = rospy.Publisher("joy",Joy,queue_size=10)

    def explicit_data_callback(self,explicit_xbee_message):
        if len(explicit_xbee_message.data)==16:
            formatspec = 'cchhhhh????'
            data = struct.unpack(formatspec,explicit_xbee_message.data)
            self.joy_msg.axes = data[5:7]
            self.joy_msg.buttons = data[7:]
            rospy.logdebug(data)
            self.joy_msg.header.stamp = rospy.Time.now()
            self.joy_msg.header.frame_id = "rc"
            self.pub.publish(self.joy_msg)

    def run(self):
        try:
            self.device.open()
            self.device.set_api_output_mode(APIOutputMode.EXPLICIT)
            self.device.flush_queues()
            self.device.add_expl_data_received_callback(self.explicit_data_callback)
            rospy.loginfo("{}: Waiting for data in explicit format...".format(rospy.get_name()))
            rospy.spin()
        finally:
            if self.device is not None and self.device.is_open():
                self.device.close() 

if __name__ == '__main__':
    xbee_read()