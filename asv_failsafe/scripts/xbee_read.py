#!/usr/bin/python
import rospy
from digi.xbee.devices import ZigBeeDevice
from digi.xbee.models.mode import APIOutputMode
import struct
from sensor_msgs.msg import Joy
from digi.xbee.exception import InvalidPacketException, InvalidOperatingModeException, TimeoutException, XBeeException
import time

class xbee_read():
    def __init__(self):
        rospy.init_node("xbee_read")
        PORT = rospy.get_param("~device","/dev/ttyUSB1")
        BAUD_RATE = rospy.get_param("~baud",115200)
        self.joy_msg = Joy()
        self.device = ZigBeeDevice(PORT, BAUD_RATE)
        hz = rospy.Rate(10)
        self.pub = rospy.Publisher("joy",Joy,queue_size=10)
        try:
            self.device.open()
        except Exception as exc:
            rospy.logerr(exc)
            if self.device.is_open:
                self.device.close()
            return
        self.device.flush_queues()
        self.device.set_api_output_mode(APIOutputMode.EXPLICIT)
        rospy.sleep(rospy.Duration.from_sec(2.0))
        rospy.on_shutdown(self.shutdown_handle)
        while not rospy.is_shutdown():
            try:
                data_msg = self.device.read_expl_data()
            except TimeoutException:
                rospy.logerr("Timeout!")
            except InvalidPacketException:
                rospy.logerr("Bad Checksum")
            except XBeeException as exc:
                rospy.logerr("XBee Error!: {}".format(exc))
            else:
                if data_msg:
                    self.handle_data(data_msg)
            hz.sleep()

    def shutdown_handle(self):
        if self.device._is_open:
            self.device.close()

    def handle_data(self,data_msg):
        if len(data_msg.data)==16:
            formatspec = '>cchhhhh????'
            self.parse_joystick(struct.unpack(formatspec,data_msg.data))
        elif len(data_msg.data)==3:
            formatspec = 'cc?'
            self.parse_commands(struct.unpack(formatspec,data_msg.data))

    def parse_joystick(self,data):
        rospy.loginfo(data[2:7])
        self.joy_msg.axes = data[2:7]
        self.joy_msg.buttons = data[7:]
        rospy.logdebug(data)
        self.joy_msg.header.stamp = rospy.Time.now()
        self.joy_msg.header.frame_id = "override"
        self.pub.publish(self.joy_msg)

    def parse_commands(self,data):
        cmd = data[0]+data[1]
        if cmd == 'st':
            if data[2]:
                rospy.loginfo("Start Mission Request")
            else:
                rospy.loginfo("Stop Mission Request")
        elif cmd == 'rt':
            if data[2]:
                rospy.loginfo("Return Home Request")
            else:
                rospy.loginfo("Resume Mission Request")
        else:
            rospy.logwarn("Unknown Command: {}".format(cmd))

if __name__ == '__main__':
    try:
        a=xbee_read()
    except rospy.ROSInterruptException:
        if a.device.is_open:
            a.device.close()