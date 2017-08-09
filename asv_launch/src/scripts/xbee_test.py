#!/usr/bin/env python
from xbee import ZigBee
import serial
import binascii 
import struct
import rospy
from std_msgs.msg import String,Bool

class xbeeHandler():
    def __init__(self):
        self.ser = serial.Serial(
                                  port='/dev/ttyO5',
                                  baudrate = 115200,
                                  bytesize = serial.EIGHTBITS,
                                  parity = serial.PARITY_NONE,
                                  stopbits = serial.STOPBITS_ONE,
                                  timeout = 0,
                                  )
        self.xbee = ZigBee(self.ser,escaped = True,callback=self.rx)

    def rx(self,payload):
        print "YES"
        if payload.get('id') == 'tx_status':
            pass
        elif payload.get('rf_data'):
            data=payload.get('rf_data')
            rospy.loginfo("Received:  {}".format(binascii.hexlify(data)))
            self.thefonz(data)
        else:
            print "unknown packet:  "+str(payload)

    def thefonz(self,data):
        fonzie = struct.unpack('ffffff',data)
        print "the real shit"
        print fonzie

def main():
    rospy.init_node('xbee_rxtx')
    hz = rospy.Rate(40);
    xb = xbeeHandler()
    while not rospy.is_shutdown():
        hz.sleep()
    xb.xbee.halt()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
