#!/usr/bin/env python
from xbee import ZigBee,XBee
import serial
from binascii import hexlify
import re
import rospy
from std_msgs.msg import String,Bool
import subprocess

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
        self.cmd_dict={"ur":0,"xy":1,"re":2}
        self.toArduPub = rospy.Publisher("to_ardu",String,queue_size=1)
        self.toLogPub = rospy.Publisher("to_log",Bool,queue_size=1)
        rospy.Subscriber("to_xbee", String,self.tx)

    def rx(self,payload):
        if payload.get('id') == 'tx_status':
            print "tx_status"+str(payload)
        elif payload.get('rf_data'):
            if payload.get('rf_data')=='x':
                rospy.signal_shutdown("Exit Command Received")
            hexstring = hexlify(payload.get('rf_data'))
            self.process_pkt(self.cmd_dict.get(hexstring[0:4].decode("hex")),hexstring)
        else:
            print "unknown packet:  "+str(payload)
        

    def process_pkt(self,cmd,hexstring):
        #Universal Remote Command
        if cmd==0:
            hexlist = re.findall('....',hexstring[4:-8])
            hexlist.extend(re.findall('..',hexstring[-8:]))
            joy_out = "ur"
            for i in range(len(hexlist)):
                joy_out+=str(hexlist[i])
            self.toArduPub.publish(joy_out)
        elif cmd==2:
            val = int(hexstring.decode("hex")[-1:])
            rospy.loginfo("Telling Pi to Start Recording")
            self.toLogPub.publish(val)
        elif cmd==None:
            print "unknown command:  "+hexstring

    def tx(self,msg):
        self.xbee.tx(dest_addr_long='\x00\x13\xA2\x00\x40\xAF\xB7\x7B', data=msg.data)

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
