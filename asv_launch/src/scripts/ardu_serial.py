#!/usr/bin/env python
import rospy
import serial
import threading
import time
from std_msgs.msg import String
from binascii import hexlify
import os

class serialHandler():

    def __init__(self):
        self.ser = serial.Serial(
                            port='/dev/ttyO4',
                            baudrate = 115200,
                            bytesize = serial.EIGHTBITS,
                            parity = serial.PARITY_NONE,
                            stopbits = serial.STOPBITS_ONE,
                            timeout = 0,
                            )
        self.inThread = threading.Thread(target=self.rx , args=())
        self.inThread.start()
        self.txArd_sub = rospy.Subscriber("to_ardu",String,self.tx)
        rospy.spin()

    def rx(self):
        while not rospy.is_shutdown():
            pkt = ''
            if self.ser.inWaiting():
                pkt = self.findPacket()
            if pkt:
                print "rx:  "+pkt

    def findPacket(self):
        try:
            pkt = ''
            cs = ''
            inByte = self.ser.read()
            if inByte == '~':
                #start byte found"
                #find size of incoming packet
                for i in range(2):
                    cs += self.ser.read()
                    length = int(cs,16)
                #read the rest of the packet
                for i in range(length):
                    inByte = self.ser.read()
                    if inByte == '\n':
                        break
                    else:
                        pkt += inByte
                return pkt
            else:
                return False
        except:
            return False

    def tx(self,msg):
        if msg.data == "exit":
            self.end()
        pkt = self.packData(msg.data)
        self.ser.write(pkt)

    def packData(self,data):
        pkt = data+'$'
        cs = len(data)
        return '~'+str(cs)+data+'$'

    def end(self):
        self.inThread.join()
        rospy.signal_shutdown("Shutdown Command Given")

def main():
    rospy.init_node('ardu_serial')
    sh = serialHandler()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
