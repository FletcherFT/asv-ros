#!/usr/bin/env python
import rospy
import serial
import threading
import time
from binascii import hexlify
from std_msgs.msg import String

class terminal():

    def __init__(self):
        self.msg_pub = rospy.Publisher("term_msg",String,queue_size=10)

    def run(self):
        while True:
            msg = raw_input("Input Command:  ")
            if msg == "exit" or msg == "q" or msg == "x":
                self.msg_pub.publish("exit")
                self.end()
                break
            else:
                self.msg_pub.publish(msg)

    def end(self):
        self.shutdown_flag=True
        rospy.signal_shutdown("Shutdown Command Given")

def shutdownHandle():
    print "Shutting Down"

def main():
    sh = terminal()
    rospy.init_node('ardu_terminal')
    rospy.on_shutdown(shutdownHandle)
    while not rospy.is_shutdown():
        sh.run()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
