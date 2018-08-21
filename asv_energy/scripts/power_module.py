#!/usr/bin/python
# Simple demo of reading each analog input from the ADS1x15 and printing it to
# the screen.
# Author: Tony DiCola
# License: Public Domain
import rospy
import numpy as np

# Import the ADS1x15 module.
import Adafruit_ADS1x15
from std_msgs.msg import Float32, String

# Create an ADS1115 ADC (16-bit) instance.
adc = Adafruit_ADS1x15.ADS1115()

# Choose a gain of 1 for reading voltages from 0 to 4.09V.
# Or pick a different gain to change the range of voltages that are read:
#  - 2/3 = +/-6.144V
#  -   1 = +/-4.096V
#  -   2 = +/-2.048V
#  -   4 = +/-1.024V
#  -   8 = +/-0.512V
#  -  16 = +/-0.256V
# See table 3 in the ADS1015/ADS1115 datasheet for more info on gain.
GAIN = 1
FACTORS = np.array([0.005029050402805,1.0,1.0,0.000741000453586])
ZEROS = np.array([-65.019643478960049,0.0,0.0,0.107945780783920])

CURRENT_PIN = 0
VOLTAGE_PIN = 3

# Main loop.
def main():
    rospy.init_node("adc_reader")
    pub = rospy.Publisher("adc",String,queue_size=1)
    hz = rospy.Rate(10)
    rospy.loginfo('Reading ADS1x15 values, press Ctrl-C to quit...')
    # Print nice channel column headers.
    rospy.logdebug('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*range(4)))
    rospy.logdebug('-' * 37)
    try:
        while not rospy.is_shutdown():
            # Read all the ADC channel values in a list.
            raw = np.zeros(4)
            current = np.zeros(4)
            for i in range(4):
                # Read the specified ADC channel using the previously set gain value.
                raw[i] = adc.read_adc(i, gain=GAIN)
                # Note you can also pass in an optional data_rate parameter that controls
                # the ADC conversion time (in samples/second). Each chip has a different
                # set of allowed data rate values, see datasheet Table 9 config register
                # DR bit values.
                #values[i] = adc.read_adc(i, gain=GAIN, data_rate=128)
                # Each value will be a 12 or 16 bit signed integer value depending on the
                # ADC (ADS1015 = 12-bit, ADS1115 = 16-bit).
            values = FACTORS*(raw)+ZEROS
            # Print the ADC values.
            rospy.logdebug('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*values))
            # Pause for half a second.
            pub.publish('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*values))
            hz.sleep()
    except rospy.ROSInterruptException:
        print("Exiting")

if __name__=="__main__":
    main()
