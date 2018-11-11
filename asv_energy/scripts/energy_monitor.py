#!/usr/bin/python
import rospy
from sensor_msgs.msg import BatteryState
from asv_messages.msg import Float64Stamped,Readings
from std_srvs.srv import Trigger
from math import sqrt

class EnergyMonitor:
    def __init__(self):
        rospy.init_node('energy_monitor')
        self._flag = False
        self._prev_power_mu = 0
        self._prev_power_std = 0
        self._power_mu = 0
        self._power_std = 0
        self._energy_mu = 0
        self._energy_std = 0
        self._power_pub = rospy.Publisher('energy/power',Readings,queue_size=10)
        self._energy_pub = rospy.Publisher('energy/aggregate',Readings,queue_size=10)
        self._power_msg = Readings()
        self._energy_msg = Readings()
        self._current_std = 0.008
        self._voltage_std = 0.01
        rospy.Subscriber("energy/battery",BatteryState,self.batteryCallback)
        rospy.spin()

    def batteryCallback(self,msg):
        # First Message
        if not self._flag:
            self._flag = True
            self._stamp = msg.header.stamp
            self._prev_power_mu = self._power_mu
            self._prev_power_std = self._power_std
        else:
            # get the time step since last update
            dt = (msg.header.stamp - self._stamp).to_sec()
            self._stamp = msg.header.stamp
            self.calcPower(msg.voltage,msg.current)
            # calculate the mean and std of the energy consumed
            self._energy_mu += (self._prev_power_mu + self._power_mu)*dt/2.0 # aggregate the energy
            self._energy_std = self._energy_std + self._prev_power_std+self._power_std
            # store the previous values of power distribution
            self._prev_power_mu = self._power_mu
            self._prev_power_std = self._power_std

        # send the power and energy RV readings.
        self._power_msg.header = msg.header
        self._power_msg.data = [self._power_mu,self._power_std]
        self._energy_msg.header = msg.header
        self._energy_msg.data = [self._energy_mu,self._energy_std]
        self._power_pub.publish(self._power_msg)
        self._energy_pub.publish(self._energy_msg)

    def calcPower(self,V,I):
        # POWER IS A PRODUCT OF TWO NORMAL VARIABLES, V AND I
        var_V = (self._voltage_std)**2
        var_I = (self._current_std)**2
        self._power_mu = V*I
        self._power_std = sqrt(var_V*var_I + var_V*(I**2) + var_I*(V**2))

if __name__=="__main__":
    try:
        EnergyMonitor()
    except rospy.ROSInterruptException:
        pass