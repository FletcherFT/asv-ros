#!/usr/bin/python
import rospy
from sensor_msgs.msg import BatteryState
from asv_energy.msg import Float64Stamped
from std_srvs.srv import Trigger

class EnergyMonitor:
    def __init__(self):
        rospy.init_node('energy_monitor')
        self._flag = False
        self._prev_energy = 0
        self._power_pub = rospy.Publisher('energy/power',Float64Stamped,queue_size=10)
        self._energy_pub = rospy.Publisher('energy/aggregate',Float64Stamped,queue_size=10)
        self._power_msg = Float64Stamped()
        self._energy_msg = Float64Stamped()
        self._energy_datum_srv = rospy.Service("energy/datum",Trigger,self.datumCallback)
        rospy.Subscriber("energy/battery",BatteryState,self.batteryCallback)
        rospy.spin()

    def datumCallback(self,request):
        self._prev_energy = 0
        return [True,"Reset energy aggregator."]

    def batteryCallback(self,msg):
        if not self._flag:
            self._flag = True
            self._stamp = msg.header.stamp
            power = msg.voltage*msg.current
            self._prev_power = power
        else:
            dt = (msg.header.stamp - self._stamp).to_sec()
            self._stamp = msg.header.stamp
            power = msg.voltage*msg.current # instantaneous power
            self._prev_energy+=(power + self._prev_power)*dt/2.0 # aggregate the energy
            self._prev_power = power
        self._power_msg.header = msg.header
        self._power_msg.data = power
        self._energy_msg.header = msg.header
        self._energy_msg.data = self._prev_energy
        self._power_pub.publish(self._power_msg)
        self._energy_pub.publish(self._energy_msg)

if __name__=="__main__":
    try:
        EnergyMonitor()
    except rospy.ROSInterruptException:
        pass