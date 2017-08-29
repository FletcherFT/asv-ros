#!/usr/bin/python
import rospy
from geometry_msgs.msg import WrenchStamped
from asv_control_msgs.msg import ThrusterComponents
import numpy as np
inv = np.linalg.inv
pinv = np.linalg.pinv
mul = np.matmul

class ThrusterAllocationNode():
    def __init__(self, taminfo, motorinfo):

        #get the thruster allocation matrix, tam
        self.B = np.array(map(float, taminfo['B'])).reshape(taminfo['cols'], 6)

        pcoeff = np.repeat(np.array(motorinfo['Pcoeff']),2)

        #construct the inverse power weighting matrix
        self.invW = inv(np.diag(pcoeff))

        #obtain the thruster saturation limits
        self.mins = np.array(motorinfo['mins'])
        self.maxs = np.array(motorinfo['maxs'])

        rospy.Subscriber('wrench_commanded', WrenchStamped, self.wrenchCallback)
        self.wrench_pub = rospy.Publisher('wrench_achieved',WrenchStamped,queue_size=1)
        self.thrusters_pub = rospy.Publisher('thruster_components', ThrusterComponents,queue_size=1)
        rospy.loginfo('Listening for wrench requests on '
                      '%s...', rospy.resolve_name('wrench_commanded'))
        rospy.loginfo('Publishing motor levels on '
                      '%s...', rospy.resolve_name('thruster_components'))
    def normalize(self,v):
        norm=np.linalg.norm(v)
        if norm==0: 
           return v
        return v/norm

    def wrenchCallback(self, wrench_com_msg):
        tcom=[]
        tcom.append(wrench_com_msg.wrench.force.x)
        tcom.append(wrench_com_msg.wrench.force.y)
        tcom.append(wrench_com_msg.wrench.force.z)
        tcom.append(wrench_com_msg.wrench.torque.x)
        tcom.append(wrench_com_msg.wrench.torque.y)
        tcom.append(wrench_com_msg.wrench.torque.z)
        tcom = np.array(tcom)

        #obtain lagrangian multipliers (use pinv to avoid singular matrix errors)
        f = mul(mul(self.invW,self.B.T),pinv(mul(self.B,mul(self.invW,self.B.T))))

        #obtain the corresponding thrusts
        thrusts = mul(f,tcom)

        #make sure the thrusters are saturated if necessary
        thrustmags = np.sqrt(thrusts[0::2]**2+thrusts[1::2]**2)
        count = 0
        for idx,i in enumerate(thrustmags>self.maxs):
        #if the bugger is saturated
            if i:
                count = count + 1
                #normalise the thrust vector, then multiply by saturation
                unit = self.normalize(thrusts[[2*idx,2*idx+1]])
                thrusts[[2*idx,2*idx+1]]=unit*self.maxs[idx]
        if count>0:
            rospy.logwarn("{} Thrusters are Saturated, turn down the heat!".format(count))

        #publish the thrusts
        thruster_components_msg = ThrusterComponents()
        thruster_components_msg.header.frame_id = 'base_link'
        thruster_components_msg.x = thrusts[0::2].tolist()
        thruster_components_msg.y = thrusts[1::2].tolist()
        thruster_components_msg.header.stamp = rospy.Time.now()
        self.thrusters_pub.publish(thruster_components_msg)

        #obtain achieved wrench
        tach = mul(thrusts,self.B.T)
        
        #publish the achieved wrench
        wrench_ach = WrenchStamped()
        wrench_ach.wrench.force.x = tach[0]
        wrench_ach.wrench.force.y = tach[1]
        wrench_ach.wrench.force.z = tach[2]
        wrench_ach.wrench.torque.x = tach[3]
        wrench_ach.wrench.torque.y = tach[4]
        wrench_ach.wrench.torque.z = tach[5]
        wrench_ach.header.frame_id = 'base_link'
        wrench_ach.header.stamp = rospy.Time.now()
        self.wrench_pub.publish(wrench_ach)


if __name__ == "__main__":
    try:
        rospy.init_node('thruster_allocator')
        taminfo = rospy.get_param("~tam")
        motorinfo = rospy.get_param("~motorinfo")
        node = ThrusterAllocationNode(taminfo,motorinfo)
        rospy.spin()
    except KeyError as e:
        rospy.logerr("Parameter %s not set!", e)

