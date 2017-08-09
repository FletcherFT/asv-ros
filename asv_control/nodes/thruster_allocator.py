#!/usr/bin/python
import rospy
from geometry_msgs.msg import WrenchStamped
from asv_control_msgs.msg import ThrusterComponents
import numpy

class ThrusterAllocationNode():
    """
    Node for calculating the thruster speed needed to apply a given wrench
    to an ASV. The underlying theory is an affine thruster model as described
    in 'Guidance and Control of Ocean Vehicles' by T. Fossen, section 4.1, 
    pp. 96-97. The calculation is done using a thruster allocation matrix (TAM)
    which defines how requested forces in each degree of freedom have to be
    translated to thruster forces.

    This node subscribes to 'wrench', expecting a geometry_msgs/WrenchStamped.
    Whenever a new message arrives on this topic, the desired motor speeds
    are calculated and published on the topic 'motor_levels' as
    asv_control_msgs/ThrusterComponents.
    """
    def __init__(self, tam, dof):
        self.tam = tam
        self.dof = dof
        rospy.Subscriber('wrench', WrenchStamped, self.wrenchCallback)
        self.pub = rospy.Publisher('thruster_components', ThrusterComponents,queue_size=1)
        rospy.loginfo('Listening for wrench requests on '
                      '%s...', rospy.resolve_name('wrench'))
        rospy.loginfo('Publishing motor levels on '
                      '%s...', rospy.resolve_name('thruster_components'))
        rospy.loginfo('Thruster allocation matrix:\n %s', str(self.tam))
        self.signed_sqrt_v = numpy.vectorize(self.signed_sqrt)

    def signed_sqrt(self, number):
        return -numpy.sqrt(-number) if number < 0 else numpy.sqrt(number)

    def wrenchCallback(self, wrench_stamped_msg):
        wrench = numpy.array([
                wrench_stamped_msg.wrench.force.x,
                wrench_stamped_msg.wrench.force.y,
                wrench_stamped_msg.wrench.force.z,
                wrench_stamped_msg.wrench.torque.x,
                wrench_stamped_msg.wrench.torque.y,
                wrench_stamped_msg.wrench.torque.z])[self.dof]
        thruster_components = self.signed_sqrt_v(numpy.dot(self.tam.T, wrench))
        thruster_components_msg = ThrusterComponents()
        thruster_components_msg.header.stamp = wrench_stamped_msg.header.stamp
        thruster_components_msg.header.frame_id = 'base_link'
        thruster_components_msg.x = thruster_components[0::2].tolist()
        thruster_components_msg.y = thruster_components[1::2].tolist()
        self.pub.publish(thruster_components_msg)

if __name__ == "__main__":
    try:
        rospy.init_node('thruster_allocator')
        taminfo = rospy.get_param("~tam")
        tam = numpy.array(map(float, taminfo['B'])).reshape(taminfo['rows'], taminfo['cols'])
        dof = numpy.where(numpy.array(taminfo['dof_mat']))
        node = ThrusterAllocationNode(tam,dof)
        rospy.spin()
    except KeyError as e:
        rospy.logerr("Parameter %s not set!", e)
