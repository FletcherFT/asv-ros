#!/usr/bin/python
import rospy
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
import numpy as np
from math import cos, sin, pi, radians, degrees
from optimisation import quadprog_solve_qp
inv = np.linalg.inv
mul = np.matmul
det = np.linalg.det

class ThrusterAllocationNode():
    def __init__(self):
        #step 0:  get the thrusters
        if rospy.has_param('~thruster'):
            self.t = rospy.get_param('~thruster')
        else:
            raise KeyError

        rospy.loginfo("...Initialising QP Framework...")
        t = self.t
        n = t['n']
	self.names = t['name']
        self.sigma = float(t['sigma'])
        #Construction of the P matrix (QP Standard Form)
        Pwr = t['pcoeff']
        Omg = t['omega']
        Q = [1e9 for i in range(n)]
        self.P = np.diag(Pwr+Omg+Q)
        #Construction of the initial
        #Initialise the thrusts and angles
        f0 = np.array([0.0 for i in range(n)])
        a0 = np.deg2rad(t['alpha0'])
        da = np.array([0.0 for i in range(n)])
        fmax = np.array(t['fmax'])
        fmin = np.array(t['fmin'])
        damax = np.array(t['rpmmax'])/60*2*pi
        damin = np.array(t['rpmmin'])/60*2*pi
        lx = np.array(t['lx'])
        ly = np.array(t['ly'])
        l = np.vstack((lx,ly))
        #Construction of the LHS inequality eqn (G)
        # for the maximum thrust constraints
        G = np.eye(n)
        G = np.hstack((G,np.zeros([n,2*n])))
        # for the minimum thrust constraints
        G = np.vstack((G,-G))
        # for the maximum angle change constraints
        temp = np.eye(n)
        temp = np.hstack((np.zeros([n,n]),temp,np.zeros([n,n])))
        # for the minimum angle change constraints
        temp = np.vstack((temp,-temp))
        # combine to get the entire G
        self.G = np.vstack((G,temp))
        # store these variables for later recalculation
        self.n = n
        self.f0 = f0
        self.a0 = a0
        self.da = da
        self.fmax = fmax
        self.fmin = fmin
        self.damax = damax
        self.damin = damin
        self.l = l
        # initialise tau commanded and tau solution (3DoF Solution)
        self.tau_com = np.zeros(3)
        self.tau_sol = np.zeros(3)

        self.prev_time = rospy.Time.now()
        self.update()
        rospy.Subscriber('tau_com',WrenchStamped,self.wrenchCallback)
        self.tsol_pub = rospy.Publisher('tau_sol',WrenchStamped,queue_size=1)
        self.thrusters_pub = rospy.Publisher('joint_states',JointState,queue_size=1)
        self.period = rospy.rostime.Duration.from_sec(1.0/10.0)
        self.timer = rospy.Timer(self.period, self.jointsend)

    def update(self):
        curr_time = rospy.Time.now()
        dt = (curr_time-self.prev_time).to_sec()
        self.prev_time = curr_time
        #recover saved variables
        n = self.n
        f0 = self.f0
        a0 = self.a0
        fmax = self.fmax
        fmin = self.fmin
        damax = self.damax
        damin = self.damin
        l = self.l
        #Construction of the RHS inequality eqn (h)
        #rows 0:n -> maximum saturation constraint
        h = fmax-f0
        #rows n+1:2n -> minimum saturation constraint
        h = np.append(h,-(fmin-f0))
        #rows 2n+1:3n -> maximum angle change constraint
        h = np.append(h,damax)
        #rows 3n+1:4n -> minimum angle change constraint
        h = np.append(h,-(damin))
        #Construct the thruster allocation matrix
        T = np.cos(a0)
        T = np.vstack((T,np.sin(a0)))
        temp = np.multiply(l[0,:],np.sin(a0))-np.multiply(l[1,:],np.cos(a0))
        T = np.vstack((T,temp))
        #Construct the derivative allocation matrix (set to a0)
        dTda = -np.sin(a0)
        dTda = np.vstack((dTda,np.cos(a0)))
        temp = np.multiply(l[0,:],np.cos(a0))+np.multiply(l[1,:],np.sin(a0))
        dTda = np.vstack((dTda,temp))
        #Construction of RHS equality eqn (b)
        tau = self.tau_com
        b = tau - mul(T,f0.T)
        #Construction of the LHS equality eqn (A)
        T1 = np.diag(mul(dTda,f0.T))
        A = np.hstack((T,T1,np.eye(n)))
        #Construction of the linear objective term (q)
        epsilon = 1e-9
        #maneuvrability scalar (large sigma = high maneuvrability, high power consumption)
        sigma = self.sigma
        q0 = np.zeros(n)
        q1 = np.repeat(sigma/(epsilon+det(dTda*dTda.T)),n)
        q = np.hstack((q0,q1,q0))
        return q,h,A,b

    def wrenchCallback(self, wrench_com_msg):
        tau=[]
        tau.append(wrench_com_msg.wrench.force.x)
        tau.append(wrench_com_msg.wrench.force.y)
        tau.append(wrench_com_msg.wrench.torque.z)
        #update the required tau
        self.tau_com=np.array(tau)
        #organise into QP standard form
        q,h,A,b = self.update()
        #recover saved variables
        n = self.n
        f0 = self.f0
        a0 = self.a0
        l = self.l
        try:
            #solve for x = [df,da,s]
            x = quadprog_solve_qp(self.P,q,self.G,h,A,b)
        except ValueError:
            #TODO publish error
            rospy.logwarn("No good solution, reverting to previous good solution")
            #set x to be all zeros
            x = np.zeros(3*n)
        #get change in thrusts
        df = x[0:n]
        #update thrusts
        f = f0+df
        #get change in angles
        da = x[n:2*n]
        #update angles
        a = a0+da
        #constrain angles to 0<=a<=2*pi
        a[a>pi]=-pi-(a[a>pi]-pi)
        a[a<=-pi]=pi+(pi+a[a<=-pi])
        self.f0 = f
        self.a0 = a
        self.da = da
        #get the solved wrench (really for debugging)
        T = np.cos(a)
        T = np.vstack((T,np.sin(a)))
        temp = np.multiply(l[0,:],np.sin(a))-np.multiply(l[1,:],np.cos(a))
        T = np.vstack((T,temp))
        self.tau_sol = mul(T,f.T)

        #publish the achieved wrench
        tsol = WrenchStamped()
        tsol.wrench.force.x = self.tau_sol[0]
        tsol.wrench.force.y = self.tau_sol[1]
        tsol.wrench.torque.z = self.tau_sol[2]
        tsol.header.frame_id = 'base_link'
        tsol.header.stamp = rospy.Time.now()
        self.tsol_pub.publish(tsol)

    def jointsend(self,event):
        joint_states = JointState()
        joint_states.header.stamp = rospy.Time.now()
        joint_states.name = self.names
        joint_states.position = self.a0
        joint_states.velocity = self.da
        joint_states.effort = self.f0
        self.thrusters_pub.publish(joint_states)

if __name__ == "__main__":
    try:
        rospy.init_node('thruster_allocator')
        node = ThrusterAllocationNode()
        rospy.spin()
    except KeyError as e:
        rospy.logerr("Parameters not found!")
    except rospy.ROSInterruptException:
        pass

