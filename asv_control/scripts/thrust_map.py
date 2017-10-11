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
            t = rospy.get_param('~thruster')
        else:
            raise KeyError
        #initialise
        self.prev_time = rospy.Time.now()
        self.update(t)
        rospy.Subscriber('tau_com',WrenchStamped,self.wrenchCallback)
        self.tsol_pub = rospy.Publisher('tau_sol',WrenchStamped,queue_size=1)
        self.thrusters_pub = rospy.Publisher('joint_states',JointState,queue_size=1)
        self.period = rospy.rostime.Duration.from_sec(1.0/10.0)
        self.timer = rospy.Timer(self.period, self.jointsend)

    def update(self,t=None):
        curr_time = rospy.Time.now()
        dt = (curr_time-self.prev_time).to_sec()
        if dt>0.1:
            dt=0.01
        self.prev_time = curr_time
        #checks if this is initialisation
        if not t is None:
            rospy.loginfo("...Initialising...")
            names = []
            #the thrust/speed matrix (K)
            k = []
            #the power cost diagonal
            power = []
            #the angle change cost diagonal
            omega = []
            #the slack variable cost diagonal
            Q = []
            #the previous force vector
            f0 = []
            #the previous angle vector
            a0 = []
            #the current change in angle
            da = []
            #number of thrusters
            n = t['n']
            #largest thrust possible
            fmax = 0.
            for i in range(n):
                names.append(t['name'][i])
                #K matrix components
                k.append(t['kcoeff'][i])
                #P matrix components
                power.append(t['pcoeff'][i]) #the higher the power is the less the thruster will be used
                omega.append(float(t['omega'][i])) #the higher omega is the less the thrusters will turn
                Q.append(1e3) #should be really large to penalise the slack variable (i.e. get tau_sol as close to tau_com as possible)
                #initialising the thrust vector
                f0.append(0.)
                #initialising the previous angle vector
                a0.append(radians(t['alpha0'][i]))
                #initialising the currrent change in angle
                da.append(0.)
            self.names=names
            self.K = np.diag(k)
            self.P = np.diag(power+omega+Q)
            #build the G matrix (this never changes)
            G = np.eye(3*n)
            #index for each row of x
            elems = [2,4]
            elems = np.repeat(elems,n)
            temp = range(2*n)
            temp = np.repeat(temp,elems)
            G = G[temp]
            #flip the sign to get standard form
            G[1::2] = G[1::2]*-1
            self.G = G
            #store the f0 and a0 lists for later
            self.f0 = f0
            self.a0 = a0
            self.da = da

            #initial tau
            self.tau = np.array([0.,0.,0.])
            #initial tsol
            self.tsol = np.array([0.,0.,0.])
            #store the thruster info
            self.t = t
            return
        else:
            #thruster info
            t = self.t
            #previous force
            f0 = self.f0
            #previous angle
            a0 = self.a0
            n = t['n']
        #init the h vector components
        #the inequality constraints for Delta f
        hf = []
        #the inequality constraints for Delta alpha
        ha = []
        #Thruster allocation matrix (fossen 7.421)
        T = []
        #partial derivative of T w.r.t. alpha (fossen 7.438)
        dTda = []
        #objective function linear offset term q
        #tiny offset to avoid inversion problems
        epsilon = 1e-9
        #maneuvrability scalar (large sigma = high maneuvrability, high power consumption)
        sigma = 0.01
        for i in range(n):
            #INEQUALITY CONTRAINT EQUATION (RHS)
            #THRUSTER SATURATION CONSTRAINTS
            #TODO added scalar to ensure thrust change doesn't oscillate between the minimum and maximum
            #df <= fmax-f0
            hf.append(0.01*(t['fmax'][i]-f0[i]))
            #-df <= f0-fmin
            hf.append(0.01*(f0[i]-t['fmin'][i]))
            #da <= amax-a0
            ha.append(radians(t['alphamax'][i])-a0[i])
            #-da <= a0-amin
            ha.append(a0[i]-radians(t['alphamin'][i]))
            #da <= 2*pi*rpmmax/60*dt
            ha.append(2*pi*t['rpmmax'][i]/60*dt)
            #-da <= -2*pi*rpmmin/60*dt
            ha.append(-2*pi*t['rpmmin'][i]/60*dt)
            #update T and dTda
            lx = t['lx'][i]
            ly = t['ly'][i]
            #T = [cosai,sinai,-lyicosai+lxisinai]
            T.append([cos(a0[i]),sin(a0[i]),-ly*cos(a0[i])+lx*sin(a0[i])])
            #dT/da = [-sinai,cosai,lyisinai+lxicosai]
            dTda.append([-sin(a0[i]),cos(a0[i]),lx*cos(a0[i])+ly*sin(a0[i])])
        T = np.array(T).T
        dTda = np.array(dTda).T
        #update the q vector
        q0 = np.zeros(n,dtype='float')
        q1 = np.array(np.repeat(sigma/(epsilon+det(dTda*dTda.T)),n))
        q = np.hstack((q0,q1,q0))
        #update the h vector
        h = np.array(hf+ha)
        #linear constraint dTda*f0
        T1 = np.diag(mul(dTda,np.array(f0).T))
        #update the A matrix
        A = np.hstack((T,T1,np.eye(n)))
        #update the b vector
        tau = np.array(self.tau)
        #b = tau - T(a0)*u0
        b = tau-mul(T,np.array(f0).T)
        return q,h,A,b

    def getTau(self,f,a):
        T = []
        for i in range(self.t['n']):
            #update T 
            lx = self.t['lx'][i]
            ly = self.t['ly'][i]
            #add row to T matrix (to be Transposed after for loop)
            T.append([cos(a[i]),sin(a[i]),-ly*cos(a[i])+lx*sin(a[i])])
        return mul(np.array(T).T,np.array(f).T)

    def wrenchCallback(self, wrench_com_msg):
        tau=[]
        tau.append(wrench_com_msg.wrench.force.x)
        tau.append(wrench_com_msg.wrench.force.y)
        tau.append(wrench_com_msg.wrench.torque.z)
        #update the required tau
        self.tau=np.array(tau)
        #organise into QP standard form
        q,h,A,b = self.update(None)
        try:
            #solve for x = [df,da,s]
            x = quadprog_solve_qp(self.P,q,self.G,h,A,b)
        except ValueError:
            rospy.logwarn("No good solution, reverting to previous good solution")
            f = np.array(self.f0)
            a = np.array(self.a0)
        else:
            n = self.t['n']
            #previous thrusts
            f0 = np.array(self.f0,dtype='float')
            #get change in thrusts
            df = x[0:n]
            #update thrusts
            f = f0+df
            #previous angle
            a0 = np.array(self.a0,dtype='float')
            #get change in angles
            da = x[n:2*n]
            #update angles
            a = a0+da
            f[abs(f)<0.01]=0.
            a[abs(a)<0.01]=0.
            self.f0 = f.tolist()
            self.a0 = a.tolist()
            self.da = da.tolist()
        #get the solved wrench (really for debugging)
        self.tsol = self.getTau(f.tolist(),a.tolist())

        #publish the achieved wrench
        tsol = WrenchStamped()
        tsol.wrench.force.x = self.tsol[0]
        tsol.wrench.force.y = self.tsol[1]
        tsol.wrench.torque.z = self.tsol[2]
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

