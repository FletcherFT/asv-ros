#!/usr/bin/python
import rospy
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from numpy import *
import numpy as np
from scipy.linalg import block_diag
from math import cos, sin, pi, radians, degrees
from asv_control_msgs.msg import Thrusters
import cvxopt
from cvxopt import matrix

def cvxopt_solve_qp(P, q, G=None, h=None, A=None, b=None):
    P = .5 * (P + P.T)  # make sure P is symmetric
    args = [matrix(P), matrix(q)]
    if G is not None:
        args.extend([matrix(G), matrix(h)])
        if A is not None:
            args.extend([matrix(A), matrix(b)])
    sol = cvxopt.solvers.qp(*args)
    if 'optimal' not in sol['status']:
        return None
    return np.array(sol['x']).reshape((P.shape[1],))

class ConstrainedNonrotatableAllocation:
    def __init__(self):
        rospy.init_node("control_allocation")
        # default is AP mode
        self.thruster = rospy.get_param('thrusterAP')
        self.r = 3
        self.n = self.thruster['n']
        self.W = np.diag(self.thruster['W'])
        self.Q = np.diag(self.thruster['Q'])
        self.Phi = block_diag(self.W,self.Q,0)
        self.R = concatenate( (zeros((self.r+self.n+1,self.n+2*self.r)), concatenate((zeros((self.r+self.n,1)),1))),axis=1)
        for i in range(self.n):
            alpha = self.thruster['alpha'][i]
            lx = self.thruster['lx'][i]
            ly = self.thruster['ly'][i]
            if i==0:
                self.T=np.array([cos(alpha),sin(alpha),lx*sin(alpha)-ly*cos(alpha)])
            else:
                self.T=hstack((self.T,np.array([cos(alpha),sin(alpha),lx*sin(alpha)-ly*cos(alpha)])))
        self.T=reshape(self.T,(self.n,self.r)).T
        self.A1 = hstack((self.T,-eye(self.n),zeros((self.n,1))))
        self.C1 = hstack((eye(self.n),zeros((self.n,2*self.r+1))))
        self.A2 = block([
            [-eye(self.r),zeros((self.r,self.n+1))],
            [eye(self.r),zeros((self.r,self.n+1))],
            [eye(self.r),zeros((self.r,self.n)),ones((self.r,1))],
            [eye(self.r),zeros((self.r,self.n)),-ones((self.r,1))]
        ])
        self.C2 = block([
            [zeros((self.r,self.n)),-eye(self.r),zeros((self.r,self.r+1))],
            [zeros((self.r,self.n+self.r)),eye(self.r),zeros((self.r,1))],
            [zeros((2*self.r,self.n+self.r*2+1))]
        ])
        self.thrust_msg = Thrusters()
        self.thrust_pub = rospy.Publisher("thruster",Thrusters,queue_size=10)
        self.sol_msg = WrenchStamped()
        self.sol_pub = rospy.Publisher("tau_sol",WrenchStamped,queue_size=10)
        rospy.Subscriber("tau_com",WrenchStamped,self.wrenchCallback)
        rospy.spin()

    def wrenchCallback(self,msg):
        tau_com = np.array([[msg.wrench.force.x],[msg.wrench.force.y],[msg.wrench.torque.z]])
        p = cat( (tau_com, np.array(self.thruster['fmin'])[np.newaxis].T, np.array(self.thruster['fmax'])[np.newaxis].T,np.array([[self.thruster['Beta']]])))
        try:
            #solve for x = [df,da,s]
            #z = quadprog_solve_qp(self.Phi,mul(self.R,p),self.A2,mul(self.C2,p),self.A1,mul(self.C1,p))
            z = cvxopt_solve_qp(self.Phi,mul(self.R,p),self.A2,mul(self.C2,p),self.A1,mul(self.C1,p))
        except ValueError as exc:
            rospy.logerr(exc)
        except Exception as exc:
            rospy.logerr(exc)
            raise
        rospy.loginfo(z)
        thrusts = z[0:self.n]
        slacks = z[self.n:2*self.n]
        maxthrust = z[2*self.n]
        s = "|"+2*self.n*"\t{}\t|"
        rospy.logdebug(s.format(*z))
        tau_sol = mul(self.T,thrusts)
        self.sol_msg.header.stamp = rospy.Time.now()
        self.sol_msg.header.frame_id = "base_link"
        self.sol_msg.wrench.force.x = tau_sol[0]
        self.sol_msg.wrench.force.y = tau_sol[1]
        self.sol_msg.wrench.torque.z = tau_sol[2]
        self.sol_pub.publish(self.sol_msg)
        self.thrust_msg.header.stamp = rospy.Time.now()
        self.thrust_msg.header.frame_id = str(self.thruster['name'])
        self.thrust_msg.force = thrusts
        self.thrust_msg.rpm = self.forceToRPM(thrusts)
        self.thrust_msg.pwm = self.forceToPWM(thrusts)
        self.thrust_pub.publish(self.thrust_msg)

    def forceToRPM(self,thrusts):
        return [6,6,6]
    
    def forceToPWM(self,thrusts):
        return [100,100,100]

if __name__ == "__main__":
    try:
        node = ConstrainedNonrotatableAllocation()
    except KeyError as e:
        rospy.logerr("Parameters not found!")
    except rospy.ROSInterruptException:
        pass