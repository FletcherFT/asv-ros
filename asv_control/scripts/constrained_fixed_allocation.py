#!/usr/bin/python
import rospy
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from numpy import vstack, hstack, shape, diag, array, reshape, eye, zeros, ones, block, matmul
import numpy as np
from scipy.linalg import block_diag
from math import cos, sin, pi, radians, degrees
from asv_control_msgs.msg import Thrusters
from asv_control_msgs.srv import ConfigureSteppers
import quadprog

def quadprog_solve_qp(P, q, G=None, h=None, A=None, b=None):
    qp_G = .5 * (P + P.T)   # make sure P is symmetric
    qp_a = -q
    if A is not None:
        qp_C = -vstack([A, G]).T
        qp_b = -hstack([b, h])
        meq = A.shape[0]
    else:  # no equality constraint
        qp_C = -G.T
        qp_b = -h
        meq = 0
    return quadprog.solve_qp(qp_G, qp_a, qp_C, qp_b, meq)[0]

class ConstrainedNonrotatableAllocation:
    def __init__(self):
        rospy.init_node("allocator")
        # default is AP mode
        self.qpsetup(True)
        self.thrust_msg = Thrusters()
        self.thrust_pub = rospy.Publisher("thrusters",Thrusters,queue_size=10)
        self.sol_msg = WrenchStamped()
        self.sol_pub = rospy.Publisher("tau_sol",WrenchStamped,queue_size=10)
        self.mode_server = rospy.Service('~modeconfig',ConfigureSteppers,self.mode)
        rospy.Subscriber("tau_com",WrenchStamped,self.wrenchCallback)
        rospy.spin()

    def qpsetup(self,mode):
        if mode:
            self.thruster = rospy.get_param('thrusterAP')
        else:
            self.thruster = rospy.get_param('thrusterDP')
        self.kpwm = self.thruster['Kpwm']
        self.deadband = self.thruster['deadband']
        self.krpm = self.thruster['Krpm']
        self.r = 3
        self.n = self.thruster['n']
        self.W = diag(self.thruster['W'])
        self.Q = diag(self.thruster['Q'])
        self.Phi = block_diag(self.W,self.Q)
        self.R = zeros((self.r+self.n,self.n+2*self.r))
        for i in range(self.n):
            alpha = self.thruster['alpha'][i]
            lx = self.thruster['lx'][i]
            ly = self.thruster['ly'][i]
            if i==0:
                self.T=array([cos(alpha),sin(alpha),lx*sin(alpha)-ly*cos(alpha)])
            else:
                self.T=hstack((self.T,np.array([cos(alpha),sin(alpha),lx*sin(alpha)-ly*cos(alpha)])))
        self.T=reshape(self.T,(self.n,self.r)).T
        self.A1 = hstack((self.T,-eye(self.n)))
        self.C1 = hstack((eye(self.n),zeros((self.n,2*self.r))))
        self.A2 = block([
            [-eye(self.r),zeros((self.r,self.n))],
            [eye(self.r),zeros((self.r,self.n))],
            [zeros((self.r,self.r)),zeros((self.r,self.n))],
            [zeros((self.r,self.r)),zeros((self.r,self.n))]
        ])
        self.C2 = block([
            [zeros((self.r,self.n)),-eye(self.r),zeros((self.r,self.r))],
            [zeros((self.r,self.n+self.r)),eye(self.r)],
            [zeros((2*self.r,self.n+self.r*2))]
        ])

    def wrenchCallback(self,msg):
        tau_com = array([msg.wrench.force.x,msg.wrench.force.y,msg.wrench.torque.z])
        p = hstack((tau_com,array(self.thruster['fmin']),array(self.thruster['fmax'])))
        P = self.Phi
        q = matmul(self.R,p)
        A = self.A1
        b = matmul(self.C1,p)
        G = self.A2
        h = matmul(self.C2,p)
        try:
            #solve for x = [df,da,s]
            z = quadprog_solve_qp(P,q,G,h,A,b)
        except ValueError as exc:
            rospy.logerr(exc)
        except Exception as exc:
            rospy.logerr(exc)
        finally:
            pass
        thrusts = z[0:self.n]
        slacks = z[self.n:2*self.n]
        s = "|"+(2*self.n)*"\t{:.4}\t|"
        rospy.logdebug(s.format(*z))
        tau_sol = matmul(self.T,thrusts)
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

    def mode(self,request):
        try:
            self.qpsetup(request.mode)
            if request.mode:
                S="Configuring for AP mode"
            else:
                S="Configuring for DP mode"
        except Exception as exc:
            response=[False,str(exc)]
        else:
            rospy.logdebug(S)
            response=[True,S]
        return response

    def forceToRPM(self,thrusts):
        rpm = [0,0,0]
        for idx,i in enumerate(thrusts):
            if i>0:
                rpm[idx]=self.krpm[idx]*i**2
            else:
                rpm[idx]=-self.krpm[idx]*i**2
        return rpm
    
    def forceToPWM(self,thrusts):
        pwm = [0,0,0]
        for idx,i in enumerate(thrusts):
            if i>0:
                pwm[idx]=self.kpwm[idx]*i**2#+self.deadband[idx]
                if abs(pwm[idx])<self.deadband[idx]:
                    pwm[idx]=0
            else:
                pwm[idx]=-self.kpwm[idx]*i**2#-self.deadband[idx]
                if abs(pwm[idx])<self.deadband[idx]:
                    pwm[idx]=0
        return pwm

if __name__ == "__main__":
    try:
        node = ConstrainedNonrotatableAllocation()
    except KeyError as e:
        rospy.logerr("Parameters not found!")
    except rospy.ROSInterruptException:
        pass