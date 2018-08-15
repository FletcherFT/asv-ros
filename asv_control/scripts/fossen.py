import numpy as np
from math import sin, cos
mul = np.matmul
#DONE
def smtrx(r):
    return np.array([[0,-r[2],r[1]],[r[2],0,-r[0]],[-r[1],r[0],0]])
#DONE
def m2c(M,v):
    nu1 = v[0:3]
    nu2 = v[3:]
    M = 0.5*(M+M.T)
    M11 = M[0:3,0:3]
    M12 = M[0:3,3:]
    M21 = M12.T
    M22 = M[3:,3:]
    nu1 = v[0:3]
    nu2 = v[3:]
    dt_dnu1 = mul(M11,nu1)+mul(M12,nu2)
    dt_dnu2 = mul(M21,nu1)+mul(M22,nu2)
    top = np.concatenate((np.zeros([3,3]),-smtrx(dt_dnu1)))
    btm = np.concatenate((-smtrx(dt_dnu1),-smtrx(dt_dnu2)))
    return np.concatenate((top,btm),1)
def gvect(W,B,theta,phi,r_g,r_b):
    #putting this into ros convention (x forward, y left, z up)
    sth = sin(theta)
    cth = cos(theta)
    sphi = sin(phi)
    cphi = cos(phi)
    return np.array([
            -(W-B)*sth,
            (W-B)*cth*sphi,
            (W-B)*cth*cphi,
            (r_g[1]*W-r_b[1]*B)*cth*cphi-(r_g[2]*W-r_b[2]*B)*cth*sphi,
            -(r_g[2]*W-r_b[2]*B)*sth-(r_g[0]*W-r_b[0]*B)*cth*cphi,
            (r_g[0]*W-r_b[0]*B)*cth*sphi-(r_g[1]*W-r_b[1]*B)*sth
            ])
