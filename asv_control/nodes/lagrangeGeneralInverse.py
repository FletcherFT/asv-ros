import numpy as np
inv = np.linalg.inv
mul = np.matmul

def main():
    #TODO Generate PW based on coefficients for each thruster P(Thrust) curve)
    PW = np.array([0.6916,0.6916,0.6916,0.6916,2.086,2.086])
    #Define thruster weighting matrix
    W = np.diag(PW)
    invW = inv(W)
    #Define decomposed thruster configuration matrix
    B = np.array([[1.,0.,1.,0.,0.,0.],[0.,1.,0.,1.,0.,1.],[-0.5,-0.15,-0.5,0.15,0.,0.7]])
    #Define desired control force vector
    tref = np.array([1.,1.,1.])
    print tref

    invWBT = mul(invW,B.T)
    f = mul(mul(invW,B.T),inv(mul(B,mul(invW,B.T))))
    f = mul(f,tref)
    print f
    tauS = mul(f,B.T)
    print tauS

if __name__=="__main__":
    main()
