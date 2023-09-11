import modern_robotics as mr
import numpy as np

def IKbody(Blist, M, T, thetalist0, eomg, ev):
    thetalist = thetalist0;
    i = 0;
    maxiterations = 20;
    Vb = mr.se3ToVec(mr.MatrixLog6(mr.TransInv(mr.FKinBody(M, Blist.T, thetalist)) * T))

    # err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;
    # while err && i < maxiterations
    thetalist = thetalist + np.linalg.pinv(mr.JacobianBody(Blist.T, thetalist)) * Vb;
    i = i + 1;
    Vb = mr.se3ToVec(mr.MatrixLog6(mr.TransInv(mr.FKinBody(M, Blist.T, thetalist)) * T));
    #     err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;
    # end
    # success = ~ err;
    # end
    return thetalist

Blist = np.array([[0,  0,  1,  0,  3,  0],  
                  [0,  0,  1,  0,  2,  0],  
                  [0,  0,  1,  0,  1,  0]])

M = np.array([[1, 0, 0, 3],
              [0, 1, 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])

T = np.array([[-0.585, -0.811, 0, 0.076],  
              [ 0.811, -0.585, 0, 2.608], 
              [     0,      0, 1,     0], 
              [     0,      0, 0,     1]])

thetalist0 = [0.7854, 0.7854, 0.7854]

eomg = 0.001 
ev = 0.0001
mr.IKinBody

print(mr.IKinBody(Blist, M, T, thetalist0, eomg, ev))