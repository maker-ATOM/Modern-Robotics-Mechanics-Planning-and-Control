# import libraries
import modern_robotics as mr
import numpy as np

# modified IKinBody function
def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 20
    Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist, \
                                                      thetalist)), T)))
    err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
          or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
    while err and i < maxiterations:
        thetalist = thetalist \
                    + np.dot(np.linalg.pinv(mr.JacobianBody(Blist, \
                                                         thetalist)), Vb)
        i = i + 1
        Vb \
        = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist, \
                                                       thetalist)), T)))
        err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
              or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev

        # display intermediate iterations      
        print("Iteration:", i+1)
        print("joint vector:", )
        print(thetalist)
        print("SE(3) end effector config:",)
        print(mr.FKinBody(M,Blist, thetalist))
        print("error_twist:")
        print(Vb)
        print("angular_error_magnitude ||omega_b||", err)
        print("linear_error_magnitude ||omega_b||", ev)
        print(" ")
        
# paramters for IK
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

# call IKinBodyIterates to calculate joint angles
IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev)