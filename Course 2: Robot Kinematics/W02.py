import modern_robotics as mr
import numpy as np

blist = np.array([[0,0,0,0],[0,0,0,0],[0,0,-1,0],[0,0,0,0]])

tlist = np.array([1.57, 1.57, 1])

# Js = mr.JacobianBody(blist,tlist)
Ms = mr.MatrixExp6(blist)
Adj = mr.Adjoint(Ms)
# print(Adj)

b3 = np.array([0,0,0,0,0,1])
x = np.array([[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, -1],[0, 0, 0, 0]])
at = np.array([[ 1,  0,  0,  0],[ 0,  1,  0,  0], [ 0,  0,  1, -1], [ 0,  0,  0,  1]])
adj = mr.Adjoint(at)
# print(adj)


slist = np.array([[ 0,  0,  0],[ 0,  0,  0], [ 1,  1,  1] ,[ 0,  0,  0],[ 0,  -1,  -2],[ 0,  0,  0]])
m = np.array([[1, 0, 0, 3], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
t = np.array([[-0.585, -0.811, 0, 0.076], [0.811, -0.585, 0, 2.608], [0, 0, 1, 0], [0, 0, 0, 1]])
tnot = np.array([[0.7854],[0.7854],[0.7854]])
eomg = 0.001
ev = 0.0001

Slist = np.array([[0, 0,  1,  4, 0,    0],
                    [0, 0,  0,  0, 1,    0],
                    [0, 0, -1, -6, 0, -0.1]])
M = np.array([[-1, 0,  0, 0],
                [ 0, 1,  0, 6],
                [ 0, 0, -1, 2],
                [ 0, 0,  0, 1]])
T = np.array([[0, 1,  0,     -5],
                [1, 0,  0,      4],
                [0, 0, -1, 1.6858],
                [0, 0,  0,      1]])
thetalist0 = np.array([1.5, 2.5, 3])
eomg = 0.01
ev = 0.001

ik = mr.IKinSpace(Slist, M, T, thetalist0, eomg, ev)
print(ik)