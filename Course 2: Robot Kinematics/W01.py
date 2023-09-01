import modern_robotics as mr
import numpy as np

m = np.array([[1,0,0,3.732],[0,1,0,0],[0,0,1,2.732],[0,0,0,1]])

slist = np.array([[0,0,0,0,0,0],[0,1,1,1,0,0],[1,0,0,0,0,1],[0,0,1,-0.732,0,0],[-1,0,0,0,0,-3.732],[0,1,2.732,3.732,1,0]])
rlist = np.array([[0,0,0,0,0,0],[0,1,1,1,0,0],[1,0,0,0,0,1],[0,2.732,3.732,2,0,0],[2.732,0,0,0,0,0],[0,-2.732,-1,0,1,0]])

tlist = np.array([-1.57, 1.57, 1.0471, -0.785, 1, 0.5235])

T = mr.FKinSpace(m, slist, tlist)
print(T)
T = mr.FKinBody(m, rlist, tlist)
print(T)
