#--------------------------------------
#Generate reference trajectory for NMPC
#--------------------------------------

import numpy as np

sample_time = 0.025             # seconds
duration = 120                   # seconds

# Trajectory
traj = np.zeros((int(duration/sample_time+1),10)) # x y z phi theta psi u v w p q r u1 u2 u3 u4

traj[:,0] = 0                       # x
traj[:,1] = 0                       # y
traj[:,2] = 0                       # psi
traj[:,3] = 0                       # u
traj[:,4] = 0                       # v
traj[:,5] = 0                       # r
traj[:,6] = 0                      # u1
traj[:,7] = 0                      # u2
traj[:,8] = 0                      # u3
traj[:,9] = 0                      # u4

# write to txt
np.savetxt('stationary.txt',traj,fmt='%f')