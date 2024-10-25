#--------------------------------------
#Generate reference trajectory for NMPC
#--------------------------------------

import numpy as np
import math
#import tf.transformations as tf


# Parameters
sample_time = 0.05             # seconds
duration = 240                   # seconds

r = 2
v = 1.5

x0 = 0                       
y0 = 0

# trajectory
traj = np.zeros((int(duration/sample_time+1),10)) # x y z phi theta psi u v w p q r u1 u2 u3 u4
t = np.arange(0,duration,sample_time)
t = np.append(t, duration)

traj[:,0] = -r*np.cos(t*v/r)+x0     # x
traj[:,1] = -r*np.sin(t*v/r)+y0     # y
traj[:,2] = t*v/r-0.5*np.pi         # psi
traj[:,3] = v*np.sin(t*v/r)         # u
traj[:,4] = -v*np.cos(t*v/r)        # v
traj[:,5] = 0                      # r
traj[:,6] = 0                      # u1
traj[:,7] = 0                      # u2
traj[:,8] = 0                      # u3
traj[:,9] = 0                      # u4
'''
for i in range(0,int(duration/sample_time+1)):
    if np.sin(traj[i,5])>=0:
        traj[i,5] = traj[i,5]%np.pi
    else:
        traj[i,5] = -np.pi + traj[i,5]%np.pi
    #traj[i,5] = np.sin(traj[i,5])
'''


# write to txt
np.savetxt('circle.txt',traj,fmt='%f')
