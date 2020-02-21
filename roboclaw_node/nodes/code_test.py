#!/usr/bin/env python

from math import pi, cos, sin

import numpy as np

X = np.array([0.00,0.00,0.00])  
get_cmd_vel=0
U =np.array([[0],[0],[0],[0]])

lx = 0.2485
ly = 0.1762
r = 0.0635

B_inv = np.array([[1.00, -1.00, -(lx+ly)],[1.00, 1.00, (lx+ly)] ,[1.00, 1.00, -(lx+ly)], [1.00, -1.00, (lx+ly)]])
        
X[0]=0.5
X[1]=0
X[2]=0

U_inv =X

#wheel velocities 
U = np.dot(B_inv,U_inv)

print("U:")
print(U)

real_sp_m1 = ((U[0]/1.79)*63)+64
real_sp_m2 = ((U[1]/1.79)*63)+64
real_sp_m3 = ((U[2]/1.79)*63)+64
real_sp_m4 = ((U[3]/1.79)*63)+64


print("real_sp_m1:")
print(real_sp_m1)

print("real_sp_m2:")
print(real_sp_m2)

print("real_sp_m3:")
print(real_sp_m3)

print("real_sp_m4:")
print(real_sp_m4)