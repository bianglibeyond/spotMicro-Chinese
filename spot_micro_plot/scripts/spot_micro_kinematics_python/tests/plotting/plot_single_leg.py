#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
from math import pi
from ...spot_micro_stick_figure import SpotMicroLeg
from ...utilities import transformations
from ...utilities import spot_micro_kinematics as smk

d2r = pi/180
r2d = 180/pi

# Attaching 3D axis to the figure
fig = plt.figure()
ax = p3.Axes3D(fig)

ax.set_xlabel('X')
ax.set_zlabel('Y')
ax.set_ylabel('Z')

ax.set_xlim3d([-0.15, 0.15])
ax.set_zlim3d([-0.25, 0.05])
ax.set_ylim3d([-0.15,0.15])

# ax.invert_yaxis()

# Set azimtuth and elevation of plot
# ax.view_init(elev=135,azim=0)

# Instantiate spot micro stick figure obeject
ht_start = transformations.homog_transform(0,0,0,0,0,0)
l1 = 0.055
l2 = 0.1075
l3 = 0.13
sml = SpotMicroLeg(0,0,0,l1,l2,l3,ht_start,leg12=True)

# Try leg to a desired position
x4 = 0
y4 = -0.18
z4 = 00.05

(q1,q2,q3) = smk.ikine(x4,y4,z4,l1,l2,l3,legs12 = True)

print('Leg angles')
print('q1: %2.1f deg, q2: %2.1f deg, q3: %2.1f deg'%(q1*r2d,q2*r2d,q3*r2d))

sml.set_angles(q1,q2,q3)
# sml.set_angles(10*d2r,0*d2r,00*d2r)


# Get leg coordinates
coords = sml.get_leg_points()

print('Right Back p4 coordinates:')
print('x4: %1.3f, y4: %1.3f, z4: %1.3f'%(coords[3][0],coords[3][1],coords[3][2]))


# Initialize empty list top hold line objects
lines = []

# Plot color order for leg links: (hip, upper leg, lower leg)
plt_colors = ['r','c','b']
for i in range(3):
    
    # Due to mplot3d rotation and view limitations, swap y and z to make the stick figure
    # appear oriented better
    x_vals = [coords[i][0], coords[i+1][0]]
    y_vals = [coords[i][1], coords[i+1][1]]
    z_vals = [coords[i][2], coords[i+1][2]]
    lines.append(ax.plot(x_vals,z_vals,y_vals,color=plt_colors[i])[0])


plt.show()
    
