#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
from math import pi
from ...spot_micro_stick_figure import SpotMicroStickFigure
from ...utilities import spot_micro_kinematics as smk

d2r = pi/180
r2d = 180/pi


def update_lines(num, coord_data, lines):

    line_to_leg__and_link_dict =   {4:(0,0),
                                    5:(0,1),
                                    6:(0,2),
                                    7:(1,0),
                                    8:(1,1),
                                    9:(1,2),
                                    10:(2,0),
                                    11:(2,1),
                                    12:(2,2),
                                    13:(3,0),
                                    14:(3,1),
                                    15:(3,2)}

    for line, i in zip(lines, range(len(lines))):

        if i < 4:
            # First four lines are the square body
            if i == 3:
                ind = -1
            else:
                ind = i
            x_vals = [coord_data[num][ind][0][0], coord_data[num][ind+1][0][0]]
            y_vals = [coord_data[num][ind][0][1], coord_data[num][ind+1][0][1]]
            z_vals = [coord_data[num][ind][0][2], coord_data[num][ind+1][0][2]]
            # NOTE: there is no .set_data() for 3 dim data...
            line.set_data(x_vals,z_vals)
            line.set_3d_properties(y_vals)

    # Next 12 lines are legs
    # Leg 1, link 1, link 2, link 3
    # Leg 2, link 1, link 2, link 3...
        else:
            leg_num = line_to_leg__and_link_dict[i][0]
            link_num = line_to_leg__and_link_dict[i][1]
            x_vals = [coord_data[num][leg_num][link_num][0], coord_data[num][leg_num][link_num+1][0]]
            y_vals = [coord_data[num][leg_num][link_num][1], coord_data[num][leg_num][link_num+1][1]]
            z_vals = [coord_data[num][leg_num][link_num][2], coord_data[num][leg_num][link_num+1][2]]
            
            line.set_data(x_vals,z_vals)
            line.set_3d_properties(y_vals)
    return lines


# Attaching 3D axis to the figure
fig = plt.figure()
ax = p3.Axes3D(fig)

ax.set_xlabel('X')
ax.set_ylabel('Z')
ax.set_zlabel('Y')

ax.set_xlim3d([-0.2, 0.2])
ax.set_zlim3d([0, 0.4])
ax.set_ylim3d([-0.2,0.2])

# Set azimtuth and elevation of plot
# ax.view_init(elev=135,azim=0)

# Instantiate spot micro stick figure obeject
sm = SpotMicroStickFigure(x=0,y=0.14,z=0, theta=00*d2r)

# Define absolute position for the legs
l = sm.body_length
w = sm.body_width
l1 = sm.hip_length
l2 = sm.upper_leg_length
l3 = sm.lower_leg_length
desired_p4_points = np.array([ [-l/2,   0,  w/2 + l1],
                               [ l/2 ,  0,  w/2 + l1],
                               [ l/2 ,  0, -w/2 - l1],
                               [-l/2 ,  0, -w/2 - l1] ])

sm.set_absolute_foot_coordinates(desired_p4_points)

# Set a pitch angle
sm.set_body_angles(theta=00*d2r)

# Get leg coordinates
coords = sm.get_leg_coordinates()

# Initialize empty list top hold line objects
lines = []

# Construct the body of 4 lines from the first point of each leg (the four corners of the body)
for i in range(4):
    # For last leg, connect back to first leg point
    if i == 3:
        ind = -1
    else:
        ind = i

    # Due to mplot3d rotation and view limitations, swap y and z to make the stick figure
    # appear oriented better
    x_vals = [coords[ind][0][0], coords[ind+1][0][0]]
    y_vals = [coords[ind][0][1], coords[ind+1][0][1]]
    z_vals = [coords[ind][0][2], coords[ind+1][0][2]]
    lines.append(ax.plot(x_vals,z_vals,y_vals,color='k')[0])


# Plot color order for leg links: (hip, upper leg, lower leg)
plt_colors = ['r','c','b']
for leg in coords:
    for i in range(3):
        
        # Due to mplot3d rotation and view limitations, swap y and z to make the stick figure
        # appear oriented better
        x_vals = [leg[i][0], leg[i+1][0]]
        y_vals = [leg[i][1], leg[i+1][1]]
        z_vals = [leg[i][2], leg[i+1][2]]
        lines.append(ax.plot(x_vals,z_vals,y_vals,color=plt_colors[i])[0])

# Create data of robot pitching up and down
num_angles = 25
pitch_angles = np.linspace(-30*d2r,30*d2r,num_angles)
coord_data = []
for theta in pitch_angles:
    # Set a pitch angle
    sm.set_body_angles(theta=theta)

    x = sm.get_leg_angles()
    
    # Get leg coordinates and append to data list
    coord_data.append(sm.get_leg_coordinates())

coord_data = coord_data + coord_data[::-1]

# Creating the Animation object
line_ani = animation.FuncAnimation(fig, update_lines, num_angles*2, fargs=(coord_data, lines),
                                   interval=75, blit=False)

plt.show()
