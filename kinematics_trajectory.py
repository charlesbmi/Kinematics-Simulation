"""
DH-parameter simulation for a robotic arm

Something like https://goo.gl/images/QkQACc, or
https://www.amazon.com/SainSmart-Desktop-Grippers-Assembled-MEGA2560/dp/B00UMOSQCI/
"""
import csv
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button, RadioButtons
import tkinter as tk
from tkinter.filedialog import askopenfilename


# Denavit-Hartenberg parameters
_NUM_LINKAGES = 4
d_params = [0.5, 0, 0, 0]
a_params = [0, 1, 1, 0.25]
alpha_params = [np.deg2rad(90), 0, 0, 0]
theta_deg_params_0 = [0, 0, 0, 0]

def generate_coordinates(theta_deg_params=None) :
    """
    Create a line with the passed in Denavit-Hartenberg theta parameters, and
    the global d/a/alpha-parameters.

    Assumes 3-D coordinates

    Args:
        theta_deg_params (list(float)): List of mechanism joint angles (in degrees)
    """
    lineData = np.empty((3, _NUM_LINKAGES+1)) # 1 more joint than links
    lineData[:,0] = [0.5, 0.5, 0.0]

    lineData[:,1] = lineData[:,0] + [0, 0, 0.5]
    if theta_deg_params is None:
        theta_deg_params = [0, 0, 0, 0]
    theta_params = [np.deg2rad(deg) for deg in theta_deg_params]

    # Calculate homogenous matrices for rotation and translation
    T_all = [np.identity(4)]
    for link_idx in range(_NUM_LINKAGES):
        # Variables correspond to DH parameters
        theta = theta_params[link_idx]
        d = d_params[link_idx]
        a = a_params[link_idx]
        alpha = alpha_params[link_idx]

        rot_mat = np.array(
            [[np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha)],
             [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha)],
             [0            ,  np.sin(alpha)                ,  np.cos(alpha)]
            ])
        trans_mat = np.array(
            [[a * np.cos(theta)],
            [a * np.sin(theta)],
            [d]])
        Tn = np.concatenate((np.concatenate((rot_mat, trans_mat), axis=1), [[0, 0, 0, 1]]), axis=0)
        T_all.append(Tn)

    for idx in range(len(T_all)):
        # Multiply homogenous rotation+translation matrices to get
        # fixed-observer coordinates
        T = np.identity(4)
        for jdx in range(idx+1):
            T = np.matmul(T, T_all[jdx])
        # coordinates = lineData translations
        lineData[:,idx] = T[:-1,-1]

    return lineData

# Open up csv file
trajectory_csv_filename = askopenfilename()
row_count = 0
with open(trajectory_csv_filename,'r') as trajectory_csv_file:
    row_count = sum(1 for row in csv.reader(trajectory_csv_file))

# Attaching 3D axis to the figure
fig = plt.figure(figsize=(11,8))
ax = p3.Axes3D(fig)

# Setting the axes properties
ax.set_xlim3d([-1.0, 1.0])
ax.set_xlabel('X')
ax.set_ylim3d([-1.0, 1.0])
ax.set_ylabel('Y')
ax.set_zlim3d([0.0, 2.0])
ax.set_zlabel('Z')
ax.set_title('Mechanism Simulation')

line_data = generate_coordinates(theta_deg_params=theta_deg_params_0)
l, = ax.plot(line_data[0, :], line_data[1, :], line_data[2, :],
             linestyle='-', marker='h')

def redraw_lines(angles):
    # All 3 properties must be set for Line to be updated
    new_line = generate_coordinates(theta_deg_params=angles)
    l.set_data(new_line[0:2,:])
    l.set_3d_properties(new_line[2,:])
    fig.canvas.draw_idle()

def animate(idx):
    with open(trajectory_csv_filename, "r") as trajectory_csv_file:
        trajectory = list(csv.reader(trajectory_csv_file))
        angles = trajectory[idx]
        angles = [float(angle) for angle in angles]
        redraw_lines(angles)

trajectory_animation = animation.FuncAnimation(
    fig, animate, row_count, interval=100, repeat_delay=1000, repeat=True)

plt.show()
