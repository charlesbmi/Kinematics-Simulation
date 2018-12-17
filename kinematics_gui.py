"""
DH-parameter simulation for a robotic arm

Something like https://goo.gl/images/QkQACc, or
https://www.amazon.com/SainSmart-Desktop-Grippers-Assembled-MEGA2560/dp/B00UMOSQCI/
"""
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d  # Needed for 3d-plot
import matplotlib.animation as animation
from matplotlib.widgets import Slider

# Denavit-Hartenberg parameters
_NUM_LINKAGES = 4
d_params = [0.5, 0, 0, 0]
a_params = [0, 1, 1, 0.25]
alpha_params = [np.deg2rad(90), 0, 0, 0]
theta_deg_params_0 = [0, 60, -90, 30]

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

# Attaching 3D axis to the figure
fig = plt.figure(figsize=(11,8))
plt.subplots_adjust(right=0.7)
ax_3d = fig.add_subplot(111, projection='3d')
matplotlib.rcParams.update({'font.size': 16})

# Setting the axes properties
ax_3d.set_xlim3d([-1.0, 1.0])
ax_3d.set_xlabel('X')
ax_3d.set_ylim3d([-1.0, 1.0])
ax_3d.set_ylabel('Y')
ax_3d.set_zlim3d([0.0, 2.0])
ax_3d.set_zlabel('Z')
ax_3d.set_title('Mechanism Simulation')

line_data = generate_coordinates(theta_deg_params=theta_deg_params_0)
l, = ax_3d.plot(line_data[0, :], line_data[1, :], line_data[2, :],
                linestyle='-', marker='h')

def redraw_lines(angles):
    # All 3 properties must be set for Line to be updated
    new_line = generate_coordinates(theta_deg_params=angles)
    l.set_data(new_line[0:2,:])
    l.set_3d_properties(new_line[2,:])
    fig.canvas.draw_idle()

# Add Slider
ax_color = 'lightgoldenrodyellow'
sliders = []
for ax_idx in range(_NUM_LINKAGES):
    ax_slider = plt.axes([0.75, 0.35 + (0.1 * ax_idx), 0.15, 0.05], facecolor=ax_color)
    theta_deg_0 = theta_deg_params_0[ax_idx]
    sliders.append(Slider(ax_slider, r'$\theta_{}$'.format(ax_idx + 1), -180.0, 180.0, valinit=theta_deg_0))

def update(_):
    # Arg (unused): value of the slider that triggered the update function
    angles = [slider.val for slider in sliders]
    redraw_lines(angles)

for slider in sliders:
    slider.on_changed(update)

plt.show()
