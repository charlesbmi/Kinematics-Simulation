"""
A simple example of an animated plot... In 3D!
"""
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button, RadioButtons

# Building something like https://goo.gl/images/QkQACc
# or rather: https://www.amazon.com/SainSmart-Desktop-Grippers-Assembled-MEGA2560/dp/B00UMOSQCI/
_LINKAGE_LENGTHS = [0.5, 0, 1, 1, 0.75, 0, 0.25]
_NUM_LINKAGES = len(_LINKAGE_LENGTHS)

def generate_coordinates(dims=3, theta_deg=None) :
    """
    Create a line using a random walk algorithm

    Assumes 3-D coordinates

    dims is the number of dimensions the line has.
    """
    lineData = np.empty((dims, _NUM_LINKAGES+1)) # 1 more joint than lines
    lineData[:,0] = [0.5, 0.5, 0.0]
    lineData[:,1] = lineData[:,0] + [0, 0, _LINKAGE_LENGTHS[0]]
    if theta_deg is None:
        theta_deg = np.random.uniform(-np.pi, np.pi, _NUM_LINKAGES-1)
    theta = [np.deg2rad(deg) for deg in theta_deg]

    for link_idx in range(1,len(_LINKAGE_LENGTHS)):
        linkage_length = _LINKAGE_LENGTHS[link_idx]
        joint_angle = theta[link_idx-1]
        lineData[:,link_idx+1] = [lineData[0,link_idx] + (linkage_length * np.cos(joint_angle)),
                                  lineData[1,link_idx] + (linkage_length * np.sin(joint_angle)),
                                  lineData[2,link_idx],
                                  ]
    return lineData

def update_lines(num, dataLines, lines) :
    for line, data in zip(lines, dataLines) :
        # NOTE: there is no .set_data() for 3 dim data...
        line.set_data(data[0:2, :num])
        line.set_3d_properties(data[2,:num])
    return lines

# Attaching 3D axis to the figure
fig = plt.figure()
plt.subplots_adjust(right=0.7)
ax_3d = fig.add_subplot(111, projection='3d')

# 3 lines of random 3-D lines
data = [generate_coordinates()]

# Setting the axes properties
ax_3d.set_xlim3d([0.0, 1.0])
ax_3d.set_xlabel('X')
ax_3d.set_ylim3d([0.0, 1.0])
ax_3d.set_ylabel('Y')
ax_3d.set_zlim3d([0.0, 1.0])
ax_3d.set_zlabel('Z')

ax_3d.set_title('Mechanism Simulation')

l, = ax_3d.plot(data[0][0, :], data[0][1, :], data[0][2, :],
                linestyle='-', marker='o')

# Add Slider
ax_color = 'lightgoldenrodyellow'
slider_axes = [plt.axes([0.8, 0.2 + (0.1 * ax), 0.15, 0.03], facecolor=ax_color) for ax in range(_NUM_LINKAGES - 1)]
sliders = [Slider(axis, 'Angle', -180.0, 180.0, valinit=15) for axis in slider_axes]

def update(val):
    angles = [slider.val for slider in sliders]
    # All 3 properties must be set for Line to be updated
    new_line = generate_coordinates(theta_deg=angles)
    l.set_data(new_line[0:2,:])
    l.set_3d_properties(new_line[2,:])
    fig.canvas.draw_idle()

for slider in sliders:
    slider.on_changed(update)

plt.show()
