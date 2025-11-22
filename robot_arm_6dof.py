# robot_arm_6dof.py
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, Button, TextBox
import matplotlib.gridspec as gridspec
import matplotlib.image as mpimg
import os

# -----------------------
# DH transform utility
# -----------------------
def dh_transform(theta_deg, d, a, alpha_deg):
    th = math.radians(theta_deg)
    al = math.radians(alpha_deg)
    ct, st = math.cos(th), math.sin(th)
    ca, sa = math.cos(al), math.sin(al)
    T = np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0 ,       sa,      ca,     d   ],
        [0 ,        0,       0,     1   ]
    ], dtype=float)
    return T

# -----------------------
# Robot classes
# -----------------------
class Link:
    def __init__(self, a=0.0, alpha=0.0, d=0.0, theta_offset=0.0, name=""):
        self.a = a
        self.alpha = alpha
        self.d = d
        self.theta_offset = theta_offset
        self.name = name

class RobotArm:
    def __init__(self, links):
        self.links = links
        self.n = len(links)

    def forward_kinematics(self, thetas):
        assert len(thetas) == self.n
        Ts = [np.eye(4)]
        for link, th in zip(self.links, thetas):
            theta = th + link.theta_offset
            A = dh_transform(theta, link.d, link.a, link.alpha)
            Ts.append(Ts[-1] @ A)
        return Ts

# -----------------------
# Draw helpers
# -----------------------
def draw_robot_simple(ax, Ts, show_frames=True, frame_len=0.05):
    ax.cla()
    origins = [T[:3,3] for T in Ts]
    # line between origins
    xs = [p[0] for p in origins]; ys = [p[1] for p in origins]; zs = [p[2] for p in origins]
    ax.plot(xs, ys, zs, color='gray', linewidth=3, marker='o', markersize=6)

    # frames
    if show_frames:
        for T in Ts:
            origin = T[:3,3]
            R = T[:3,:3]
            x_e = origin + R[:,0]*frame_len
            y_e = origin + R[:,1]*frame_len
            z_e = origin + R[:,2]*frame_len
            ax.plot([origin[0], x_e[0]],[origin[1], x_e[1]],[origin[2], x_e[2]], color='r')
            ax.plot([origin[0], y_e[0]],[origin[1], y_e[1]],[origin[2], y_e[2]], color='g')
            ax.plot([origin[0], z_e[0]],[origin[1], z_e[1]],[origin[2], z_e[2]], color='b')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')

# -----------------------
# Build the "waist+shoulder+elbow+spherical wrist" robot
# -----------------------
# Default units: meters. Tweak these with sliders
# These DH params are a simple representative layout for a 6-DOF articulated arm:
# Link 1 (waist): rotate about Z, raises base height (d)
# Link 2 (shoulder): long link a2
# Link 3 (elbow): long link a3
# Link 4 (wrist roll): axis rotated (alpha 90) to allow spherical wrist
# Link 5 (wrist pitch)
# Link 6 (wrist yaw / tool)
links = [
    Link(a=0.0,  alpha=90, d=0.08,  name='waist'),     # base height, rotate about Z
    Link(a=0.25, alpha=0,  d=0.0,   name='shoulder'),  # shoulder link (a2)
    Link(a=0.20, alpha=0,  d=0.0,   name='elbow'),     # elbow link (a3)
    Link(a=0.0,  alpha=90, d=0.0,   name='wrist_roll'),# rotate to make spherical wrist
    Link(a=0.0,  alpha=-90,d=0.0,   name='wrist_pitch'),
    Link(a=0.05, alpha=0,  d=0.0,   name='tool')       # end effector small length
]
robot = RobotArm(links)

# initial slider values (degrees for thetas, meters for lengths)
init_thetas = [0.0, -30.0, 60.0, 0.0, -20.0, 0.0]
init_lengths = [links[0].d, links[1].a, links[2].a, links[3].a, links[4].a, links[5].a]

# -----------------------
# Build Matplotlib UI
# -----------------------
plt.rcParams['toolbar'] = 'toolbar2'
fig = plt.figure(figsize=(13,6))
gs = gridspec.GridSpec(1, 3, width_ratios=[2.4, 0.9, 0.01], wspace=0.25)

ax3d = fig.add_subplot(gs[0], projection='3d')
ax_side = fig.add_subplot(gs[1])
ax_side.axis('off')

# load reference image (your uploaded sketch)
ref_path = "/mnt/data/A_3D-rendered_digital_illustration_displays_a_six-.png"
if os.path.exists(ref_path):
    ax_img = fig.add_axes([0.75, 0.05, 0.12, 0.12])
    ax_img.imshow(mpimg.imread(ref_path))
    ax_img.axis('off')

# slider axes positions
theta_sliders = []
len_sliders = []
n = robot.n
start_y = 0.92
h = 0.045
left = 0.74
width = 0.22

for i in range(n):
    ax_theta = fig.add_axes([left, start_y - i*(h+0.01), width, h])
    s = Slider(ax_theta, f"θ{i+1} (deg)", -180, 180, valinit=init_thetas[i], valfmt='%0.0f')
    theta_sliders.append(s)

for i in range(n):
    ax_len = fig.add_axes([left, start_y - (n+i)*(h+0.01), width, h])
    # limit lengths reasonably: base height 0.01..0.30, link lengths 0.01..0.6
    if i==0:
        s = Slider(ax_len, f"d1 (m)", 0.01, 0.3, valinit=init_lengths[i], valstep=0.005)
    else:
        s = Slider(ax_len, f"a{i+1} (m)", 0.01, 0.6, valinit=init_lengths[i], valstep=0.005)
    len_sliders.append(s)

# Reset button and EEF display
ax_reset = fig.add_axes([left, 0.02, 0.08, 0.04])
btn_reset = Button(ax_reset, 'Reset')
ax_text = fig.add_axes([left+0.09, 0.02, 0.13, 0.04])
txt_box = TextBox(ax_text, 'EEF (m)', initial='')

# Update routine
def update(val=None):
    thetas = [s.val for s in theta_sliders]
    lengths = [s.val for s in len_sliders]
    # map slider lengths into links:
    robot.links[0].d = lengths[0]   # base height (d1)
    robot.links[1].a = lengths[1]   # shoulder a2
    robot.links[2].a = lengths[2]   # elbow a3
    # wrist links (i=3,4) keep small a values, but allow changing if slider moved
    robot.links[3].a = lengths[3]
    robot.links[4].a = lengths[4]
    robot.links[5].a = lengths[5]
    Ts = robot.forward_kinematics(thetas)
    draw_robot_simple(ax3d, Ts, show_frames=True, frame_len=0.04)
    total = sum([robot.links[i].a if i>0 else robot.links[i].d for i in range(robot.n)])
    lim = max(0.2, total*1.4)
    ax3d.set_xlim([-lim, lim]); ax3d.set_ylim([-lim, lim]); ax3d.set_zlim([0, lim*1.4])
    ax3d.view_init(elev=30, azim=-60)
    ee = Ts[-1][:3,3]
    txt_box.set_val(f"{ee[0]:.3f}, {ee[1]:.3f}, {ee[2]:.3f}")
    fig.canvas.draw_idle()

for s in theta_sliders: s.on_changed(update)
for s in len_sliders: s.on_changed(update)

def on_reset(event):
    for s, v in zip(theta_sliders, init_thetas):
        s.set_val(v)
    for s, v in zip(len_sliders, init_lengths):
        s.set_val(v)
    update()
btn_reset.on_clicked(on_reset)

# initial draw
update()
plt.suptitle("6-DOF Arm (waist, shoulder, elbow, spherical wrist) — Option A layout")
plt.show()
