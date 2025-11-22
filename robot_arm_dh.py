# robot_arm_sideview.py
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, Button, TextBox
import matplotlib.gridspec as gridspec
import matplotlib.image as mpimg
import os

# ------------------------
# Utility: rotation about Z
# ------------------------
def rotz(theta_deg):
    t = math.radians(theta_deg)
    c, s = math.cos(t), math.sin(t)
    R = np.array([[c, -s, 0],
                  [s,  c, 0],
                  [0,  0, 1]])
    return R

# ------------------------
# Forward kinematics (side-plane method)
# ------------------------
def forward_kinematics_side(theta1_deg, theta2_deg, theta3_deg, theta4_deg,
                            d1_cm, L2_cm, L3_cm, L4_cm):
    """
    Compute joint origins positions in global frame (cm).
    Convention:
      - Base origin at (0,0,0).
      - Base height (waist) raises first joint by +Z = d1_cm.
      - Link angles theta2..theta4 measured in degrees where:
          0 deg = pointing forward (positive X)
         90 deg = pointing up (positive Z)
      - Theta1 rotates the planar chain around Z (waist), so we then view side-projection (XZ).
    Returns list of positions p0..p4 as numpy arrays [x,y,z] in cm.
    """
    # base origin
    p0 = np.array([0.0, 0.0, 0.0])

    # first joint (shoulder) is at base height
    p_shoulder = p0 + np.array([0.0, 0.0, d1_cm])  # (0,0,d1)

    # helper: compute link vector in local XZ plane (dx, dz)
    def link_vec(length_cm, angle_deg):
        a = math.radians(angle_deg)
        dx = length_cm * math.cos(a)
        dz = length_cm * math.sin(a)
        return dx, dz

    # compute shoulder link vector in local coordinates
    dx2, dz2 = link_vec(L2_cm, theta2_deg)
    # after waist rotation theta1, the dx component projects into X,Y
    phi = math.radians(theta1_deg)
    Rx = math.cos(phi)
    Ry = math.sin(phi)
    v2 = np.array([dx2 * Rx, dx2 * Ry, dz2])

    p_elbow = p_shoulder + v2

    dx3, dz3 = link_vec(L3_cm, theta2_deg + theta3_deg)  # relative sum for serial chain
    v3 = np.array([dx3 * Rx, dx3 * Ry, dz3])
    p_wrist = p_elbow + v3

    dx4, dz4 = link_vec(L4_cm, theta2_deg + theta3_deg + theta4_deg)
    v4 = np.array([dx4 * Rx, dx4 * Ry, dz4])
    p_ee = p_wrist + v4

    return [p0, p_shoulder, p_elbow, p_wrist, p_ee]

# ------------------------
# Drawing helpers
# ------------------------
def draw_side_robot(ax, positions_cm, show_frames=True, frame_len=5.0):
    """
    positions_cm: list of points [p0..pN] in cm
    Side view will show X (horizontal) vs Z (vertical). Y is depth, but we keep 3D plot for nicer look.
    """
    ax.cla()

    # convert cm -> meters for nicer axis or keep cm? we'll keep cm to match your numbers
    pts = np.array(positions_cm)  # shape (n,3)

    # plot links (lines)
    xs = pts[:,0]; ys = pts[:,1]; zs = pts[:,2]
    ax.plot(xs, ys, zs, color='gray', linewidth=3, marker='o', markersize=6)

    # small frames at each joint (draw local X and Z for visual)
    if show_frames:
        for p in pts:
            # draw a small cross in X-Z plane
            ox, oy, oz = p
            # local X axis: small segment along +X
            ax.plot([ox, ox+frame_len], [oy, oy], [oz, oz], color='r')
            # local Z axis: small segment along +Z
            ax.plot([ox, ox], [oy, oy], [oz, oz+frame_len], color='b')

    # labels
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    ax.set_zlabel('Z (cm)')

# ------------------------
# Defaults (your specs)
# ------------------------
# Angles defaults (deg)
def_theta1 = 90.0
def_theta2 = 90.0
def_theta3 = 90.0
def_theta4 = 90.0

# lengths in cm (your numbers)
def_d1 = 5.0   # base/waist height
def_L2 = 15.0  # shoulder->elbow
def_L3 = 10.0  # elbow->wrist
def_L4 = 5.0   # wrist->eef

# ------------------------
# Build the UI
# ------------------------
plt.rcParams['toolbar'] = 'toolbar2'
fig = plt.figure(figsize=(12,6))
gs = gridspec.GridSpec(1, 3, width_ratios=[2.6, 0.9, 0.01], wspace=0.25)
ax3d = fig.add_subplot(gs[0], projection='3d')
ax_side_ui = fig.add_subplot(gs[1])
ax_side_ui.axis('off')

# optional reference image (if present)
ref_path = "/mnt/data/A_3D-rendered_digital_illustration_displays_a_six-.png"
if os.path.exists(ref_path):
    ax_img = fig.add_axes([0.76, 0.05, 0.12, 0.12])
    ax_img.imshow(mpimg.imread(ref_path))
    ax_img.axis('off')

# sliders positions
left = 0.74
start_y = 0.92
h = 0.055
width = 0.22

from matplotlib.widgets import Slider, Button, TextBox

theta_sliders = []
labels = ["θ1 waist", "θ2 shoulder", "θ3 elbow", "θ4 wrist-pitch"]
init_angles = [def_theta1, def_theta2, def_theta3, def_theta4]
for i, lab in enumerate(labels):
    axs = fig.add_axes([left, start_y - i*(h+0.01), width, h])
    s = Slider(axs, lab + " (deg)", -180, 180, valinit=init_angles[i], valfmt='%0.0f')
    theta_sliders.append(s)

# length sliders for d1, L2, L3, L4
len_sliders = []
len_labels = ["d1 base (cm)", "L2 shoulder (cm)", "L3 elbow (cm)", "L4 wrist (cm)"]
init_lengths = [def_d1, def_L2, def_L3, def_L4]
for j, lab in enumerate(len_labels):
    axl = fig.add_axes([left, start_y - (len(theta_sliders) + j)*(h+0.01) - 0.01, width, h])
    s = Slider(axl, lab, 1.0, 60.0, valinit=init_lengths[j], valstep=0.5)
    len_sliders.append(s)

# reset & EEF display
ax_reset = fig.add_axes([left, 0.02, 0.08, 0.04])
btn_reset = Button(ax_reset, 'Reset')
ax_text = fig.add_axes([left+0.09, 0.02, 0.13, 0.04])
txt_box = TextBox(ax_text, 'EEF (cm)', initial='')

# update function
def update(val=None):
    t1 = theta_sliders[0].val
    t2 = theta_sliders[1].val
    t3 = theta_sliders[2].val
    t4 = theta_sliders[3].val
    d1 = len_sliders[0].val
    L2 = len_sliders[1].val
    L3 = len_sliders[2].val
    L4 = len_sliders[3].val

    pts = forward_kinematics_side(t1, t2, t3, t4, d1, L2, L3, L4)
    draw_side_robot(ax3d, pts, show_frames=True, frame_len=max(1.0, min(L2, 5.0)))

    # side view orientation: show X horizontal, Z vertical (Y depth)
    # choose a view that best shows X-Z plane: azim = 90 or -90 makes Y axis point away
    ax3d.view_init(elev=0, azim=90)  # side view (XZ plane)
    total = d1 + L2 + L3 + L4
    lim = max(10.0, total * 0.7)
    ax3d.set_xlim([-lim, lim])
    ax3d.set_ylim([-lim, lim])
    ax3d.set_zlim([0, total * 1.2])

    ee = pts[-1]
    txt_box.set_val(f"{ee[0]:.1f}, {ee[1]:.1f}, {ee[2]:.1f}")
    fig.canvas.draw_idle()

for s in theta_sliders: s.on_changed(update)
for s in len_sliders: s.on_changed(update)

def on_reset(event):
    for s, v in zip(theta_sliders, init_angles):
        s.set_val(v)
    for s, v in zip(len_sliders, init_lengths):
        s.set_val(v)
    update()
btn_reset.on_clicked(on_reset)

# initial draw with your requested defaults
for i, s in enumerate(theta_sliders):
    s.set_val(init_angles[i])
for i, s in enumerate(len_sliders):
    s.set_val(init_lengths[i])
update()

plt.suptitle("Side-view 4-DOF Robotic Arm (angles in degrees, lengths in cm)")
plt.show()
