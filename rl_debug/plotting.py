import numpy as np
import matplotlib.pyplot as plt

# ==============================
# Load data
# ==============================
actions_path = "actions_log_after_constrain.npy"
actions = np.load(actions_path)

# ==============================
# Split left / right
# ==============================
left_pos = actions[:, 0:3]
left_aa  = actions[:, 3:6]

right_pos = actions[:, 7:10]
right_aa  = actions[:, 10:13]

# ==============================
# Normalize angle-axis (direction only)
# ==============================
def normalize(v):
    n = np.linalg.norm(v, axis=1, keepdims=True)
    n[n == 0] = 1.0
    return v / n

left_dir = normalize(left_aa)
right_dir = normalize(right_aa)

# ==============================
# Helper: set padded equal limits
# ==============================
def set_padded_limits(ax, points, pad=0.2):
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    center = (mins + maxs) / 2
    span = max(maxs - mins) or 1.0  # fallback if all points are identical
    margin = pad * span
    lim = span / 2 + margin
    ax.set_xlim(center[0] - lim, center[0] + lim)
    ax.set_ylim(center[1] - lim, center[1] + lim)
    ax.set_zlim(center[2] - lim, center[2] + lim)

# ==============================
# LEFT arm
# ==============================
fig_left = plt.figure()
ax_left = fig_left.add_subplot(111, projection="3d")

ax_left.scatter(
    left_pos[:, 0],
    left_pos[:, 1],
    left_pos[:, 2],
    s=15,
    alpha=0.8
)

ax_left.quiver(
    left_pos[:, 0],
    left_pos[:, 1],
    left_pos[:, 2],
    left_dir[:, 0],
    left_dir[:, 1],
    left_dir[:, 2],
    length=0.03,
    normalize=False,
    color='red'
)

# Annotate indices
for i, (x, y, z) in enumerate(left_pos):
    ax_left.text(x, y, z, str(i), fontsize=7)

ax_left.set_title("Left Arm EE Scatter + Angle-Axis Direction")
ax_left.set_xlabel("X")
ax_left.set_ylabel("Y")
ax_left.set_zlabel("Z")
set_padded_limits(ax_left, left_pos, pad=0.2)

# ==============================
# RIGHT arm
# ==============================
fig_right = plt.figure()
ax_right = fig_right.add_subplot(111, projection="3d")

ax_right.scatter(
    right_pos[:, 0],
    right_pos[:, 1],
    right_pos[:, 2],
    s=15,
    alpha=0.8
)

ax_right.quiver(
    right_pos[:, 0],
    right_pos[:, 1],
    right_pos[:, 2],
    right_dir[:, 0],
    right_dir[:, 1],
    right_dir[:, 2],
    length=0.03,
    normalize=False,
    color='red'
)

# Annotate indices
for i, (x, y, z) in enumerate(right_pos):
    ax_right.text(x, y, z, str(i), fontsize=7)

ax_right.set_title("Right Arm EE Scatter + Angle-Axis Direction")
ax_right.set_xlabel("X")
ax_right.set_ylabel("Y")
ax_right.set_zlabel("Z")
set_padded_limits(ax_right, right_pos, pad=0.2)

plt.show()