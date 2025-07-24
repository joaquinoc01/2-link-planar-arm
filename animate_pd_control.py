import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os

# Load simulation data
data = np.genfromtxt("build/control_simulation.csv", delimiter=",", names=True)

l1, l2 = 1.0, 1.0  # link lengths

def forward_kinematics(q):
    theta1, theta2 = q
    x1 = l1 * np.cos(theta1)
    y1 = l1 * np.sin(theta1)
    x2 = x1 + l2 * np.cos(theta1 + theta2)
    y2 = y1 + l2 * np.sin(theta1 + theta2)
    return np.array([[0, x1, x2], [0, y1, y2]])

# Set up the figure and axis
fig, ax = plt.subplots()
ax.set_xlim(-2.1, 2.1)
ax.set_ylim(-2.1, 2.1)
ax.set_aspect('equal')
ax.grid()

# Plot elements
arm_line, = ax.plot([], [], 'o-', lw=4, label="Actual")
des_line, = ax.plot([], [], 'o--', color='gray', alpha=0.6, label="Desired")

def init():
    arm_line.set_data([], [])
    des_line.set_data([], [])
    return arm_line, des_line

def update(i):
    q = [data["q1"][i], data["q2"][i]]
    q_des = [data["q1_des"][i], data["q2_des"][i]]

    pts = forward_kinematics(q)
    pts_des = forward_kinematics(q_des)

    arm_line.set_data(pts[0], pts[1])
    des_line.set_data(pts_des[0], pts_des[1])
    return arm_line, des_line

ani = animation.FuncAnimation(
    fig, update, frames=len(data), init_func=init,
    interval=20, blit=True
)

# Title and legend
plt.legend()
plt.title("2-Link Arm: PD Control Tracking")

# Save animation as GIF
os.makedirs("media", exist_ok=True)
ani.save("media/arm_control.gif", writer="pillow", fps=50)
print("✅ Animation saved to media/arm_control.gif")
