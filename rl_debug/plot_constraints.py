import numpy as np
import matplotlib.pyplot as plt

# Define the constraints function
def combined_constraints(p):
    x, y, z = p
    constraints = []

    # Sphere constraint
    center = np.array([0.0, 0.0, 0.0])
    radius = 1.3 / 2.0
    constraints.append(radius - np.linalg.norm(p - center))

    # 2D boundary constraints in x-y
    # Remove inner circle and replace with x >= 0
    constraints.append(x - (0.2))                  # x >= 0.2
    constraints.append(0.8 - np.sqrt(x**2 + y**2))  # outer circle r <= 0.8
    constraints.append(2.5*x + 0.2 - y)    # y <= 2.5*x + 0.2
    constraints.append(y - (-2.5*x - 0.2)) # y >= -2.5*x - 0.2
    constraints.append(0.3 - y)            # y <= 0.4
    constraints.append(y - (-0.3))         # y >= -0.4

    # z constraint
    constraints.append(z - (-0.02))        # z >= -0.02

    return constraints

# Create a grid in x, y, z
x_vals = np.linspace(-10, 10, 50)
y_vals = np.linspace(-0.5, 0.5, 50)
z_vals = np.linspace(-0.02, 0.7, 50)

X, Y, Z = np.meshgrid(x_vals, y_vals, z_vals)
feasible = np.zeros_like(X, dtype=bool)

# Check which points satisfy all constraints
for i in range(X.shape[0]):
    for j in range(X.shape[1]):
        for k in range(X.shape[2]):
            p = np.array([X[i,j,k], Y[i,j,k], Z[i,j,k]])
            if all(c >= 0 for c in combined_constraints(p)):
                feasible[i,j,k] = True

# Plotting
fig = plt.figure(figsize=(10,8))
ax = fig.add_subplot(111, projection='3d')

# Scatter plot of feasible points
ax.scatter(X[feasible], Y[feasible], Z[feasible], color='cyan', alpha=0.3, s=5)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Feasible Region Defined by Constraints')
plt.show()
