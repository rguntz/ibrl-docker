import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize
from matplotlib.patches import Circle
from mpl_toolkits.mplot3d import art3d

def add_2d_boundaries(ax):
    """Plot the old 2D boundaries on z=0 plane for visualization."""
    # Circles
    c1 = Circle((0, 0), 0.3, fill=False, linewidth=2, color='black')
    c2 = Circle((0, 0), 0.8, fill=False, linewidth=2, color='black')
    ax.add_patch(c1)
    ax.add_patch(c2)
    art3d.pathpatch_2d_to_3d(c1, z=0, zdir="z")
    art3d.pathpatch_2d_to_3d(c2, z=0, zdir="z")

    # Horizontal lines
    ax.plot([-1, 1], [0.4, 0.4], zs=0, color='black', linewidth=2)
    ax.plot([-1, 1], [-0.4, -0.4], zs=0, color='black', linewidth=2)

    # Sloped lines
    x = np.linspace(-1, 1, 500)
    ax.plot(x, -2.5*x - 0.2, zs=0, color='black', linewidth=2)
    ax.plot(x, 2.5*x + 0.2, zs=0, color='black', linewidth=2)

def plot_sphere(ax, center=np.array([0,0,0]), diameter=1.3, color='lightblue', alpha=0.2):
    """Plot a semi-transparent sphere representing the 3D boundary."""
    radius = diameter / 2.0
    u, v = np.mgrid[0:2*np.pi:50j, 0:np.pi:25j]
    x = center[0] + radius*np.cos(u)*np.sin(v)
    y = center[1] + radius*np.sin(u)*np.sin(v)
    z = center[2] + radius*np.cos(v)
    ax.plot_surface(x, y, z, color=color, alpha=alpha, edgecolor='none')

def distance_to_target_3d(p, target):
    return np.sum((p - target)**2)

def combined_constraints(p):
    x, y, z = p
    constraints = []

    # Sphere constraint
    center = np.array([0.0, 0.0, 0.0])
    radius = 1.3 / 2.0
    constraints.append(radius - np.linalg.norm(p - center))

    # 2D boundary constraints
    r = np.sqrt(x**2 + y**2)
    constraints.append(r - 0.3)
    constraints.append(0.8 - r)
    constraints.append(2.5*x + 0.2 - y)
    constraints.append(y - (-2.5*x - 0.2))
    constraints.append(0.4 - y)
    constraints.append(y - (-0.4))

    # NEW: z >= 0
    constraints.append(z - 0.0)

    return constraints

def closest_point_3d(target):
    num_constraints = len(combined_constraints(np.zeros(3)))  # automatically 8 now
    cons = [{'type': 'ineq', 'fun': lambda p, i=i: combined_constraints(p)[i]} for i in range(num_constraints)]
    x0 = np.array([0.5, 0.0, 0.0])  # feasible starting point
    res = minimize(distance_to_target_3d, x0=x0, args=(target,), constraints=cons)
    if res.success:
        # Clip z to be >= 0
        closest_point = res.x.copy()
        print("z value : ", closest_point[2])
        closest_point[2] = max(closest_point[2], 0.0)
        return closest_point
    else:
        raise ValueError("Optimization failed!")

def test_combined_boundary_projection(n_tests=100):
    for i in range(n_tests):
        target = np.random.uniform(-1, 1, size=3)
        closest = closest_point_3d(target)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')  

        # Plot 3D sphere boundary
        plot_sphere(ax)

        # Plot 2D boundaries at z=0
        add_2d_boundaries(ax)

        # Plot target in red
        ax.scatter(*target, color='red', s=50, label='Target')

        # Plot closest point in green
        print("closest : ", closest)
        ax.scatter(*closest, color='green', s=50, label='Closest feasible')

        ax.set_xlim([-1.5, 1.5])
        ax.set_ylim([-1.5, 1.5])
        ax.set_zlim([-1.5, 1.5])
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title(f"Test {i+1}/{n_tests}")
        ax.legend()
        ax.grid(True)

        plt.show()
        plt.close(fig)  # <-- prevent memory warning

# Run tests
test_combined_boundary_projection(100)
