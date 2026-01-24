import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from scipy.optimize import minimize

def add_circles(ax):
    ax.add_patch(Circle((0, 0), 0.3, fill=False, linewidth=2))
    ax.add_patch(Circle((0, 0), 0.8, fill=False, linewidth=2))

def add_horizontal_lines(ax):
    ax.axhline(y=0.4, linewidth=2)
    ax.axhline(y=-0.4, linewidth=2)

def add_equation_line_1(ax):
    x = np.linspace(-1, 1, 500)
    ax.plot(x, -2.5*x - 0.2, linewidth=2)

def add_equation_line_2(ax):
    x = np.linspace(-1, 1, 500)
    ax.plot(x, 2.5*x + 0.2, linewidth=2)

class BoundaryChecker:
    def __init__(self):
        self.inner_radius = 0.3
        self.outer_radius = 0.8

    def inside(self, x, y):
        if y > 2.5 * x + 0.2:
            return False
        if y < -2.5 * x - 0.2:
            return False
        if y > 0.4 or y < -0.4:
            return False

        r = np.sqrt(x**2 + y**2)
        if r < self.inner_radius or r > self.outer_radius:
            return False

        return True

def sample_and_plot_valid_points(ax, boundary, n_samples=400):
    xs = np.linspace(-2, 2, n_samples)
    ys = np.linspace(-2, 2, n_samples)

    valid_x, valid_y = [], []

    for x in xs:
        for y in ys:
            if boundary.inside(x, y):
                valid_x.append(x)
                valid_y.append(y)

    ax.scatter(valid_x, valid_y, s=5)

def load_and_plot_positions(filepath="total_position.npy"):
    data = np.load(filepath)
    right_positions = data[:, 3:5]

    fig, ax = plt.subplots()
    ax.scatter(right_positions[:, 0], right_positions[:, 1])

    add_circles(ax)
    add_horizontal_lines(ax)
    add_equation_line_1(ax)
    add_equation_line_2(ax)

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title("Right Position (x-y)")
    ax.axis("equal")
    ax.grid(True)

    plt.show()


def distance_to_target(p, target):
    x, y = p
    return (x - target[0])**2 + (y - target[1])**2

def boundary_constraints(p):
    x, y = p
    constraints = []
    r = np.sqrt(x**2 + y**2)
    
    # circle constraints
    constraints.append(r - 0.3)         # r >= inner_radius
    constraints.append(0.8 - r)         # r <= outer_radius
    
    # line constraints
    constraints.append(2.5*x + 0.2 - y)      # y <= 2.5 x + 0.2
    constraints.append(y - (-2.5*x - 0.2))   # y >= -2.5 x - 0.2
    
    # horizontal limits
    constraints.append(0.4 - y)         # y <= 0.4
    constraints.append(y - (-0.4))      # y >= -0.4
    return constraints

def closest_point_inside_boundary(target):
    # Define constraints in scipy format
    cons = [{'type': 'ineq', 'fun': lambda p, i=i: boundary_constraints(p)[i]} for i in range(6)]

    # Use a guaranteed feasible initial guess
    x0 = [0.5, 0]

    # Solve
    res = minimize(distance_to_target, x0=x0, args=(target,), constraints=cons)

    if res.success:
        return res.x
    else:
        raise ValueError("Optimization failed to find a point inside the boundary.")


def test_boundary_projection(n_tests=100):
    boundary = BoundaryChecker()

    for i in range(n_tests):
        # Sample a random target point in [-1, 1] x [-1, 1]
        x_target = np.random.uniform(-1, 1)
        y_target = np.random.uniform(-1, 1)
        target = (x_target, y_target)

        # Find the closest point inside the boundary
        x_closest, y_closest = closest_point_inside_boundary(target)

        # Plot everything
        fig, ax = plt.subplots()

        # Plot the valid points cloud
        sample_and_plot_valid_points(ax, boundary, n_samples=200)

        # Plot the target point in red
        ax.scatter(x_target, y_target, color='red', s=50, label='Target')

        # Plot the optimized point in green
        ax.scatter(x_closest, y_closest, color='green', s=50, label='Closest inside boundary')

        # Plot boundaries
        add_circles(ax)
        add_horizontal_lines(ax)
        add_equation_line_1(ax)
        add_equation_line_2(ax)

        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_title(f"Test {i+1}/{n_tests}")
        ax.axis("equal")
        ax.grid(True)
        ax.legend()

        # Show the figure and wait until user closes it
        plt.show()


# Run 100 interactive tests
test_boundary_projection(100)


load_and_plot_positions("total_position.npy")
