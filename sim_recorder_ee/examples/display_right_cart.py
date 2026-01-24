


def plot_3d_position_right_arm_from_file(
    filepath="/home/qtf5422/Desktop/AIRE/ibrl-docker/sim_recorder_ee/examples/plotting_values/right_cart.npy",
    refresh_dt=0.05,
):
    import numpy as np
    import matplotlib.pyplot as plt
    from itertools import product, combinations
    import time
    import os

    plt.ion()
    fig = plt.figure("Right EE Position", figsize=(12, 6))
    ax_x = fig.add_subplot(121, projection='3d')
    ax_y = fig.add_subplot(122, projection='3d')
    plt.show()

    cube_center = np.array([0.01354977, -0.00898126, 0.39927967])
    cube_size = 0.1
    r = cube_size / 2

    x = [cube_center[0] - r, cube_center[0] + r]
    y = [cube_center[1] - r, cube_center[1] + r]
    z = [cube_center[2] - r, cube_center[2] + r]
    vertices = np.array(list(product(x, y, z)))

    def plot_axes(ax, right_cart, azim=0, invert_x=False):
        ax.cla()

        ax.scatter(*right_cart, color='r', s=50, label='Right EE')

        for s, e in combinations(vertices, 2):
            if np.sum(np.abs(s - e) == cube_size) == 1:
                ax.plot3D(*zip(s, e), color="b", linewidth=2)

        ax.scatter(*cube_center, color='b', s=20, label='Cube')

        ax.set_xlim(0.1, -0.1) if invert_x else ax.set_xlim(-0.1, 0.1)
        ax.set_ylim(-0.1, 0.1)
        ax.set_zlim(0.3, 0.5)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        ax.view_init(elev=0, azim=azim)
        ax.legend()

    while True:
        if not os.path.exists(filepath):
            time.sleep(refresh_dt)
            continue

        try:
            right_cart = np.load(filepath)
            if right_cart.shape != (3,):
                continue

            plot_axes(ax_x, right_cart, azim=0, invert_x=False)
            plot_axes(ax_y, right_cart, azim=90, invert_x=True)

            fig.canvas.draw()
            fig.canvas.flush_events()

        except Exception as e:
            print("Read error:", e)

        plt.pause(refresh_dt)



plot_3d_position_right_arm_from_file()