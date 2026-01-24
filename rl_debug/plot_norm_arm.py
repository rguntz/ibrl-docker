import numpy as np
import matplotlib.pyplot as plt
import os

def plot_speed_norm(filename="/home/qtf5422/Desktop/AIRE/ibrl-docker/rl_debug/speed_norms.csv"):
    if not os.path.isfile(filename):
        raise FileNotFoundError(f"File not found: {filename}")

    # Load data (skip header)
    speed_norms = np.loadtxt(filename, skiprows=1)

    if speed_norms.size == 0:
        print("File is empty, nothing to plot.")
        return

    plt.figure()
    plt.plot(speed_norms)
    plt.xlabel("Time step")
    plt.ylabel("Speed norm")
    plt.title("Speed norm over time")
    plt.grid(True)
    plt.show()


plot_speed_norm()