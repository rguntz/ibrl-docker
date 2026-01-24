import numpy as np
import matplotlib.pyplot as plt
import ast
import re

def plot_trajectory_from_string(s):
    # Remove leading text
    s = s.split("trajectory", 1)[-1]

    # Replace array(...) with [...]
    s = re.sub(r'array\s*\(', '[', s)
    s = s.replace(')', ']')

    # Parse
    data = ast.literal_eval(s)

    # Convert to NumPy array
    traj = np.array(data)

    # 🔑 FIX: remove extra nesting if present
    traj = np.squeeze(traj)

    print("Trajectory shape:", traj.shape)  # debugging, optional

    # Plot only dimensions 7 to 10
    plt.figure()
    for i in range(7, 10):
        plt.plot(traj[:, i], label=f'dim {i}')

    plt.xlabel("Time step")
    plt.ylabel("Value")
    plt.legend()
    plt.grid(True)
    plt.show()



# Example usage
trajectory_string = """[array([ 0.25337864, -0.0002846 ,  0.1640473 , -0.0035994 ,  0.01430668,
       -0.00073702,  0.03999842,  0.26562026,  0.02617122,  0.14617332,
       -0.0173958 ,  0.12804015,  0.08497455,  0.03999012]), array([ 2.53318251e-01, -5.06406321e-04,  1.64417239e-01, -3.64156929e-03,
        1.46512037e-02,  3.56238263e-05,  3.99984159e-02,  3.03896911e-01,
        5.11779526e-02,  1.33894279e-01, -8.47478028e-03,  2.01916750e-01,
        8.07700710e-02,  3.99901196e-02]), array([ 0.25325703, -0.00073512,  0.16479854, -0.00366191,  0.01495756,
        0.00076216,  0.03999842,  0.3421753 ,  0.0761827 ,  0.12163061,
        0.00047084,  0.27575704,  0.07654757,  0.03999012])]
"""

plot_trajectory_from_string(trajectory_string)
