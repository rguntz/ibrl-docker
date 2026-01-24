import random
import time
import os

# Allowed angles
angles = [0, 45, 90, 135]

while True:
    # Clear the terminal
    os.system('cls' if os.name == 'nt' else 'clear')

    # Generate values
    num_1_10 = random.randint(1, 10)
    num_1_7 = random.randint(1, 7)
    angle = random.choice(angles)

    # Print values
    print("Number horizontal:", num_1_10)
    print("Number vertical:", num_1_7)
    print("Angle:", angle)

    # Wait 5 seconds
    time.sleep(5)
