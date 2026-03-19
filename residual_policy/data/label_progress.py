import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import glob
import os 
from tqdm import tqdm

def plot_arm_data_event(parquet_path, right_arm_slice=(0,3), left_arm_slice=(18,21), 
                  thumb_index=24, action_thumb = 18, event_step=None):
    """
    Loads a parquet file containing robot arm data and plots:
    - Left arm Z position
    - Right arm Z position
    - Left hand thumb flexion
    Optionally marks a specific event step on all plots.

    Parameters
    ----------
    parquet_path : str
        Path to the parquet file.
    left_arm_slice : tuple
        Slice indices for left arm in the state array (start, end).
    right_arm_slice : tuple
        Slice indices for right arm in the state array (start, end).
    thumb_index : int
        Column index for the left hand thumb flexion.
    event_step : int or None
        Step index to mark with a vertical line. If None, no line is plotted.
    """
    # Load parquet
    df = pd.read_parquet(parquet_path)
    state = np.stack(df["observation.state"].to_numpy()).astype(np.float32)

    action = np.stack(df["action"].to_numpy()).astype(np.float32)
    thumb_action_left = action[:, action_thumb]
    
    # Split arms
    state_left = state[:, left_arm_slice[0]:left_arm_slice[1]]
    state_right = state[:, right_arm_slice[0]:right_arm_slice[1]]
    
    # Z positions
    z_left = state_left[:, 2]
    z_right = state_right[:, 2]
    
    # Thumb flexion
    thumb_flex_left = state[:, thumb_index]
    
    # Step indices
    steps = np.arange(len(z_left))
    
    # Plot
    plt.figure(figsize=(10, 8))
    
    plt.subplot(3, 1, 1)
    plt.plot(steps, z_left, label="Left Z")
    if event_step is not None:
        plt.axvline(event_step, color='red', linestyle='--', label='Event')
    plt.title("Left Arm Z Position")
    plt.xlabel("Step Index")
    plt.ylabel("Z")
    plt.legend()
    
    plt.subplot(3, 1, 2)
    plt.plot(steps, z_right, label="Right Z", color='orange')
    if event_step is not None:
        plt.axvline(event_step, color='red', linestyle='--')
    plt.title("Right Arm Z Position")
    plt.xlabel("Step Index")
    plt.ylabel("Z")
    plt.legend()
    
    plt.subplot(3, 1, 3)
    plt.plot(steps, thumb_flex_left, label="Thumb Flexion", color='green')
    plt.plot(steps, thumb_flex_left, label="Thumb Flexion (state)", color='green')
    plt.plot(steps, thumb_action_left, label=f"Thumb Action (action[{action_thumb}])", linestyle="--")
    if event_step is not None:
        plt.axvline(event_step, color='red', linestyle='--')
    plt.title("Left Hand Thumb Flexion")
    plt.xlabel("Step Index")
    plt.ylabel("Flexion")
    plt.legend()
    
    plt.tight_layout()
    plt.show()

def detect_first_z_rise(parquet_path, arm_slice=(18,21), z_threshold=1.05, rise_steps=4):
    """
    Detect the first 'valley-to-rise' event in the Z position of an arm.

    Parameters
    ----------
    parquet_path : str
        Path to the parquet file containing the state.
    arm_slice : tuple
        Slice of the state array corresponding to the arm (start, end).
    z_threshold : float
        Threshold below which the Z value is considered 'low'.
    rise_steps : int
        Number of consecutive steps with increasing Z to confirm the rise.

    Returns
    -------
    event_step : int or None
        The step index of the first confirmed Z rise. None if no event is detected.
    """
    # Load parquet
    df = pd.read_parquet(parquet_path)
    state = np.stack(df["observation.state"].to_numpy()).astype(np.float32)
    
    # Extract arm and Z position
    state_arm = state[:, arm_slice[0]:arm_slice[1]]
    z = state_arm[:, 2]
    
    recording = False  # Are we below threshold?
    
    for i in range(len(z) - rise_steps):
        if not recording:
            if z[i] < z_threshold:
                recording = True  # Start monitoring the rise
        else:
            # Check if Z increases for the next 'rise_steps'
            increasing = all(z[i + j + 1] > z[i + j] for j in range(rise_steps))
            if increasing:
                return i  # Event detected at this step
    
    # No event found
    return None

def detect_thumb_decrease_event(parquet_path, thumb_index=18, step_gap=5, drop_threshold=0.8):
    """
    Detect the first event where the thumb value drops by at least a given
    threshold within a fixed number of steps.

    Parameters
    ----------
    parquet_path : str
        Path to the parquet file containing the state.
    thumb_index : int
        Column index of the left thumb flexion in the action array.
    step_gap : int
        Number of steps between the current step and the comparison step.
    drop_threshold : float
        Minimum decrease required to detect the event.

    Returns
    -------
    event_step : int or None
        Step index where the decrease event starts. None if not found.
    """

    # Load parquet and extract state
    df = pd.read_parquet(parquet_path)
    action = np.stack(df["action"].to_numpy()).astype(np.float32)

    # Extract thumb flexion
    thumb = action[:, thumb_index]

    if len(thumb) <= step_gap:
        return None  # Not enough data

    # Detect first drop of at least `drop_threshold` within `step_gap`
    for t in range(len(thumb) - step_gap):
        if thumb[t + step_gap] <= thumb[t] - drop_threshold:
            return t

    return None

def add_progress_to_parquet(parquet_path):
    """
    Adds a 'progress' column to a parquet file:
    - 0 until Z-rise event (point 1)
    - 1 from Z-rise event to thumb decrease event (point 2)
    - 2 from thumb decrease event until the end

    Raises an error if either event cannot be found.

    Parameters
    ----------
    parquet_path : str
        Path to the parquet file.
    left_arm_slice : tuple
        Slice of the left arm in the state array (start, end).
    z_threshold : float
        Threshold for detecting Z-rise.
    rise_steps : int
        Steps for detecting Z-rise.
    thumb_index : int
        Column index for left thumb flexion.
    decrease_steps : int
        Steps for detecting thumb decrease.
    """
    # Detect first Z-rise
    df = pd.read_parquet(parquet_path)
    state = np.stack(df["observation.state"].to_numpy()).astype(np.float32)
    
    # Detect first Z-rise
    point1 = detect_first_z_rise(parquet_path=parquet_path)
    if point1 is None:
        raise RuntimeError(f"Z-rise event (point 1) not found in the episode. {parquet_path}")
    
    # Thumb decrease
    point2 = detect_thumb_decrease_event(parquet_path=parquet_path)
    if point2 is None:
        raise RuntimeError(f"Thumb decrease event (point 2) not found in the episode. {parquet_path}")
    
    # Create progress column
    progress = np.zeros(len(state), dtype=int)
    progress[point1:point2] = 1
    progress[point2:] = 2
    
    # Add to DataFrame and overwrite parquet
    df["progress"] = progress
    df.to_parquet(parquet_path, index=False)
    
    return point1, point2  # Optional: return the detected points for reference


def add_regression_to_parquet(parquet_path):
    """
    Adds a 'regression' column to a parquet file based on the 'progress' column.
    Each stage transition (0->1, 1->2, 2->end) gets a linear increase from 0 to 1.

    Parameters
    ----------
    parquet_path : str
        Path to the parquet file containing a 'progress' column.
    """
    df = pd.read_parquet(parquet_path)
    
    if "progress" not in df.columns:
        raise RuntimeError(f"'progress' column not found in {parquet_path}. Run add_progress_to_parquet first.")
    
    progress = df["progress"].to_numpy()
    regression = np.zeros(len(progress), dtype=np.float32)
    
    # Define the stage transitions
    transitions = [0, 1, 2]
    
    for i, stage in enumerate(transitions):
        # Start index: first occurrence of this stage
        start_idx = np.where(progress == stage)[0][0]
        # End index: first occurrence of next stage, or end of array
        if i < len(transitions) - 1:
            next_stage = transitions[i + 1]
            next_idx_arr = np.where(progress == next_stage)[0]
            end_idx = next_idx_arr[0] if len(next_idx_arr) > 0 else len(progress)
        else:
            end_idx = len(progress)
        
        # Linear regression 0->1 for this range
        regression[start_idx:end_idx] = np.linspace(0, 1, end_idx - start_idx, endpoint=True)
    
    df["regression"] = regression
    df.to_parquet(parquet_path, index=False)
    

def plot_arm_data_with_progress(parquet_path, right_arm_slice=(0,3), left_arm_slice=(18,21),
                                thumb_index=24):
    """
    Plots left/right Z positions, thumb flexion, and progress column.

    Parameters
    ----------
    parquet_path : str
        Path to the parquet file.
    left_arm_slice : tuple
        Slice of the left arm in the state array.
    right_arm_slice : tuple
        Slice of the right arm in the state array.
    thumb_index : int
        Column index for left thumb flexion.
    """
    df = pd.read_parquet(parquet_path)
    state = np.stack(df["observation.state"].to_numpy()).astype(np.float32)
    
    state_left = state[:, left_arm_slice[0]:left_arm_slice[1]]
    state_right = state[:, right_arm_slice[0]:right_arm_slice[1]]
    
    z_left = state_left[:, 2]
    z_right = state_right[:, 2]
    thumb_flex_left = state[:, thumb_index]
    progress = df["progress"].to_numpy()
    regression = df["regression"].to_numpy()
    
    steps = np.arange(len(z_left))
    
    plt.figure(figsize=(12, 10))
    
    plt.subplot(4, 1, 1)
    plt.plot(steps, z_left, label="Left Z")
    plt.title("Left Arm Z Position")
    plt.xlabel("Step Index")
    plt.ylabel("Z")
    plt.legend()
    
    plt.subplot(4, 1, 2)
    plt.plot(steps, z_right, label="Right Z", color='orange')
    plt.title("Right Arm Z Position")
    plt.xlabel("Step Index")
    plt.ylabel("Z")
    plt.legend()
    
    plt.subplot(4, 1, 3)
    plt.plot(steps, thumb_flex_left, label="Thumb Flexion", color='green')
    plt.title("Left Hand Thumb Flexion")
    plt.xlabel("Step Index")
    plt.ylabel("Flexion")
    plt.legend()
    
    plt.subplot(4, 1, 4)
    plt.plot(steps, progress, label="Progress", color='purple')
    plt.plot(steps, regression, label="regression", color='red')
    plt.title("Task Progress")
    plt.xlabel("Step Index")
    plt.ylabel("Progress")
    plt.yticks([0, 1, 2])
    plt.legend()
    
    plt.tight_layout()
    plt.show()

def process_parquet_folders(folders):
    """
    Apply add_progress_to_parquet to all parquet files in the given folders with a progress bar.

    Parameters
    ----------
    folders : list of str
        List of folder paths containing parquet files.
    left_arm_slice : tuple
        Slice of the left arm in the state array.
    z_threshold : float
        Threshold for detecting Z-rise.
    rise_steps : int
        Steps for detecting Z-rise.
    thumb_index : int
        Column index for left thumb flexion.
    decrease_steps : int
        Steps for detecting thumb decrease.
    """
    # Collect all parquet files from all folders
    all_files = []
    for folder in folders:
        files = glob.glob(os.path.join(folder, "*.parquet"))
        all_files.extend(files)
    
    if not all_files:
        print("No parquet files found in the provided folders.")
        return
    
    # Process all files with tqdm
    for parquet_path in tqdm(all_files, desc="Processing parquet files"):
        # Apply the add_progress_to_parquet function
        point1, point2 = add_progress_to_parquet(parquet_path)

        if point1 > point2 : 
            raise RuntimeError(f"point 2 inferior to point 1. {parquet_path}")
        
        add_regression_to_parquet(parquet_path)

def plot_arm_data_thumb(parquet_path, right_arm_slice=(0,3), left_arm_slice=(18,21), thumb_index=24, action_thumb = 18):
    """
    Loads a parquet file containing robot arm data and plots:
    - Left arm Z position
    - Right arm Z position
    - Left hand thumb flexion
    - Left hand thumb action (from df["action"], column 18)
    """

    # Load parquet file
    df = pd.read_parquet(parquet_path)

    # Extract state and convert to float32
    state = np.stack(df["observation.state"].to_numpy()).astype(np.float32)

    # Extract actions
    action = np.stack(df["action"].to_numpy()).astype(np.float32)

    # Split arms
    state_left = state[:, left_arm_slice[0]:left_arm_slice[1]]
    state_right = state[:, right_arm_slice[0]:right_arm_slice[1]]

    # Z positions
    z_left = state_left[:, 2]
    z_right = state_right[:, 2]

    # Thumb flexion (state)
    thumb_flex_left = state[:, thumb_index]

    # Thumb action (column action_thumb in action)
    thumb_action_left = action[:, action_thumb]

    # Step indices
    steps = np.arange(len(z_left))

    # Plot
    plt.figure(figsize=(10, 8))

    plt.subplot(3, 1, 1)
    plt.plot(steps, z_left, label="Left Z")
    plt.title("Left Arm Z Position")
    plt.xlabel("Step Index")
    plt.ylabel("Z")

    plt.subplot(3, 1, 2)
    plt.plot(steps, z_right, label="Right Z", color='orange')
    plt.title("Right Arm Z Position")
    plt.xlabel("Step Index")
    plt.ylabel("Z")

    plt.subplot(3, 1, 3)
    plt.plot(steps, thumb_flex_left, label="Thumb Flexion (state)", color='green')
    plt.plot(steps, thumb_action_left, label=f"Thumb Action (action[{action_thumb}])", linestyle="--")
    plt.title("Left Hand Thumb Flexion")
    plt.xlabel("Step Index")
    plt.ylabel("Value")
    plt.legend()

    plt.tight_layout()
    plt.show()

def plot_parquet_folders(folders):
    """
    Apply add_progress_to_parquet to all parquet files in the given folders with a progress bar.

    Parameters
    ----------
    folders : list of str
        List of folder paths containing parquet files.
    left_arm_slice : tuple
        Slice of the left arm in the state array.
    z_threshold : float
        Threshold for detecting Z-rise.
    rise_steps : int
        Steps for detecting Z-rise.
    thumb_index : int
        Column index for left thumb flexion.
    decrease_steps : int
        Steps for detecting thumb decrease.
    """
    # Collect all parquet files from all folders
    all_files = []
    for folder in folders:
        files = glob.glob(os.path.join(folder, "*.parquet"))
        all_files.extend(files)
    
    if not all_files:
        print("No parquet files found in the provided folders.")
        return
    
    # Process all files with tqdm
    for parquet_path in tqdm(all_files, desc="Processing parquet files"):
        # Apply the add_progress_to_parquet function
        plot_arm_data_with_progress(parquet_path=parquet_path)


parquet_path = "/home/qtf5422/.cache/huggingface/lerobot/ankile/dexmg-two-arm-coffee/data/chunk-000/episode_000898.parquet"
folders = [
    "/home/qtf5422/.cache/huggingface/lerobot/ankile/dexmg-two-arm-coffee/data/chunk-000",
    "/home/qtf5422/.cache/huggingface/lerobot/ankile/dexmg-two-arm-coffee/data/chunk-001"
]



plot_arm_data_thumb(parquet_path=parquet_path)

# Detect first Z-rise event
event_step = detect_first_z_rise(parquet_path)
print("First Z-rise event at step:", event_step)

# Plot and mark the event
plot_arm_data_event(parquet_path, event_step=event_step)


###################################################################

thumb_opening = detect_thumb_decrease_event(parquet_path)

print("thumb_opening : ", thumb_opening)

# Plot and mark the event
plot_arm_data_event(parquet_path, event_step=thumb_opening)

###################################################################

# Add progress column
point1, point2 = add_progress_to_parquet(parquet_path)
print("Detected points:", point1, point2)
add_regression_to_parquet(parquet_path)

# Plot with progress
plot_arm_data_with_progress(parquet_path)

###################################################################

process_parquet_folders(folders)


###################################################################

plot_parquet_folders(folders=folders)

###################################################################


