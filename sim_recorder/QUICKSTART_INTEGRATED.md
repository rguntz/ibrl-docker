# QUICKSTART: Integrated Teleop Recording

**🎯 ONE SCRIPT, NO SEPARATE PROCESSES**

This is the simplest way to record demos - everything runs in one Python script:
- MuJoCo simulation
- Real robot teleoperation  
- Camera capture (4 cameras: high, low, left_wrist, right_wrist)
- Robot state recording (qpos, qvel)
- SERL-compatible data format

## Quick Start

```bash
# 1. Run the integrated recorder
cd sim_recorder/examples
python integrated_teleop_recorder.py --duration 60 --auto-record

# That's it! Data saved to: recorded_episodes/episode_<timestamp>/
```

## What Gets Recorded

Each episode folder contains:

```
recorded_episodes/episode_20240115_143022/
├── observations.npz         # All observations
│   ├── cam_high: (T, 128, 128, 3)
│   ├── cam_low: (T, 128, 128, 3)
│   ├── cam_left_wrist: (T, 128, 128, 3)
│   ├── cam_right_wrist: (T, 128, 128, 3)
│   ├── qpos: (T, 8)        # Joint positions
│   └── qvel: (T, 8)        # Joint velocities
├── actions.npy              # Actions from leader (T, 7)
└── meta.json                # Metadata
```

## Usage Options

```bash
# Basic: Record for 30 seconds
python integrated_teleop_recorder.py --duration 30 --auto-record

# Custom save location
python integrated_teleop_recorder.py --save-dir /path/to/my_demos --auto-record

# Higher control frequency
python integrated_teleop_recorder.py --control-freq 30.0 --auto-record

# Skip home position
python integrated_teleop_recorder.py --no-home --auto-record

# Manual recording control (press SPACE to start/stop)
python integrated_teleop_recorder.py --duration 120
```

## Reading Recorded Data (for training)

```python
import numpy as np
import json

# Load episode
episode_dir = "recorded_episodes/episode_20240115_143022"

# Load observations
obs = np.load(f"{episode_dir}/observations.npz")
cam_high = obs['cam_high']        # (T, 128, 128, 3)
cam_low = obs['cam_low']          # (T, 128, 128, 3)
cam_left = obs['cam_left_wrist']  # (T, 128, 128, 3)
cam_right = obs['cam_right_wrist'] # (T, 128, 128, 3)
qpos = obs['qpos']                # (T, 8)
qvel = obs['qvel']                # (T, 8)

# Load actions
actions = np.load(f"{episode_dir}/actions.npy")  # (T, 7)

# Load metadata
with open(f"{episode_dir}/meta.json") as f:
    meta = json.load(f)

print(f"Episode: {meta['episode_name']}")
print(f"Steps: {meta['num_steps']}")
print(f"Duration: {meta['duration']:.1f}s")
```

## How It Works

1. **Connects to real leader robot** (192.168.1.2) via Trossen SDK
2. **Loads MuJoCo sim** from `trossen_sim/trossen_sim/envs/xmls/trossen_ai_scene_joint.xml`
3. **Control loop** (20Hz default):
   - Read leader joint positions → actions
   - Apply actions to sim via `mj_data.ctrl`
   - Step physics with `mujoco.mj_step()`
   - If recording:
     - Capture 4 camera images using `mujoco.Renderer`
     - Get robot state from `mj_data.qpos`, `mj_data.qvel`
     - Append to episode buffer
4. **Save on stop** → NPZ format in `recorded_episodes/`

## Comparison to Original Architecture

**Before (3 processes):**
```
Terminal 1: python sim_recorder/server/app.py
Terminal 2: python sim_recorder/examples/sim_camera_pusher.py
Terminal 3: python sim_recorder/examples/teleop_publisher.py
```

**Now (1 process):**
```
python sim_recorder/examples/integrated_teleop_recorder.py --auto-record
```

## Configuration

Edit in the script if needed:

```python
# Camera configuration
camera_names = ['cam_high', 'cam_low', 'cam_left_wrist', 'cam_right_wrist']
camera_resolution = (128, 128)  # (H, W)

# Control frequency
control_freq = 20.0  # Hz

# Robot model
model = 'wxai_v0'
```

## Troubleshooting

**Camera not found:**
- Check camera names in XML match exactly
- Verify `trossen_ai_scene_joint.xml` path

**Leader connection failed:**
- Verify robot IP: `ping 192.168.1.2`
- Check robot is powered on
- Use `--leader-ip` flag if different IP

**Slow recording:**
- Reduce `control_freq` (default 20Hz)
- Reduce `camera_resolution` to (64, 64)
- Record fewer cameras

**Memory issues:**
- Don't record for too long (>5 min)
- Check disk space in `save_dir`

## Convert to SERL RLDS Format

After recording with integrated script, convert to RLDS:

```python
from sim_recorder.server.exporters import RLDSExporter
import numpy as np

# Load recorded episode
episode_dir = "recorded_episodes/episode_20240115_143022"
obs = np.load(f"{episode_dir}/observations.npz")
actions = np.load(f"{episode_dir}/actions.npy")

# Prepare data for exporter
episodes = [{
    'observations': dict(obs),
    'actions': actions,
    'rewards': np.ones(len(actions)),  # Dummy rewards
    'dones': np.zeros(len(actions), dtype=bool)
}]

# Export to RLDS
exporter = RLDSExporter(
    dataset_name='my_task',
    output_dir='rlds_datasets'
)
exporter.export(episodes)

print("✓ RLDS dataset created at: rlds_datasets/my_task")
```

## Next Steps

1. **Collect multiple demos:** Run script multiple times with `--auto-record`
2. **Train policy:** Use SERL training scripts with your recorded data
3. **Evaluate:** Load policy and run in sim
