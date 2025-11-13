# Sim Recorder

**Self-contained** data collection for SERL robot learning: Record demos from **MuJoCo simulation** controlled by **real robot teleoperation**.

## 🚀 Quick Start (2 Terminals Required)

### Terminal 1: Start Web UI Server
```bash
cd sim_recorder/server
python app.py
# Server starts at http://localhost:5000
```

### Terminal 2: Run Teleoperation
```bash
cd sim_recorder/examples
python teleop_with_server.py --no-visualize --leader-left-ip 192.168.1.2 --leader-right-ip 192.168.1.3
# Connects to robots and streams cameras to web UI
```

### Open Web UI
- **Browser**: `http://localhost:5000`
- **Live camera feeds**: 4 real-time camera streams
- **Recording controls**: START/STOP buttons
- **Status**: Recording progress and episode info

## 🎯 Usage Options

**With MuJoCo Visual Viewer:**
```bash
# Terminal 2 (after starting server in Terminal 1)
python teleop_with_server.py --leader-left-ip 192.168.1.2 --leader-right-ip 192.168.1.3
# Opens MuJoCo viewer window + web UI
```

**Headless (Web UI Only):**
```bash
# Terminal 2 (after starting server in Terminal 1)  
python teleop_with_server.py --no-visualize --leader-left-ip 192.168.1.2 --leader-right-ip 192.168.1.3
# No MuJoCo viewer, web UI only (recommended for data collection)
```

**What it does:**
1. Connects to real leader robots (dual robot support)
2. Loads MuJoCo simulation (self-contained XML + meshes)
3. [Optional] Shows MuJoCo viewer window
4. **Always** streams 4 cameras to web UI in real-time
5. Records: camera views + robot states + actions
6. Saves in SERL-compatible format

## 📦 Self-Contained Package

This `sim_recorder/` folder is **completely independent** - it includes:

- ✅ **MuJoCo XML files**: `assets/trossen_ai_scene_joint.xml`, `assets/trossen_ai_joint.xml`
- ✅ **3D meshes & textures**: `assets/meshes/` (all STL files + PNG textures)
- ✅ **Web UI**: Complete Flask server + HTML/JS/CSS interface
- ✅ **Recording logic**: Camera capture, state recording, data export
- ✅ **Dual robot support**: Control two sim robots with two real leaders

**External dependencies** (install separately):
- `mujoco` (MuJoCo physics engine)
- `trossen_arm` (real robot interface)
- Python packages in `requirements.txt`

No other SERL folders required!

## How It Works

```
Real Leader Robot (192.168.1.2)
       ↓ (read joint positions)
   [Actions: 7D joint commands]
       ↓
MuJoCo Simulation
   • Apply actions to sim robot
   • Step physics
   • Render 4 cameras (128x128)
   • Get robot state (qpos, qvel)
       ↓
Save Episode Data
   • observations.npz (cameras + states)
   • actions.npy (leader commands)
   • meta.json (episode info)
```

## What Gets Recorded

Each episode folder contains:

```
recorded_episodes/episode_20240115_143022/
├── observations.npz         # All observations
│   ├── cam_high: (T, 128, 128, 3)      # High camera view
│   ├── cam_low: (T, 128, 128, 3)       # Low camera view  
│   ├── cam_left_wrist: (T, 128, 128, 3)  # Left wrist camera
│   ├── cam_right_wrist: (T, 128, 128, 3) # Right wrist camera
│   ├── qpos: (T, 8)         # Joint positions (6 arm + 2 gripper)
│   └── qvel: (T, 8)         # Joint velocities
├── actions.npy              # Actions from leader robot (T, 7)
└── meta.json                # Metadata (num_steps, duration, FPS, etc.)
```

Where `T` = number of timesteps in the episode.

## Usage Examples

### Basic Recording

```bash
# Terminal 1: Start server
cd sim_recorder/server
python app.py

# Terminal 2: Run teleop (headless recommended)
cd sim_recorder/examples
python teleop_with_server.py --no-visualize --leader-left-ip 192.168.1.2 --leader-right-ip 192.168.1.3

# Open browser: http://localhost:5000
# Click START to begin recording, STOP to end
```

### Different Robot IPs

```bash
# If your leader robots have different IPs
python teleop_with_server.py --no-visualize \
    --leader-left-ip 192.168.1.10 \
    --leader-right-ip 192.168.1.11
```

### With MuJoCo Viewer

```bash
# If you want to see the simulation visually
python teleop_with_server.py \
    --leader-left-ip 192.168.1.2 \
    --leader-right-ip 192.168.1.3
# Opens MuJoCo viewer window + web UI
```

### Inspect Recorded Data

```bash
# List all episodes
python examples/inspect_episode.py recorded_episodes/

# Show episode details
python examples/inspect_episode.py recorded_episodes/episode_20240115_143022/

# Play episode as video
python examples/inspect_episode.py recorded_episodes/episode_20240115_143022/ \
    --play --camera cam_high --fps 20

# Show all 4 cameras in grid
python examples/inspect_episode.py recorded_episodes/episode_20240115_143022/ \
    --show-cameras
```

### Convert to SERL Pickle Format

```bash
# Convert all episodes to pickle format for SERL training
python examples/convert_to_serl_pickle.py recorded_episodes/ \
    --output-dir serl_demos
```

## Requirements

```bash
# Hardware needed:
- Real Trossen leader robot (for teleoperation)
- Computer with MuJoCo installed

# Software:
pip install mujoco numpy trossen_arm opencv-python
```

## Configuration

Edit `examples/teleop_with_server.py` if you need to customize:

```python
# Camera configuration
camera_names = ['cam_high', 'cam_low', 'cam_left_wrist', 'cam_right_wrist']
camera_resolution = (128, 128)  # (Height, Width)

# Robot IPs (can also set via command line)
leader_left_ip = '192.168.1.2'
leader_right_ip = '192.168.1.3'

# Server URL
server_url = 'http://localhost:5000'

# Visualization
visualize = True  # Set to False for headless mode
```

Command line options:
```bash
python teleop_with_server.py --help
```

## Troubleshooting

**Web UI not loading:**
```bash
# Make sure server is running in Terminal 1
cd sim_recorder/server
python app.py

# Check if port 5000 is available
netstat -tlnp | grep 5000

# Try different port if needed (edit app.py)
```

**Leader robot connection fails:**
```bash
# Check if robots are reachable
ping 192.168.1.2
ping 192.168.1.3

# Verify robots are powered on and in correct mode
# Use correct IPs
python teleop_with_server.py --no-visualize \
    --leader-left-ip 192.168.1.2 \
    --leader-right-ip 192.168.1.3
```

**MuJoCo XML not found:**
```bash
# sim_recorder is self-contained - XML is in assets/
ls assets/trossen_ai_scene_joint.xml
ls assets/meshes/  # Should contain all STL and texture files

# If missing, the XML path resolution will fall back to:
# ../trossen_sim/trossen_sim/envs/xmls/trossen_ai_scene_joint.xml
```

**Cameras not streaming:**
```bash
# Check MuJoCo installation
python -c "import mujoco; print(mujoco.__version__)"

# Verify camera names in XML match script
grep "<camera" assets/trossen_ai_scene_joint.xml

# Check web UI console for errors
# Make sure teleop script is running in Terminal 2
```

**Recording is slow/laggy:**
```bash
# The system runs at MuJoCo native timestep (~500Hz)
# This is optimal for smooth physics
# If too fast, add sleep in teleop loop (edit teleop_with_server.py)
```

## File Structure

```
sim_recorder/
├── README.md                          # This file
├── QUICKSTART_INTEGRATED.md           # Detailed guide
├── requirements.txt                   # Dependencies
├── assets/                            # Self-contained MuJoCo assets
│   ├── trossen_ai_scene_joint.xml     # Main scene XML
│   ├── trossen_ai_joint.xml           # Robot definition XML
│   └── meshes/                        # All 3D meshes & textures
├── examples/
│   ├── teleop_with_server.py         # Main dual-robot teleop script
│   ├── convert_to_serl_pickle.py     # Convert to pickle format
│   └── inspect_episode.py            # Visualize episodes
├── server/                            # Web UI server
│   ├── app.py                         # Flask REST API
│   ├── cameras.py                     # Camera buffer management
│   ├── recorder.py                    # Recording engine
│   └── teleop_ingest.py               # ZeroMQ teleop listener
├── ui/                                # Web interface
│   ├── index.html                     # Camera monitoring UI
│   ├── main.js                        # JavaScript controls
│   └── styles.css                     # UI styling
└── tests/                             # Unit tests
    └── test_integrated_recorder.py
```

## Using Recorded Data for Training

```python
import numpy as np

# Load episode
episode_dir = "recorded_episodes/episode_20240115_143022"

# Load observations
obs = np.load(f"{episode_dir}/observations.npz")
cam_high = obs['cam_high']        # (T, 128, 128, 3)
qpos = obs['qpos']                # (T, 8)
qvel = obs['qvel']                # (T, 8)

# Load actions
actions = np.load(f"{episode_dir}/actions.npy")  # (T, 7)

# Use for training (BC, RL, etc.)
for t in range(len(actions)):
    image = cam_high[t]
    state = np.concatenate([qpos[t], qvel[t]])
    action = actions[t]
    # ... feed to your training loop
```

## Advanced: Web UI Server

The web UI server **always runs** and provides real-time camera monitoring:

```bash
# Terminal 1: Always start the server first
cd sim_recorder/server
python app.py
```

Then in Terminal 2, choose visualization mode:

```bash
# With MuJoCo viewer
python teleop_with_server.py --leader-left-ip 192.168.1.2 --leader-right-ip 192.168.1.3

# Without MuJoCo viewer (headless)
python teleop_with_server.py --no-visualize --leader-left-ip 192.168.1.2 --leader-right-ip 192.168.1.3
```

**Web UI Features:**
- 4 camera streams updating in real-time
- Recording status (idle/recording)
- Episode name and step count
- Start/Stop recording buttons
- **Always available at**: `http://localhost:5000`

## Step-by-Step Guide

See [RECORDING_GUIDE.md](RECORDING_GUIDE.md) for detailed step-by-step instructions.

## License

Same license as parent SERL project.
