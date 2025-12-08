# Data Recording Guide - Step by Step

## Quick Start (No Web UI)

```bash
cd /home/qte9489/personal_abhi/serl/sim_recorder/examples
python integrated_teleop_recorder.py --duration 60 --auto-record
```

Done! Data saved to `recorded_episodes/episode_<timestamp>/`

---

## With Web UI Monitoring (Recommended)

### Step 1: Start Recording with Web UI

```bash
cd /home/qte9489/personal_abhi/serl/sim_recorder/examples

python integrated_teleop_recorder.py \
    --duration 60 \
    --auto-record \
    --web-ui
```

This will:
1. Connect to leader robot at 192.168.1.2
2. Load MuJoCo sim
3. **Start web server at http://localhost:5000**
4. Begin recording immediately

### Step 2: Open Web Browser (Optional)

While recording is running, open browser to:
```
http://localhost:5000
```

You'll see:
- 4 camera streams in real-time
- Recording status (REC indicator)
- Number of steps recorded
- Episode name

### Step 3: Recording Automatically Stops

After 60 seconds (or your specified duration):
- Recording stops automatically
- Data saved to `recorded_episodes/episode_<timestamp>/`
- Web server shuts down
- Script exits

---

## Manual Recording Control

If you don't want auto-record:

```bash
python integrated_teleop_recorder.py \
    --duration 120 \
    --web-ui
```

Then:
1. Script starts, waits for you to press ENTER
2. Press ENTER to begin teleoperation
3. In web browser, click "Start Recording" when ready
4. Perform your demo
5. Click "Stop Recording" in browser
6. Repeat steps 3-5 for multiple episodes
7. After 120 seconds, script exits

---

## Common Commands

### Basic recording (30 seconds)
```bash
python integrated_teleop_recorder.py --duration 30 --auto-record
```

### With web UI for monitoring
```bash
python integrated_teleop_recorder.py --duration 60 --auto-record --web-ui
```

### Custom save location
```bash
python integrated_teleop_recorder.py \
    --duration 60 \
    --save-dir my_task_demos \
    --auto-record \
    --web-ui
```

### Different robot IP
```bash
python integrated_teleop_recorder.py \
    --leader-ip 192.168.1.5 \
    --duration 60 \
    --auto-record
```

### Higher control frequency (30 Hz)
```bash
python integrated_teleop_recorder.py \
    --control-freq 30.0 \
    --duration 60 \
    --auto-record
```

### Skip home position (start immediately)
```bash
python integrated_teleop_recorder.py \
    --no-home \
    --duration 60 \
    --auto-record
```

---

## What Happens During Recording

```
┌─────────────────────────────────────────────┐
│ 1. Connect to Real Leader Robot            │
│    - IP: 192.168.1.2                        │
│    - Read joint positions @ 20 Hz           │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ 2. Load MuJoCo Simulation                   │
│    - XML: trossen_ai_scene_joint.xml        │
│    - 4 cameras: high, low, left, right      │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ 3. [Optional] Start Web UI Server          │
│    - Flask server on port 5000              │
│    - Open http://localhost:5000 in browser  │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ 4. Control Loop (20 Hz)                     │
│    ┌──────────────────────────────────────┐ │
│    │ Read leader positions → Actions      │ │
│    │ Apply to sim → Step physics          │ │
│    │ Render 4 cameras (128x128)           │ │
│    │ Get robot state (qpos, qvel)         │ │
│    │ [If web UI] Push frames to browser   │ │
│    │ Save to memory buffer                │ │
│    └──────────────────────────────────────┘ │
│    Repeat until duration expires...         │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ 5. Save Episode to Disk                     │
│    recorded_episodes/episode_YYYYMMDD/      │
│    ├── observations.npz (cameras + states)  │
│    ├── actions.npy (leader commands)        │
│    └── meta.json (episode info)             │
└─────────────────────────────────────────────┘
```

---

## Inspect Recorded Data

### List all episodes
```bash
python inspect_episode.py recorded_episodes/
```

### Show episode details
```bash
python inspect_episode.py recorded_episodes/episode_20240115_143022/
```

Output:
```
Episode: episode_20240115_143022
Steps: 1204
Duration: 60.2s
FPS: 20.0
Cameras: cam_high, cam_low, cam_left_wrist, cam_right_wrist
```

### Play episode as video
```bash
python inspect_episode.py recorded_episodes/episode_20240115_143022/ \
    --play --camera cam_high --fps 20
```

### Show all 4 cameras
```bash
python inspect_episode.py recorded_episodes/episode_20240115_143022/ \
    --show-cameras
```

---

## Troubleshooting

### Leader robot not connecting
```bash
# Check if robot is reachable
ping 192.168.1.2

# Check if robot is powered on
# Verify robot is in correct mode (leader mode)

# Try different IP
python integrated_teleop_recorder.py --leader-ip 192.168.1.5
```

### Web UI not opening
```bash
# Check if port 5000 is available
lsof -i :5000

# Use different port
python integrated_teleop_recorder.py --web-ui --web-ui-port 5001

# Then open: http://localhost:5001
```

### MuJoCo XML not found
```bash
# Check if file exists
ls ../trossen_sim/trossen_sim/envs/xmls/trossen_ai_scene_joint.xml

# If not, edit XML_PATH in integrated_teleop_recorder.py
```

### Recording is laggy/slow
```bash
# Reduce control frequency
python integrated_teleop_recorder.py --control-freq 10.0

# Or edit script to reduce camera resolution to 64x64
```

---

## Next Steps

After recording demos:

### 1. Convert to SERL pickle format
```bash
python convert_to_serl_pickle.py recorded_episodes/ --output-dir serl_demos
```

### 2. Train policy
```bash
cd ../../examples
python bc_policy.py --demo-dir ../sim_recorder/recorded_episodes
```

### 3. Evaluate policy
```bash
python async_drq_sim/async_drq_sim.py --eval
```
