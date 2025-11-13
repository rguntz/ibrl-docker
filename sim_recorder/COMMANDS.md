# Quick Commands - Data Recording

## TWO MODES: Integrated vs Server-Based

### Mode 1: Integrated (ONE command - auto-records)

```bash
cd /home/qte9489/personal_abhi/serl/sim_recorder/examples
conda activate serl
python integrated_teleop_recorder.py --duration 60 --auto-record --web-ui
```

**Use this for**: Quick recording sessions where you start/stop manually

---

### Mode 2: Server-Based (TWO terminals - web UI with START/STOP buttons)

**Terminal 1: Start Web UI Server**
```bash
cd /home/qte9489/personal_abhi/serl/sim_recorder/server
conda activate serl
python app.py
```
📱 **Open browser**: http://localhost:5000

**Terminal 2: Start MuJoCo Teleop**
```bash
cd /home/qte9489/personal_abhi/serl/sim_recorder/examples  
conda activate serl
python teleop_with_server.py
```

🎮 **MuJoCo viewer will open** - move your robot!
🔴 **Click START in web UI** when ready to record
⏹️ **Click STOP in web UI** when done

**Use this for**: Multiple recordings with web monitoring

---

## TL;DR - RECOMMENDED (Server Mode)

**Terminal 1:**
```bash
conda activate serl && cd /home/qte9489/personal_abhi/serl/sim_recorder/server && python app.py
```

**Terminal 2:**  
```bash
conda activate serl && cd /home/qte9489/personal_abhi/serl/sim_recorder/examples && python teleop_with_server.py
```

**Browser:** http://localhost:5000 👈 Click START/STOP here!

---

## Common Commands

```bash
# Basic (no web UI)
python integrated_teleop_recorder.py --duration 60 --auto-record

# With web UI (see cameras)
python integrated_teleop_recorder.py --duration 60 --auto-record --web-ui

# Longer recording
python integrated_teleop_recorder.py --duration 120 --auto-record --web-ui

# Custom save folder
python integrated_teleop_recorder.py --duration 60 --auto-record --save-dir pick_demos

# Different robot IP
python integrated_teleop_recorder.py --duration 60 --auto-record --leader-ip 192.168.1.5

# Faster control (30 Hz)
python integrated_teleop_recorder.py --duration 60 --auto-record --control-freq 30
```

---

## After Recording

```bash
# List episodes
python inspect_episode.py recorded_episodes/

# Play episode
python inspect_episode.py recorded_episodes/episode_20240115_143022/ --play

# Convert to pickle
python convert_to_serl_pickle.py recorded_episodes/ --output-dir serl_demos
```

---

## Troubleshooting

```bash
# Robot not connecting?
ping 192.168.1.2

# Port 5000 busy?
python integrated_teleop_recorder.py --duration 60 --auto-record --web-ui --web-ui-port 5001

# MuJoCo XML not found?
ls ../trossen_sim/trossen_sim/envs/xmls/trossen_ai_scene_joint.xml
```
