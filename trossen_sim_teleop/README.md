# Trossen Real-to-Sim Teleoperation

Control MuJoCo sim robots using real physical Trossen leader robots.

## How It Works

```
Physical Leader Robot           MuJoCo Sim Follower
┌────────────────┐             ┌────────────────┐
│  You move this │             │  Mirrors leader│
│  (free motion) │────USB──────►│  (qpos update) │
└────────────────┘             └────────────────┘
     ↑ trossen_arm                  ↑ mujoco
     get_all_positions()            qpos injection
```

**Key MuJoCo concept:**
- `data.qpos[:] = joint_angles` - Directly set robot pose
- `mj_forward()` - Update kinematics/physics
- `viewer.sync()` - Render frame

No physics simulation during teleop - just kinematic mirroring!

## Files

- `single_robot_teleop_sim.py` - 1 leader → 1 sim robot
- `dual_robot_teleop_sim.py` - 2 leaders → 2 sim robots

## Prerequisites

```bash
# Install trossen_arm Python bindings
pip install trossen-arm

# Already have mujoco (comes with trossen_sim)
```

## Usage

### Single Robot Teleop
```bash
cd /home/qte9489/personal_abhi/serl/trossen_sim_teleop

# Connect leader via USB, then:
python single_robot_teleop_sim.py \
    --leader-ip 192.168.1.2 \
    --duration 60

# Run for longer (5 minutes):
python single_robot_teleop_sim.py --duration 300

# Run indefinitely (until Ctrl+C):
python single_robot_teleop_sim.py --duration 999999

# Or skip home position:
python single_robot_teleop_sim.py --no-home --duration 30
```

### Dual Robot Teleop  
```bash
# Connect both leaders, then:
python dual_robot_teleop_sim.py \
    --left-ip 192.168.1.2 \
    --right-ip 192.168.1.3 \
    --duration 60

# Run for longer:
python dual_robot_teleop_sim.py \
    --left-ip 192.168.1.2 \
    --right-ip 192.168.1.3 \
    --duration 300
```

**Note:** The `--duration` parameter sets how long the teleoperation runs. After the time limit, the script stops automatically. Use Ctrl+C to stop early.

## What Happens

1. **Initialize**: Connect to leader robots via `trossen_arm` SDK
2. **Load Sim**: Load MuJoCo model from `trossen_ai_scene_joint.xml`
3. **Randomize Cube**: Spawn red cube at random table position
4. **Set Camera**: Start with left wrist camera view (better for manipulation)
5. **Home Position**: Move leaders to safe home pose (optional)
6. **Teleop Loop**:
   ```python
   while running:
       # Read from leader
       joints = leader.get_all_positions()  # 7D array (6 arm + 1 gripper)
       
       # Inject into sim control
       data.ctrl[0:6] = joints  # Arm joints
       data.ctrl[6:8] = gripper  # Gripper fingers
       
       # Step physics
       mujoco.mj_step(model, data)
       viewer.sync()
   ```
7. **Cleanup**: Close connections

## Camera Controls

The MuJoCo viewer supports multiple camera views:

**Keyboard shortcuts:**
- **`[`** (left bracket) - Previous camera
- **`]`** (right bracket) - Next camera  
- **`0-9`** - Jump to camera by index
- **`ESC`** - Toggle free camera (mouse control)

**Available cameras:**
- `[0]` cam_high - Top-down view
- `[1]` cam_low - Front low angle
- `[2]` cam_left_wrist - Left wrist view (good for picking!)
- `[3]` cam_right_wrist - Right wrist view

**Tip:** Use wrist cameras (`[2]` or `[3]`) for precise manipulation and picking.

## Joint Mapping

**Leader (physical)** → **Follower (sim)**

```
Leader qpos indices:          Sim qpos indices:
─────────────────────        ───────────────────
Single robot:                Single robot:
  joints[0-5]: arm             qpos[0-5]: left arm
  gripper: gripper             qpos[6]: left gripper

Dual robots:                 Dual robots:
  left_joints[0-5]            qpos[0-5]: left arm
  left_gripper                qpos[6]: left gripper
  right_joints[0-5]           qpos[7-12]: right arm  
  right_gripper               qpos[13]: right gripper
```

## Troubleshooting

**"No module named 'trossen_arm'"**
→ `pip install trossen-arm`

**"XML file not found"**
→ Check path to `trossen_ai_scene_joint.xml` in code

**"Leader connection failed"**
→ Check USB connection and IP address
→ Try: `ping 192.168.1.2`

**Sim robot doesn't move**
→ Check `sim_joint_start_idx` matches your XML joint order
→ Print `mj_model.joint_names` to see joint order

**Gripper not working**
→ Adjust gripper index in code (should be after 6 arm joints)

**Cube not visible**
→ Check cube joint name in XML (should be "cube_joint")

**MuJoCo crashes/closes unexpectedly**
→ Check if you reached the time limit (set by `--duration`)
→ The script stops automatically when time runs out
→ Increase duration: `--duration 300` (5 minutes)
→ Or run indefinitely: `--duration 999999`

**Camera switching:**
→ Use `[` `]` bracket keys to switch cameras
→ Number keys (0-9) don't work in passive viewer mode
→ ESC toggles free camera mode

## Next Steps

To record demos for training:
1. Run teleop script
2. Add data recording (save qpos, qvel, gripper states)
3. Convert to RLDS format
4. Use with `--preload_rlds_path` in training

## Reference

Based on ChatGPT's MuJoCo teleop pattern:
```python
while viewer.is_running():
    q_real = get_real_robot_joint_positions()
    data.qpos[:] = q_real
    mujoco.mj_forward(model, data)
    viewer.sync()
```
