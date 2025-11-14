# SERL Sim Recorder - Testing Guide

## Quick Test

1. Start server:
```bash
cd sim_recorder/server
python app.py
```

2. Start teleop (in another terminal):
```bash
cd sim_recorder/examples
python teleop_with_server.py --no-visualize
```

3. Open browser: http://localhost:5000

4. Click "Start Recording" in web UI

## Expected Behavior

- Server starts without errors
- Teleop connects to robots and performs warmup sequence
- Cameras stream in web UI
- Recording saves episodes to data/ folder
- Episodes contain camera images, robot states, and actions

## Troubleshooting

- Check robot IPs in config.yaml
- Verify MuJoCo XML exists in assets/
- Check ZeroMQ ports not in use
- Ensure all Python dependencies installed
🌐 Connecting to server at http://localhost:5000...
✓ Server connected
📡 Data transmission started on port 5556
🎮 Starting dual robot teleop control loop...
```

### 4. Data Transmission Test
1. With teleop running, check web UI
2. Camera views should appear dynamically
3. Status should show active cameras (cam_high, cam_low, cam_left_wrist, cam_right_wrist)

### 5. Recording Test
1. In web UI, click "Start Recording"
2. Move leader robots for 10-15 seconds
3. Click "Stop Recording"
4. Verify episode appears in episodes list

### 6. Data Validation Test
Check recorded episode structure:
```bash
ls data/episode_*/  # Should contain: observations.npz, actions.npy, meta.json
```

**meta.json should contain:**
```json
{
  "episode_name": "episode_...",
  "num_steps": 150,
  "fps": 15,
  "cameras": ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"],
  "has_robot_state": true
}
```

**observations.npz should contain:**
- cam_high: (150, 640, 480, 3) - High-res teleop images
- cam_low: (150, 640, 480, 3)
- cam_left_wrist: (150, 640, 480, 3)
- cam_right_wrist: (150, 640, 480, 3)
- qpos: (150, 16) - Robot joint positions
- qvel: (150, 16) - Robot joint velocities

**actions.npy should contain:**
- Shape: (150, 14) - Teleop actions from both leaders

## Performance Validation

### ZeroMQ Non-Blocking Test
1. Run teleop with `--visualize` flag
2. Monitor MuJoCo viewer FPS (should stay ~500Hz)
3. Verify smooth real-time control
4. Check that camera updates don't cause stuttering

### Data Queue Test
1. Monitor server logs for queue size messages
2. Verify 30fps camera sampling
3. Check that memory usage stays bounded

### Recording Quality Test
1. Record episode with varied robot movements
2. Verify RLDS format compatibility
3. Check data synchronization (cameras + states + actions)

## Troubleshooting

### No Cameras in Web UI
- Check ZeroMQ connection (port 5556)
- Verify teleop is sending data packets
- Check server logs for decode errors

### Recording Fails
- Ensure data receiver has latest data
- Check disk space and permissions
- Verify numpy array shapes match

### Performance Issues
- Check ZeroMQ socket buffer sizes
- Monitor system resources
- Adjust sampling rates if needed

### Robot Connection Issues
- Verify USB connections
- Check IP addresses
- Test with single robot first

## Success Criteria
- ✅ Server starts without errors
- ✅ Web UI shows dynamic camera grid
- ✅ Teleop runs at 500Hz without blocking
- ✅ Recording produces valid RLDS format
- ✅ Data includes cameras, qpos, qvel, and actions
- ✅ Memory usage stays bounded
- ✅ No dropped frames or synchronization issues</content>
<parameter name="filePath">c:\Users\nannu\Projects\serl\sim_recorder\TESTING_GUIDE.md