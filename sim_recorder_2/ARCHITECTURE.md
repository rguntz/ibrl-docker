# SERL Teleop System Architecture

## Overview

The SERL teleop system implements a real-time robot teleoperation and data recording pipeline using ZeroMQ for communication. The system consists of three main components: teleoperation client, web server, and recording engine.

## System Components

```
┌─────────────────┐    ZeroMQ    ┌─────────────────┐    HTTP     ┌─────────────────┐
│ Teleop Client   │─────────────►│ Web Server      │◄───────────►│ Web Browser     │
│ (MuJoCo + Real  │  (Port 5556) │ (Flask + ZeroMQ)│  (Port 5000)│ (UI Controls)   │
│ Robots)         │              │                 │             │                 │
└─────────────────┘              └─────────────────┘             └─────────────────┘
                                   │
                                   ▼
                            ┌─────────────────┐
                            │ Recording Engine│
                            │ (FPS-controlled │
                            │ data sampling)  │
                            └─────────────────┘
```

## Configuration Parameters

### Teleop Client (`teleop_with_server.py`)
- **Camera Resolution:** `camera_resolution=(640, 480)` - MuJoCo rendering resolution
- **Data Sample Rate:** `data_sample_rate=30` - How often data is sampled from MuJoCo loop (Hz)
- **MuJoCo Frequency:** ~500Hz (determined by `mj_model.opt.timestep`)
- **ZeroMQ Port:** `zmq_port=5556` - Data transmission port
- **Queue Size:** `max_size=30` - Maximum queued data packets

### Web Server (`app.py`)
- **Max Recording FPS:** `MAX_RECORDING_FPS=30` - Cannot exceed teleop data rate
- **Default Recording FPS:** `DEFAULT_RECORDING_FPS=15` - UI default value
- **ZeroMQ Port:** `ZMQ_PORT=5556` - Data reception port
- **Web Port:** `5000` - HTTP server port
- **Save Path:** `DATA_SAVE_PATH='data'` - Episode storage directory

### Recording Engine (`recorder.py`)
- **Recording FPS:** User-configurable 1-30Hz, validated against available data
- **Sample Method:** Takes every Nth packet from 30Hz teleop stream
- **Error Handling:** Stops recording if no data available (no dummy data generation)

## Data Flow Architecture

### 1. Teleoperation Client (`teleop_with_server.py`)

**Purpose:** Reads real robot states, applies them to MuJoCo simulation, captures cameras, and transmits complete data packets.

**Configuration:**
- **Camera Resolution:** 640×480 (configurable via `camera_resolution` parameter)
- **Data Sample Rate:** 30Hz (configurable via `data_sample_rate` parameter)
- **MuJoCo Frequency:** ~500Hz (based on `mj_model.opt.timestep`)
- **ZeroMQ Port:** 5556 (configurable via `zmq_port` parameter)

**Key Components:**
- **BoundedDataQueue:** Thread-safe queue with configurable sampling rate (default 30fps)
- **ZeroMQ PUB Socket:** Non-blocking data transmission
- **Async Transmission Thread:** Prevents blocking main teleop loop
- **Camera Renderer:** High-resolution camera capture

**Data Flow:**
1. MuJoCo runs at ~500Hz, capturing robot state and rendering cameras
2. Every teleop step, camera images and robot state are captured
3. Data is sampled at 30Hz into BoundedDataQueue (not every frame)
4. ZeroMQ transmits sampled data to server asynchronously

**Data Packet Structure:**
```python
{
    'timestamp': time.time(),
    'sequence': sequence_counter,
    'cameras': {
        'cam_high': base64_encoded_image,
        'cam_low': base64_encoded_image,
        'cam_left_wrist': base64_encoded_image,
        'cam_right_wrist': base64_encoded_image
    },
    'robot_state': {
        'qpos': [16_floats],  # joint positions
        'qvel': [16_floats]   # joint velocities
    },
    'action': [14_floats],  # leader commands
    'metadata': {
        'frequency': 30,
        'camera_resolution': (640, 480),
        'camera_names': [...]
    }
}
```

**Main Loop Pseudocode:**
```
WHILE running:
    # Read real robot states
    left_state = leader_left.get_positions()   # 7D
    right_state = leader_right.get_positions() # 7D

    # Apply to MuJoCo simulation
    mj_data.ctrl[0:7] = left_state
    mj_data.ctrl[7:14] = right_state
    mj_step(model, data)

    # Capture cameras
    cameras = capture_all_cameras()

    # Get robot states
    qpos = mj_data.qpos[3:19]  # Skip cube, get 16 joints
    qvel = mj_data.qvel[3:19]

    # Create action vector
    action = concatenate(left_state, right_state)

    # Queue data packet
    queue_teleop_data(cameras, robot_state, action)

    # Sleep for 500Hz physics
    sleep(0.002)
```

### 2. Web Server (`app.py`)

**Purpose:** Receives ZeroMQ data packets, updates web UI, and manages recording sessions.

**Configuration:**
- **Max Recording FPS:** 30Hz (limited by teleop data rate)
- **Default Recording FPS:** 15Hz
- **ZeroMQ Port:** 5556
- **Web Port:** 5000
- **Save Path:** `data/` directory

**Key Components:**
- **TeleopDataReceiver:** ZeroMQ SUB socket listener
- **CameraManager:** Buffers latest camera frames for MJPEG streaming
- **Recorder:** FPS-controlled episode recording with validation
- **Flask API:** REST endpoints with input validation

**Validation:**
- Recording FPS cannot exceed 30Hz (teleop data limit)
- Recording FPS must be ≥ 1Hz
- Server prints configuration at startup

**ZeroMQ Reception Flow:**
```
ZeroMQ SUB Socket (Port 5556)
    ↓
Receive JSON packet
    ↓
Decode base64 camera images
    ↓
Update CameraManager buffers
    ↓
Update web UI via polling
```

**API Endpoints:**
- `GET /api/status` - Recording status and camera info
- `POST /api/start` - Start recording episode
- `POST /api/stop` - Stop recording and save
- `GET /stream_cam/<id>` - MJPEG camera streams

### 3. Recording Engine (`recorder.py`)

**Purpose:** Samples data at controlled FPS rates and saves RLDS-compatible episodes.

**Configuration:**
- **Recording FPS:** User-configurable (1-30Hz), validated against data availability
- **Sample Method:** Takes every Nth data packet from 30Hz stream
- **Error Handling:** Stops recording if no data available (no dummy data)

**Key Components:**
- **FPS Sampling:** Configurable sampling rate with validation
- **Threaded Recording:** Background recording loop
- **RLDS Format:** Proper observation/action structure with metadata

**Recording Flow:**
```
WHILE recording:
    WAIT for next sample interval (1/recording_fps)

    GET latest data from TeleopDataReceiver

    IF data available:
        observation = cameras + robot_state
        action = latest_action
        APPEND to episode
    ELSE:
        LOG ERROR and STOP recording (no dummy data)
```

    SAVE episode on stop
```

## Communication Patterns

### ZeroMQ PUB/SUB Pattern
```
Teleop Client (PUB) ────► Web Server (SUB)
    │                           │
    │ sends complete           │ receives complete
    │ data packets             │ data packets
    │                          │
    └─── non-blocking ───────► └─── buffered for UI
```

**Benefits:**
- Non-blocking transmission (teleop loop stays at 500Hz)
- Reliable delivery with sequence numbers
- JSON serialization with base64 image encoding

### HTTP REST API
```
Web Browser ────► Flask Server
    │                     │
    │ UI controls         │ status updates
    │ recording start/stop│ camera streaming
```

## Data Synchronization

### Bounded Queue Strategy
```
Teleop Loop (500Hz) ───► BoundedDataQueue (max 30 items)
    │                           │
    │ produces data            │ 30fps sampling
    │ continuously             │ prevents overflow
    │                          │
Transmission Thread ───► ZeroMQ PUB Socket
    │                           │
    │ consumes queue           │ async transmission
    │ non-blocking             │ no main loop delay
```

### Web UI Updates
```
ZeroMQ Receiver ───► CameraManager ───► MJPEG Streams
    │                       │                   │
    │ latest frames         │ frame buffers     │ browser display
    │                       │                   │
Status API ◄─────────────── CameraManager
    │
Web Browser (polls every 500ms)
```

## Error Handling & Resilience

### Teleop Client
- ZeroMQ connection test on startup
- Continues operation even if server unavailable
- Graceful cleanup on interruption

### Web Server
- ZeroMQ receive timeouts (100ms)
- Exception handling in data decoding
- HTTP endpoint error responses

### Recording Engine
- Skips frames if no data available
- Validates data structure before saving
- Proper episode cleanup on errors

## Performance Characteristics

- **Teleop Loop:** 500Hz (2ms physics steps)
- **Camera Sampling:** 30fps (configurable)
- **Data Transmission:** Async, non-blocking
- **Web UI Updates:** 2Hz (500ms polling)
- **Recording FPS:** 15fps (configurable)

## File Structure & Dependencies

```
sim_recorder/
├── examples/teleop_with_server.py    # Main teleop client
├── server/
│   ├── app.py                         # Flask server + ZeroMQ receiver
│   ├── recorder.py                    # Recording engine
│   └── cameras.py                     # Camera buffer management
├── ui/                                # Web interface
└── assets/                            # MuJoCo models & meshes
```

**Runtime Dependencies:**
- mujoco (physics simulation)
- trossen_arm (robot interface)
- zmq (communication)
- flask (web server)
- opencv/numpy (image processing)

## Deployment & Testing

### Development Setup
```bash
# Terminal 1: Web server
cd sim_recorder/server && python app.py

# Terminal 2: Teleop client
cd sim_recorder/examples && python teleop_with_server.py

# Browser: http://localhost:5000
```

### Production Considerations
- ZeroMQ socket binding on all interfaces (0.0.0.0)
- Configurable ports and IPs
- Error logging and monitoring
- Resource usage monitoring (memory, CPU)

This architecture provides real-time teleoperation with reliable data recording while maintaining high performance and clean separation of concerns.