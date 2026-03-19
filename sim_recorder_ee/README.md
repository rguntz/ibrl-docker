# Sim Recorder

**Self-contained** data collection for SERL robot learning: Record demos from **MuJoCo simulation** controlled by **real robot teleoperation**.

## 🚀 Quick Start (2 Terminals Required)

### Terminal 1: Start Web UI Server
```bash
cd sim_recorder/server
python app.py --dataset_path data/dataset.hdf5
# Server starts at http://localhost:5000
```

### Terminal 2: Run Teleoperation
```bash
cd sim_recorder/examples
python teleop_with_server.py 
# Connects to robots and streams cameras to web UI
```

### Open Web UI
- **Browser**: `http://localhost:5000`
- **Live camera feeds**: 4 real-time camera streams
- **Recording controls**: START/STOP buttons
- **Status**: Recording progress and episode info




## Format of the dataset 

### HDF5 Structure

The output file (dataset.hdf5) follows this hierarchical structure:

```shell
data/ (Root Group)
├── attrs: "env_args" (JSON string with env config)
├── demo_0/ (Episode Group)
│   ├── actions             [T, 14] float32
│   ├── rewards             [T]     float32
│   └── obs/ (Observation Group)
│       ├── qpos            [T, 16] float32
│       ├── qvel            [T, 16] float32
│       ├── robot0_eef_pos  [T, 6]  float32
│       ├── robot0_eef_quat [T, 8]  float32
│       ├── robot0_gripper_qpos [T, 4] float32
│       └── {cam_name}_image    [T, C, H, W] uint8 (gzip compressed)
├── demo_1/
│   └── ...
```

- Image Tensor Layout: Images are captured as (H, W, C) but stored as (T, C, H, W) (Channel-first) to match standard PyTorch loaders.

- Compression: All image datasets are saved with gzip compression to reduce disk footprint.
Data Types:
States/Actions: float32
Images: uint8


## Data Processing

- Move the hdf5 file to the direction : 
```shell
ibrl/data/data_processing_sim
```
Run the processing file : 
```shell
python processing.py
```

