#!/usr/bin/env python3
"""
Flask Recording Server for SERL Data Collection
Receives camera frames and teleop actions, records episodes
"""

from flask import Flask, request, jsonify, send_from_directory
import numpy as np
from pathlib import Path
import threading
import json
from datetime import datetime
import zmq
import base64
import time
import yaml
import sys

# Import recorder components
from cameras import CameraManager
from recorder import Recorder

# Load configuration
CONFIG_PATH = Path(__file__).parent.parent / "config.yaml"
try:
    with open(CONFIG_PATH, 'r') as f:
        config = yaml.safe_load(f)
except FileNotFoundError:
    print(f"ERROR: Configuration file not found: {CONFIG_PATH}")
    print("Please ensure config.yaml exists in the sim_recorder directory")
    sys.exit(1)
except yaml.YAMLError as e:
    print(f"ERROR: Invalid YAML in config file: {e}")
    sys.exit(1)

# Extract server config for easy access
SERVER_CONFIG = config['server']

# Configuration constants (from YAML)
MAX_RECORDING_FPS = SERVER_CONFIG['recording']['max_fps']
DEFAULT_RECORDING_FPS = SERVER_CONFIG['recording']['default_fps']
DATA_SAVE_PATH = SERVER_CONFIG['recording']['save_path']
ZMQ_PORT = SERVER_CONFIG['network']['zmq_port']
HTTP_PORT = SERVER_CONFIG['network']['http_port']


class TeleopDataReceiver:
    """Receives teleop data via ZeroMQ and buffers for web UI and recording"""
    
    def __init__(self, zmq_port=5556):
        self.zmq_port = zmq_port
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.bind(f"tcp://*:{zmq_port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        
        # Buffer latest data
        self.latest_data = None
        self.lock = threading.Lock()
        
        # Connection status
        self.last_data_time = None
        self.teleop_connected = False
        
        # Warmup status
        self.warmup_completed = False
        self.warmup_requested = False
        
        # Receiver thread
        self.thread = None
        self.running = False
        
    def start(self):
        """Start receiving thread"""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.thread.start()
            print(f"Teleop data receiver started on port {self.zmq_port}")
    
    def stop(self):
        """Stop receiving"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        self.socket.close()
        self.context.term()
    
    def _receive_loop(self):
        """Receive and buffer teleop data"""
        while self.running:
            try:
                # Non-blocking receive with timeout
                if self.socket.poll(timeout=100):  # 100ms timeout
                    data = self.socket.recv_json()
                    
                    # Decode camera images
                    decoded_cameras = {}
                    for cam_name, cam_data in data['cameras'].items():
                        img_bytes = base64.b64decode(cam_data['data'])
                        image = np.frombuffer(img_bytes, dtype=cam_data['dtype']).reshape(cam_data['shape'])
                        decoded_cameras[cam_name] = image
                    
                    # Update buffered data
                    with self.lock:
                        self.last_data_time = time.time()
                        self.teleop_connected = True
                        self.latest_data = {
                            'timestamp': data['timestamp'],
                            'sequence': data['sequence'],
                            'cameras': decoded_cameras,
                            'robot_state': data['robot_state'],
                            'action': np.array(data['action']),
                            'metadata': data['metadata']
                        }
                        
                        # Update camera manager for web UI
                        for cam_name, image in decoded_cameras.items():
                            if cam_name == 'cam_high':
                                camera_manager.update_frame(0, image)
                            elif cam_name == 'cam_low':
                                camera_manager.update_frame(1, image)
                            elif cam_name == 'cam_left_wrist':
                                camera_manager.update_frame(2, image)
                            elif cam_name == 'cam_right_wrist':
                                camera_manager.update_frame(3, image)
                
            except Exception as e:
                print(f"Warning: Data receiver error: {e}")
                time.sleep(0.1)
    
    def get_latest_data(self):
        """Get latest received data"""
        with self.lock:
            # Check if connection is still active (within 2 seconds)
            if self.last_data_time and (time.time() - self.last_data_time) > 2.0:
                self.teleop_connected = False
            return self.latest_data.copy() if self.latest_data else None


app = Flask(__name__, static_folder='../ui', static_url_path='')

# Initialize components
camera_manager = CameraManager(num_cameras=4)
data_receiver = TeleopDataReceiver(zmq_port=ZMQ_PORT)
recorder = Recorder(camera_manager, data_receiver, base_path=DATA_SAVE_PATH)

# Store in app config for access
app.config['camera_manager'] = camera_manager
app.config['recorder'] = recorder


@app.route('/')
def index():
    """Serve web UI"""
    return send_from_directory('../ui', 'index.html')


@app.route('/<path:filename>')
def serve_static(filename):
    """Serve static files (CSS, JS)"""
    return send_from_directory('../ui', filename)


@app.route('/api/status', methods=['GET'])
def get_status():
    """Get current recording status"""
    latest_data = data_receiver.get_latest_data()
    return jsonify({
        'recording': recorder.is_recording(),
        'current_episode': recorder.current_episode_name if recorder.is_recording() else None,
        'num_steps': recorder.get_num_steps() if recorder.is_recording() else 0,
        'cameras_active': camera_manager.get_active_cameras(),
        'warmup_completed': data_receiver.warmup_completed,
        'teleop_connected': data_receiver.teleop_connected,
        'warmup_requested': data_receiver.warmup_requested,
        'latest_action': latest_data['action'].tolist() if latest_data and 'action' in latest_data else None
    })

'''
@app.route('/api/warmup/trigger', methods=['POST'])
def request_warmup():
    """Request warmup from connected teleop client"""
    if not data_receiver.teleop_connected:
        return jsonify({
            'success': False,
            'error': 'No teleop client connected'
        }), 400
    
    data_receiver.warmup_requested = True
    data_receiver.warmup_completed = False  # Reset completion flag
    
    return jsonify({
        'success': True,
        'message': 'Warmup requested'
    })
'''

@app.route('/api/warmup/clear', methods=['POST'])
def clear_warmup():
    """Clear warmup request flag"""
    data_receiver.warmup_requested = False
    return jsonify({'success': True})


@app.route('/api/warmup/reset', methods=['POST'])
def reset_warmup():
    """Reset warmup completed flag (for next episode)"""
    data_receiver.warmup_completed = False
    data_receiver.warmup_requested = False
    return jsonify({'success': True})


@app.route('/api/start', methods=['POST'])
def start_recording():
    """Start recording new episode"""
    data = request.json or {}
    episode_name = data.get('episode_name', f"episode_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
    fps = data.get('fps', 15)
    
    # Validate FPS against available data rate (30Hz max from teleop)
    if fps > MAX_RECORDING_FPS:
        return jsonify({
            'success': False,
            'error': f'FPS {fps} exceeds maximum available data rate of {MAX_RECORDING_FPS}Hz'
        }), 400
    
    if fps < 1:
        return jsonify({
            'success': False,
            'error': 'FPS must be at least 1'
        }), 400
    
    success = recorder.start_recording(episode_name, fps=fps)
    
    return jsonify({
        'success': success,
        'episode_name': episode_name,
        'fps': fps
    })


@app.route('/api/stop', methods=['POST'])
def stop_recording():
    """Stop current recording"""
    episode_path = recorder.stop_recording()
    
    return jsonify({
        'success': episode_path is not None,
        'episode_path': str(episode_path) if episode_path else None
    })


@app.route('/api/config', methods=['GET'])
def get_config():
    """Get configuration values for UI"""
    return jsonify({
        'max_recording_fps': MAX_RECORDING_FPS,
        'default_recording_fps': DEFAULT_RECORDING_FPS
    })


@app.route('/api/warmup/trigger', methods=['POST'])
def trigger_warmup():
    """Trigger warmup sequence in teleop client"""
    try:
        # Check if teleop client is connected
        if not data_receiver.teleop_connected:
            return jsonify({
                'success': False,
                'error': 'Teleop client not connected. Start the teleop client first.'
            }), 400
        
        # Send warmup command to teleop client
        if data_receiver.send_warmup_command():
            return jsonify({'success': True, 'message': 'Warmup command sent to teleop client'})
        else:
            return jsonify({
                'success': False,
                'error': 'Failed to send warmup command'
            }), 500
            
    except Exception as e:
        print(f"Error triggering warmup: {e}")
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


@app.route('/api/warmup/complete', methods=['POST'])
def warmup_complete():
    """Receive notification that warmup is complete"""
    try:
        data = request.json
        status = data.get('status', 'completed')
        
        # Set warmup completed flag
        data_receiver.warmup_completed = True
        
        print(f"Warmup completed with status: {status}")
        return jsonify({'received': True})
    except Exception as e:
        print(f"Error processing warmup notification: {e}")
        return jsonify({'error': str(e)}), 400


@app.route('/api/delete', methods=['POST'])
def delete_episode():
    """Delete an episode"""
    data = request.json
    episode_id = data.get('episode_id')
    
    success = recorder.delete_episode(episode_id)
    return jsonify({'success': success})


@app.route('/api/frame/<int:cam_id>', methods=['POST'])
def receive_frame(cam_id):
    """Receive camera frame (raw numpy bytes or JPEG)"""
    content_type = request.headers.get('Content-Type', '')
    
    if 'octet-stream' in content_type:
        # Raw numpy bytes: reshape to (H, W, 3)
        img_bytes = request.data
        frame = np.frombuffer(img_bytes, dtype=np.uint8).reshape(128, 128, 3)
    else:
        # JPEG bytes
        import io
        from PIL import Image
        img = Image.open(io.BytesIO(request.data))  
        frame = np.array(img)
    
    camera_manager.update_frame(cam_id, frame)
    
    return jsonify({'success': True})


@app.route('/stream_cam/<int:cam_id>')
def stream_camera(cam_id):
    """Stream camera feed (MJPEG)"""
    def generate():
        while True:
            frame = camera_manager.get_frame(cam_id)
            if frame is not None:
                from PIL import Image
                import io
                
                img = Image.fromarray(frame)
                buf = io.BytesIO()
                img.save(buf, format='JPEG')
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buf.getvalue() + b'\r\n')
    
    from flask import Response
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


def main():
    # Start ZeroMQ data receiver
    data_receiver.start()
    
    print("="*60)
    print("SERL Recording Server Configuration")
    print("="*60)
    print(f"ZeroMQ Data Port: {ZMQ_PORT}")
    print(f"Max Recording FPS: {MAX_RECORDING_FPS}Hz (limited by teleop data rate)")
    print(f"Default Recording FPS: {DEFAULT_RECORDING_FPS}Hz")
    print(f"Episode Save Path: {DATA_SAVE_PATH}")
    print(f"Web UI: http://localhost:{HTTP_PORT}")
    print("="*60)
    print("Waiting for teleop data... (start teleop_with_server.py)")
    print("="*60)
    
    app.run(host='0.0.0.0', port=HTTP_PORT, debug=False, threaded=True)


if __name__ == '__main__':
    main()
