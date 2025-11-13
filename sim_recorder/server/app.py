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

# Import recorder components
from cameras import CameraManager
from recorder import Recorder
from teleop_ingest import TeleopListener

app = Flask(__name__, static_folder='../ui', static_url_path='')

# Initialize components
camera_manager = CameraManager(num_cameras=4)
recorder = Recorder(camera_manager, base_path='data')
teleop_listener = TeleopListener(port=5555)

# Store in app config for access
app.config['camera_manager'] = camera_manager
app.config['recorder'] = recorder
app.config['teleop_listener'] = teleop_listener


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
    latest_state = recorder.get_latest_state() if hasattr(recorder, 'get_latest_state') else None
    latest_state_serializable = None
    if latest_state is not None:
        latest_state_serializable = {
            'qpos': latest_state['qpos'].tolist(),
            'qvel': latest_state['qvel'].tolist(),
            'action': latest_state['action'].tolist()
        }

    return jsonify({
        'recording': recorder.is_recording(),
        'current_episode': recorder.current_episode_name if recorder.is_recording() else None,
        'num_steps': recorder.get_num_steps() if recorder.is_recording() else 0,
        'cameras_active': camera_manager.get_active_cameras(),
        'latest_action': teleop_listener.get_latest_action().tolist() if teleop_listener.get_latest_action() is not None else None,
        'latest_state': latest_state_serializable
    })


@app.route('/api/start', methods=['POST'])
def start_recording():
    """Start recording new episode"""
    data = request.json or {}
    episode_name = data.get('episode_name', f"episode_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
    fps = data.get('fps', 15)
    
    success = recorder.start_recording(episode_name, fps=fps)
    
    return jsonify({
        'success': success,
        'episode_name': episode_name
    })


@app.route('/api/stop', methods=['POST'])
def stop_recording():
    """Stop current recording"""
    episode_path = recorder.stop_recording()
    
    return jsonify({
        'success': episode_path is not None,
        'episode_path': str(episode_path) if episode_path else None
    })


@app.route('/api/list', methods=['GET'])
def list_episodes():
    """List all recorded episodes"""
    episodes = recorder.list_episodes()
    return jsonify({'episodes': episodes})


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


@app.route('/api/teleop', methods=['POST']) # nothing is posted on this adress. 
def receive_teleop():
    """Receive teleop action via HTTP (alternative to ZeroMQ)"""
    data = request.json
    action = np.array(data['action'])
    
    teleop_listener.set_action(action)
    
    return jsonify({'success': True})


@app.route('/api/state', methods=['POST'])
def receive_state():
    """Receive robot state (qpos, qvel, action) from teleop client and store it for the recorder."""
    data = request.json or {}
    # Expect arrays / lists in JSON
    try:
        qpos = np.array(data['qpos'])
        qvel = np.array(data['qvel'])
        action = np.array(data['action'])
    except Exception:
        return jsonify({'success': False, 'error': 'invalid payload'}), 400

    # Push into recorder (recorder will keep latest_state and recorder thread will use it)
    if hasattr(recorder, 'set_latest_state'):
        recorder.set_latest_state(qpos, qvel, action)

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
    # Start teleop listener in background
    teleop_listener.start()
    
    print("="*60)
    print("SERL Recording Server")
    print("="*60)
    print(f"Web UI: http://localhost:5000")
    print(f"ZeroMQ: tcp://localhost:5555")
    print("="*60)
    
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)


if __name__ == '__main__':
    main()
