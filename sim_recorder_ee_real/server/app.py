#!/usr/bin/env python3
"""
Flask Recording Server for SERL Data Collection - SIMPLIFIED VERSION
Only handles control API (start/stop/list/delete).
All data recording is done locally by the teleop client.
"""

from flask import Flask, request, jsonify, send_from_directory
import numpy as np
from pathlib import Path
import threading
import json
import argparse
from datetime import datetime
import h5py

# Import recorder for episode management
from recorder import Recorder

app = Flask(__name__, static_folder='../ui', static_url_path='')

# Initialised in main() after CLI args are parsed
recorder = None


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
    return jsonify({
        'recording': recorder.is_recording(),
        'current_episode': recorder.current_episode_name if recorder.is_recording() else None,
        'message': 'Recording is controlled by teleop client (local saving)'
    })


@app.route('/api/start', methods=['POST'])
def start_recording():
    """Start recording signal — teleop client will do the actual saving."""
    data = request.json or {}
    episode_name = data.get('episode_name', f"episode_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
    fps = data.get('fps', 15)

    success = recorder.start_recording(episode_name, fps=fps)

    return jsonify({
        'success': success,
        'episode_name': episode_name,
        'message': 'Teleop client will handle local recording'
    })


@app.route('/api/stop', methods=['POST'])
def stop_recording():
    """Stop recording signal — teleop client will finalise local recording."""
    episode_name = recorder.current_episode_name if recorder.is_recording() else None
    print("episode_name : ", episode_name)
    success = recorder.stop_recording()

    return jsonify({
        'success': success,
        'episode_name': episode_name,
        'message': 'Teleop client will finalize local recording'
    })


@app.route('/api/list', methods=['GET'])
def list_episodes():
    """List all recorded episodes from HDF5"""
    episodes = recorder.list_episodes()
    return jsonify({'episodes': episodes})


@app.route('/api/delete', methods=['POST'])
def delete_episode():
    """Delete an episode"""
    data = request.json or {}
    episode_id = data.get('episode_id')

    success = recorder.delete_episode(episode_id)
    return jsonify({'success': success})


def main():
    global recorder

    parser = argparse.ArgumentParser(description='SERL Recording Server')
    parser.add_argument(
        '--dataset_path',
        type=str,
        default='data/dataset.hdf5',
        help='Path to the HDF5 dataset file (e.g. server/data/dataset.hdf5)',
    )
    args = parser.parse_args()

    recorder = Recorder(None, base_path=args.dataset_path)

    print("=" * 60)
    print("SERL Recording Server - SIMPLIFIED (Local Recording Only)")
    print("=" * 60)
    print(f"Web UI:       http://localhost:5000")
    print(f"Dataset path: {args.dataset_path}")
    print(f"Recording:    Handled by teleop client locally")
    print("=" * 60)

    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)


if __name__ == '__main__':
    main()