"""
Teleop Listener - ZeroMQ subscriber for teleop actions
"""

import zmq
import numpy as np
import threading
import json
from typing import Optional


class TeleopListener:
    """Listen for teleop actions via ZeroMQ"""
    
    def __init__(self, port=5555):
        self.port = port
        self.latest_action = None
        self.running = False
        self.thread = None
        
        # Setup ZeroMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.bind(f"tcp://*:{port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.socket.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout
    
    def start(self):
        """Start listening thread"""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._listen_loop, daemon=True)
            self.thread.start()
            print(f"✓ Teleop listener started on tcp://*:{self.port}")
    
    def stop(self):
        """Stop listening"""
        self.running = False
        if self.thread:
            self.thread.join()
    
    def _listen_loop(self):
        """Background thread that receives actions"""
        while self.running:
            try:
                message = self.socket.recv_string()
                data = json.loads(message)
                self.latest_action = np.array(data['action'])
            except zmq.Again:
                # Timeout, continue
                pass
            except Exception as e:
                print(f"Teleop listener error: {e}")
    
    def get_latest_action(self) -> Optional[np.ndarray]:
        """Get most recent action"""
        return self.latest_action
    
    def set_action(self, action: np.ndarray):
        """Manually set action (for HTTP endpoint)"""
        self.latest_action = action
