import threading
import queue
import torch


class TrajectoryExecutor:
    """Handles asynchronous trajectory execution in a separate thread."""
    
    def __init__(self, env):
        self.env = env
        self.trajectory_queue = queue.Queue(maxsize=1)  # Only hold latest trajectory
        self.latest_ts = None
        self.lock = threading.Lock()
        self.running = True
        self.interrupt_flag = threading.Event()  # Signal to interrupt current trajectory
        
        # Start the execution thread
        self.thread = threading.Thread(target=self._execution_loop, daemon=True)
        self.thread.start()
    
    def _execution_loop(self):
        """Continuously executes trajectories from the queue."""
        while self.running:

            if self.latest_ts is not None : 
                terminal = self.latest_ts.last()
                if terminal : 
                    break

            try:
                # Wait for a trajectory (blocks until available)
                trajectory = self.trajectory_queue.get(timeout=0.1)
                
                # Clear the interrupt flag before starting new trajectory
                self.interrupt_flag.clear()
                
                # Execute the trajectory (can be interrupted)
                for idx_action in range(5, len(trajectory)):
                    # Check if we should interrupt
                    if self.interrupt_flag.is_set():
                        print("Trajectory interrupted by new trajectory")
                        break
                    
                    print(f"trajectory stepping: {idx_action}")
                    print("stepping with action : ", trajectory[idx_action][7:10])
                    ts = self.env.step(trajectory[idx_action])

                    
                    # Store the latest timestep
                    with self.lock:
                        self.latest_ts = ts
                        
            except queue.Empty:
                continue
    
    def send_trajectory(self, trajectory):
        """Send a new trajectory for execution, interrupting any current execution."""
        # Signal to interrupt current trajectory execution
        self.interrupt_flag.set()
        
        # Clear old trajectory if present and add new one
        try:
            self.trajectory_queue.get_nowait()  # Remove old trajectory
        except queue.Empty:
            pass
        
        self.trajectory_queue.put(trajectory)
    
    def get_latest_ts(self):
        """Get the most recent timestep."""
        with self.lock:
            return self.latest_ts
    
    def stop(self):
        """Stop the execution thread."""
        self.running = False
        self.thread.join()



