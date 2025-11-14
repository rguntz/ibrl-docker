// Main JavaScript for SERL Recorder UI

let isRecording = false;
let statusInterval = null;

// Initialize on page load
document.addEventListener('DOMContentLoaded', () => {
    loadConfig();
    loadEpisodes();
    startStatusPolling();
});

async function loadConfig() {
    try {
        const response = await fetch('/api/config');
        const config = await response.json();
        
        // Set FPS limits from server config
        const fpsInput = document.getElementById('fps');
        fpsInput.min = 1;
        fpsInput.max = config.max_recording_fps;
        fpsInput.value = config.default_recording_fps;
        
    } catch (error) {
        console.error('Failed to load config:', error);
    }
}

function startStatusPolling() {
    // Poll status every 500ms
    statusInterval = setInterval(updateStatus, 500);
}

async function updateStatus() {
    try {
        const response = await fetch('/api/status');
        const data = await response.json();
        
        isRecording = data.recording;
        
        // Update warmup button state
        const warmupBtn = document.getElementById('warmup-btn');
        if (data.warmup_requested && !data.warmup_completed) {
            // Warmup requested but not completed - show in progress
            warmupBtn.disabled = true;
            warmupBtn.textContent = 'Warm-up Running...';
        } else if (data.teleop_connected && !data.warmup_completed) {
            // Teleop connected but warmup not completed - enable warmup button
            warmupBtn.disabled = false;
            warmupBtn.textContent = 'Start Warm-up';
        } else if (data.warmup_completed) {
            // Warmup completed - disable warmup button
            warmupBtn.disabled = true;
            warmupBtn.textContent = 'Warm-up Complete';
        } else {
            // No teleop connection - disable warmup button
            warmupBtn.disabled = true;
            warmupBtn.textContent = 'Start Warm-up';
        }
        
        if (isRecording) {
            indicator.className = 'status-indicator recording';
            statusText.textContent = 'Recording';
            warmupStatus.style.display = 'none';
            startBtn.disabled = true;
            stopBtn.disabled = false;
            recordingInfo.style.display = 'block';
            
            document.getElementById('current-episode').textContent = data.current_episode;
            document.getElementById('num-steps').textContent = data.num_steps;
        } else if (data.warmup_completed) {
            indicator.className = 'status-indicator ready';
            statusText.textContent = 'Ready';
            warmupStatus.style.display = 'none';
            startBtn.disabled = false;
            stopBtn.disabled = true;
            recordingInfo.style.display = 'none';
        } else if (data.teleop_connected) {
            indicator.className = 'status-indicator warmup';
            statusText.textContent = 'Teleop Connected';
            warmupStatus.style.display = 'inline';
            startBtn.disabled = true;
            stopBtn.disabled = true;
            recordingInfo.style.display = 'none';
        } else {
            indicator.className = 'status-indicator idle';
            statusText.textContent = 'Waiting for teleop';
            warmupStatus.style.display = 'none';
            startBtn.disabled = true;
            stopBtn.disabled = true;
            recordingInfo.style.display = 'none';
        }
        
    } catch (error) {
        console.error('Failed to update status:', error);
    }
}

function updateCameraViews(activeCameras) {
    const cameraGrid = document.getElementById('camera-grid');
    
    // Clear existing views
    cameraGrid.innerHTML = '';
    
    // Camera name mapping
    const cameraNames = {
        0: 'Cam High',
        1: 'Cam Low', 
        2: 'Cam Left Wrist',
        3: 'Cam Right Wrist'
    };
    
    // Create views for active cameras
    activeCameras.forEach(camId => {
        const cameraView = document.createElement('div');
        cameraView.className = 'camera-view';
        
        const title = document.createElement('h3');
        title.textContent = cameraNames[camId] || `Camera ${camId}`;
        
        const img = document.createElement('img');
        img.id = `cam-${camId}`;
        img.src = `/stream_cam/${camId}`;
        img.alt = `Camera ${camId}`;
        
        cameraView.appendChild(title);
        cameraView.appendChild(img);
        cameraGrid.appendChild(cameraView);
    });
    
    // If no cameras active, show message
    if (activeCameras.length === 0) {
        cameraGrid.innerHTML = '<p style="text-align: center; padding: 2rem; color: #666;">No cameras active</p>';
    }
}

async function startWarmup() {
    const warmupBtn = document.getElementById('warmup-btn');
    const startBtn = document.getElementById('start-btn');
    
    try {
        warmupBtn.disabled = true;
        warmupBtn.textContent = 'Triggering Warm-up...';
        
        // Send warmup trigger request to server
        const response = await fetch('/api/warmup/trigger', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            }
        });
        
        const result = await response.json();
        
        if (result.success) {
            warmupBtn.textContent = 'Warm-up Triggered';
            alert('Warm-up command sent to teleop client. The sequence will start shortly.');
        } else {
            alert('Failed to trigger warm-up: ' + result.error);
            warmupBtn.disabled = false;
            warmupBtn.textContent = 'Start Warm-up';
        }
        
    } catch (error) {
        console.error('Failed to trigger warmup:', error);
        alert('Failed to trigger warm-up: ' + error.message);
        warmupBtn.disabled = false;
        warmupBtn.textContent = 'Start Warm-up';
    }
}

async function startRecording() {
    // First check if warmup is completed
    try {
        const statusResponse = await fetch('/api/status');
        const statusData = await statusResponse.json();
        
        if (!statusData.warmup_completed) {
            alert('Please complete warm-up first. Start the teleop client to perform warm-up.');
            return;
        }
    } catch (error) {
        console.error('Failed to check warmup status:', error);
        alert('Unable to verify warm-up status. Please try again.');
        return;
    }
    
    const episodeName = document.getElementById('episode-name').value;
    const fps = parseInt(document.getElementById('fps').value) || 15;
    
    try {
        const response = await fetch('/api/start', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                episode_name: episodeName || undefined,
                fps: fps
            })
        });
        
        const data = await response.json();
        
        if (data.success) {
            console.log('Recording started:', data.episode_name);
            document.getElementById('episode-name').value = '';
        } else {
            alert('Failed to start recording');
        }
        
    } catch (error) {
        console.error('Failed to start recording:', error);
        alert('Error starting recording');
    }
}

async function stopRecording() {
    try {
        const response = await fetch('/api/stop', {
            method: 'POST'
        });
        
        const data = await response.json();
        
        if (data.success) {
            console.log('Recording stopped:', data.episode_path);
            loadEpisodes();
            
            // Reset warmup status so user needs to warmup before next recording
            await fetch('/api/warmup/reset', { method: 'POST' });
        } else {
            alert('Failed to stop recording');
        }
        
    } catch (error) {
        console.error('Failed to stop recording:', error);
        alert('Error stopping recording');
    }
}

async function loadEpisodes() {
    try {
        const response = await fetch('/api/list');
        const data = await response.json();
        
        const episodesList = document.getElementById('episodes-list');
        episodesList.innerHTML = '';
        
        if (data.episodes.length === 0) {
            episodesList.innerHTML = '<p class="no-episodes">No episodes recorded yet</p>';
            return;
        }
        
        data.episodes.forEach(episode => {
            const episodeDiv = document.createElement('div');
            episodeDiv.className = 'episode-item';
            episodeDiv.innerHTML = `
                <div class="episode-info">
                    <strong>${episode.name}</strong>
                    <span class="episode-meta">${episode.num_steps} steps, ${episode.duration.toFixed(1)}s</span>
                </div>
                <button class="btn btn-danger btn-small" onclick="deleteEpisode('${episode.id}')">Delete</button>
            `;
            episodesList.appendChild(episodeDiv);
        });
        
    } catch (error) {
        console.error('Failed to load episodes:', error);
    }
}

async function deleteEpisode(episodeId) {
    if (!confirm(`Delete episode ${episodeId}?`)) {
        return;
    }
    
    try {
        const response = await fetch('/api/delete', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ episode_id: episodeId })
        });
        
        const data = await response.json();
        
        if (data.success) {
            console.log('Episode deleted:', episodeId);
            loadEpisodes();
        } else {
            alert('Failed to delete episode');
        }
        
    } catch (error) {
        console.error('Failed to delete episode:', error);
        alert('Error deleting episode');
    }
}
