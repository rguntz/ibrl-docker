// Main JavaScript for SERL Recorder UI

let isRecording = false;
let statusInterval = null;

// Initialize on page load
document.addEventListener('DOMContentLoaded', () => {
    loadEpisodes();
    startStatusPolling();
});

function startStatusPolling() {
    // Poll status every 500ms
    statusInterval = setInterval(updateStatus, 500);
}

async function updateStatus() {
    try {
        const response = await fetch('/api/status');
        const data = await response.json();
        
        isRecording = data.recording;
        
        // Update UI
        const indicator = document.getElementById('status-indicator');
        const statusText = document.getElementById('status-text');
        const startBtn = document.getElementById('start-btn');
        const stopBtn = document.getElementById('stop-btn');
        const recordingInfo = document.getElementById('recording-info');
        
        if (isRecording) {
            indicator.className = 'status-indicator recording';
            statusText.textContent = 'Recording';
            startBtn.disabled = true;
            stopBtn.disabled = false;
            recordingInfo.style.display = 'block';
            
            document.getElementById('current-episode').textContent = data.current_episode;
            document.getElementById('num-steps').textContent = data.num_steps;
        } else {
            indicator.className = 'status-indicator idle';
            statusText.textContent = 'Idle';
            startBtn.disabled = false;
            stopBtn.disabled = true;
            recordingInfo.style.display = 'none';
        }
        
    } catch (error) {
        console.error('Failed to update status:', error);
    }
}

async function startRecording() {
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
