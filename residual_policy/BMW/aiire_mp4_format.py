import cv2

# Path to your video file
import subprocess
import json

def get_video_frame_size(video_path):
    """Returns the width and height of the video frames."""
    # Run ffprobe (comes with ffmpeg) to get video metadata as JSON
    cmd = [
        "ffprobe",
        "-v", "error",
        "-select_streams", "v:0",
        "-show_entries", "stream=width,height",
        "-of", "json",
        video_path
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    info = json.loads(result.stdout)
    
    width = info['streams'][0]['width']
    height = info['streams'][0]['height']
    
    return width, height

# Example usage
video_path = "/home/qtf5422/Desktop/AIRE/aiire_models/datasets/small_dataset/videos/chunk-000/video/episode_000059.mp4"
width, height = get_video_frame_size(video_path)
print(f"Frame size: {width} x {height}")
