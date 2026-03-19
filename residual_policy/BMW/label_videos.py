import cv2
from moviepy.video.io.VideoFileClip import VideoFileClip
import os 
import json 
from tqdm import tqdm

def replay_last_5_seconds(video_path):
    """
    Plays the last 5 seconds of the video in a loop until
    the user presses Space (returns True), Enter (returns False), 
    or Delete (returns 'DELETE').
    """
    # Load video
    video = VideoFileClip(video_path)
    duration = video.duration
    start_time = max(0, duration - 5)
    
    # Extract last 5 seconds as frames
    last_5 = video.subclipped(start_time, duration)
    
    # Resize to larger size for display
    last_5 = last_5.resized(height=720)  # keeps aspect ratio
    
    # Convert all frames to a list (for fast looping)
    frames = [frame[:, :, ::-1] for frame in last_5.iter_frames(fps=24, dtype='uint8')]
    # frame[:, :, ::-1] converts RGB → BGR for OpenCV
    
    # Display loop
    result = None
    while result is None:
        for frame in frames:
            cv2.imshow("SPACE=True, ENTER=False, DEL=Undo Previous", frame)
            
            key = cv2.waitKey(int(1000/24))  # wait ~1/fps seconds
            
            if key == 32:  # SPACE
                result = True
                break
            elif key == 13:  # ENTER
                result = False
                break
            elif key in [8, 127]:  # DELETE or BACKSPACE (Common codes: 8, 127)
                result = "DELETE"
                break
        
    cv2.destroyAllWindows()
    video.close()
    last_5.close()
    
    return result


def process_videos_in_folder(folder_path, json_path="video_results.json"):
    # Load existing results if file exists
    if os.path.exists(json_path):
        with open(json_path, "r") as f:
            results = json.load(f)
    else:
        results = {}
    
    # Find all mp4 files
    mp4_files = [f for f in os.listdir(folder_path) if f.endswith(".mp4")]
    
    # Track the order of files processed in this session to allow undoing
    processed_order = []

    # Use tqdm to wrap the iterator
    for file_name in tqdm(mp4_files, desc="Processing videos", unit="video"):
        # Skip already processed files (unless we just came back from an undo, 
        # but the logic below handles re-processing explicitly)
        if file_name in results and file_name not in [f for f in processed_order if results.get(f) is not None]:
            # Note: We skip only if it was processed in a previous session or earlier in this loop
            # and hasn't been flagged for re-edit via undo logic flow.
            # However, since 'processed_order' only grows, simple existence check is usually enough 
            # for the initial pass.
            continue
        
        full_path = os.path.join(folder_path, file_name)
        tqdm.write(f"Processing {file_name}...")
        
        result = replay_last_5_seconds(full_path)
        
        if result == "DELETE":
            if processed_order:
                # Get the previous file
                prev_file = processed_order.pop()
                prev_path = os.path.join(folder_path, prev_file)
                
                tqdm.write(f"\n>>> Undoing: Re-playing {prev_file} ...")
                
                # Re-run the labeling for the previous file
                new_result = replay_last_5_seconds(prev_path)
                
                if new_result != "DELETE": # Prevent infinite delete loops for now, or handle recursively if needed
                    # Update result
                    results[prev_file] = new_result
                    with open(json_path, "w") as f:
                        json.dump(results, f, indent=4)
                    tqdm.write(f"Updated {prev_file}: {new_result}")
                    
                    # If the user didn't delete again, we effectively fixed the previous one.
                    # The loop continues to the next file in the main tqdm loop.
                else:
                    # If they pressed delete again on the re-play, we pop another one (recursive undo)
                    # This requires a slightly more complex loop structure, but for simplicity:
                    tqdm.write("Double delete detected. Skipping further undo in this step.")
            else:
                tqdm.write("No previous episode to undo.")
            
            # Continue to the next file in the main loop without saving the current 'DELETE' signal
            continue

        # Save result immediately
        results[file_name] = result
        processed_order.append(file_name)
        
        with open(json_path, "w") as f:
            json.dump(results, f, indent=4)
        
        tqdm.write(f"Saved {file_name}: {result}")

# Path to your folder with MP4 videos
video_folder = "/home/qtf5422/Desktop/AIRE/residual-offpolicy-rl/arm=trossen_dual/task=entire_ecu_assembly/videos/chunk-000/observation.images.video"
process_videos_in_folder(video_folder)