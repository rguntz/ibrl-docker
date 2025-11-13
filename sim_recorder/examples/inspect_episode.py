#!/usr/bin/env python3
"""
Inspect recorded episodes from integrated_teleop_recorder
Shows episode info, plays back observations, visualizes cameras
"""

import numpy as np
import json
from pathlib import Path
import argparse
import cv2
import time


def print_episode_info(episode_dir: Path):
    """Print episode metadata and shapes"""
    print(f"\n{'='*60}")
    print(f"EPISODE: {episode_dir.name}")
    print(f"{'='*60}")
    
    # Load metadata
    with open(episode_dir / 'meta.json') as f:
        meta = json.load(f)
    
    print(f"\n📊 Metadata:")
    print(f"  Episode name: {meta['episode_name']}")
    print(f"  Steps: {meta['num_steps']}")
    print(f"  Duration: {meta['duration']:.2f}s")
    print(f"  FPS: {meta['num_steps'] / meta['duration']:.1f}")
    print(f"  Cameras: {', '.join(meta['cameras'])}")
    print(f"  Resolution: {meta['camera_resolution']}")
    print(f"  Robot: {meta['robot_model']}")
    
    # Load observations
    obs = np.load(episode_dir / 'observations.npz')
    
    print(f"\n📷 Observations:")
    for key in obs.keys():
        data = obs[key]
        print(f"  {key:20s}: {str(data.shape):20s} {data.dtype}")
    
    # Load actions
    actions = np.load(episode_dir / 'actions.npy')
    print(f"\n🎮 Actions:")
    print(f"  actions: {str(actions.shape):20s} {actions.dtype}")
    
    # Statistics
    print(f"\n📈 Action Statistics:")
    for i in range(actions.shape[1]):
        joint_actions = actions[:, i]
        print(f"  Joint {i}: min={joint_actions.min():.3f}, max={joint_actions.max():.3f}, "
              f"mean={joint_actions.mean():.3f}, std={joint_actions.std():.3f}")
    
    print(f"\n📈 Robot State Statistics:")
    qpos = obs['qpos']
    qvel = obs['qvel']
    print(f"  qpos range: [{qpos.min():.3f}, {qpos.max():.3f}]")
    print(f"  qvel range: [{qvel.min():.3f}, {qvel.max():.3f}]")
    
    print(f"\n{'='*60}\n")


def play_episode_video(episode_dir: Path, 
                       camera: str = 'cam_high',
                       fps: float = 20.0,
                       loop: bool = False):
    """Play episode as video"""
    # Load observations
    obs = np.load(episode_dir / 'observations.npz')
    
    if camera not in obs:
        print(f"✗ Camera '{camera}' not found in episode")
        print(f"  Available: {list(obs.keys())}")
        return
    
    images = obs[camera]
    num_frames = len(images)
    
    print(f"\n▶️  Playing {camera}: {num_frames} frames @ {fps} FPS")
    print(f"   Press 'q' to quit, 'space' to pause")
    
    window_name = f"{episode_dir.name} - {camera}"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    
    paused = False
    frame_idx = 0
    
    while True:
        if not paused:
            # Get frame
            frame = images[frame_idx]
            
            # Convert RGB to BGR for OpenCV
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Add frame info
            text = f"Frame {frame_idx}/{num_frames-1}"
            cv2.putText(frame_bgr, text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Show
            cv2.imshow(window_name, frame_bgr)
            
            # Advance
            frame_idx = (frame_idx + 1) % num_frames
            
            # Reset if not looping
            if frame_idx == 0 and not loop:
                print("✓ Playback complete")
                break
        
        # Wait
        key = cv2.waitKey(int(1000 / fps))
        
        if key == ord('q') or key == 27:  # q or ESC
            break
        elif key == ord(' '):  # space
            paused = not paused
            status = "PAUSED" if paused else "PLAYING"
            print(f"  {status}")
        elif key == ord('r'):  # r to restart
            frame_idx = 0
            print("  RESTART")
    
    cv2.destroyAllWindows()


def show_all_cameras(episode_dir: Path, frame_idx: int = 0):
    """Show all cameras in a grid"""
    obs = np.load(episode_dir / 'observations.npz')
    
    # Get camera images
    cam_names = [k for k in obs.keys() if k.startswith('cam_')]
    
    if not cam_names:
        print("✗ No cameras found")
        return
    
    print(f"\n📷 Showing frame {frame_idx} from all cameras")
    
    # Create grid
    images = []
    for cam_name in sorted(cam_names):
        img = obs[cam_name][frame_idx]
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        
        # Add label
        cv2.putText(img_bgr, cam_name, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        images.append(img_bgr)
    
    # Arrange in grid (2x2)
    if len(images) >= 4:
        top_row = np.hstack([images[0], images[1]])
        bottom_row = np.hstack([images[2], images[3]])
        grid = np.vstack([top_row, bottom_row])
    elif len(images) == 2:
        grid = np.hstack(images)
    else:
        grid = images[0]
    
    window_name = f"{episode_dir.name} - All Cameras"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.imshow(window_name, grid)
    
    print("  Press any key to close...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def list_episodes(input_dir: Path):
    """List all episodes in directory"""
    episode_dirs = sorted([d for d in input_dir.iterdir() if d.is_dir()])
    
    if not episode_dirs:
        print(f"No episodes found in {input_dir}")
        return
    
    print(f"\n📂 Episodes in {input_dir}:")
    print(f"{'='*80}")
    
    for episode_dir in episode_dirs:
        try:
            with open(episode_dir / 'meta.json') as f:
                meta = json.load(f)
            
            print(f"  {episode_dir.name:40s} | {meta['num_steps']:5d} steps | "
                  f"{meta['duration']:6.1f}s | {meta['num_steps']/meta['duration']:5.1f} FPS")
        except:
            print(f"  {episode_dir.name:40s} | (error loading)")
    
    print(f"{'='*80}")
    print(f"Total: {len(episode_dirs)} episodes\n")


def main():
    parser = argparse.ArgumentParser(description='Inspect recorded episodes')
    parser.add_argument('path', type=str, help='Episode directory or parent directory')
    parser.add_argument('--list', action='store_true', help='List all episodes in directory')
    parser.add_argument('--play', action='store_true', help='Play episode as video')
    parser.add_argument('--camera', default='cam_high', help='Camera to play')
    parser.add_argument('--fps', type=float, default=20.0, help='Playback FPS')
    parser.add_argument('--loop', action='store_true', help='Loop playback')
    parser.add_argument('--show-cameras', action='store_true', help='Show all cameras')
    parser.add_argument('--frame', type=int, default=0, help='Frame index for --show-cameras')
    
    args = parser.parse_args()
    
    path = Path(args.path)
    
    if not path.exists():
        print(f"✗ Path not found: {path}")
        return 1
    
    # Check if it's an episode directory (has meta.json) or parent directory
    if (path / 'meta.json').exists():
        # Single episode
        if args.list:
            print("✗ --list only works with parent directory")
            return 1
        
        if not args.play and not args.show_cameras:
            print_episode_info(path)
        
        if args.play:
            play_episode_video(path, camera=args.camera, fps=args.fps, loop=args.loop)
        
        if args.show_cameras:
            show_all_cameras(path, frame_idx=args.frame)
    
    else:
        # Parent directory
        if args.list or not (args.play or args.show_cameras):
            list_episodes(path)
        else:
            print("✗ Please specify an episode directory to play/show")
            return 1
    
    return 0


if __name__ == '__main__':
    exit(main())
