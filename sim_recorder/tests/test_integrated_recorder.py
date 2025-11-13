#!/usr/bin/env python3
"""
Simple test for integrated_teleop_recorder.py
Tests basic functionality without requiring real hardware
"""

import numpy as np
import sys
from pathlib import Path

# Add parent to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from examples.integrated_teleop_recorder import TeleopRecorder


def test_camera_capture():
    """Test camera rendering (no hardware needed)"""
    import mujoco
    
    xml_path = Path(__file__).parent.parent.parent / "trossen_sim" / "trossen_sim" / "envs" / "xmls" / "trossen_ai_scene_joint.xml"
    
    if not xml_path.exists():
        print(f"✗ XML not found: {xml_path}")
        print("  Skipping camera test")
        return False
    
    print("Testing camera capture...")
    
    # Load model
    mj_model = mujoco.MjModel.from_xml_path(str(xml_path))
    mj_data = mujoco.MjData(mj_model)
    
    # Create renderer
    renderer = mujoco.Renderer(mj_model, height=128, width=128)
    
    # Test each camera
    cameras = ['cam_high', 'cam_low', 'cam_left_wrist', 'cam_right_wrist']
    for cam_name in cameras:
        try:
            renderer.update_scene(mj_data, camera=cam_name)
            pixels = renderer.render()
            
            assert pixels.shape == (128, 128, 3), f"Wrong shape: {pixels.shape}"
            assert pixels.dtype == np.uint8, f"Wrong dtype: {pixels.dtype}"
            
            print(f"  ✓ {cam_name}: {pixels.shape} {pixels.dtype}")
        except Exception as e:
            print(f"  ✗ {cam_name}: {e}")
            return False
    
    print("✓ Camera capture test passed")
    return True


def test_episode_save_load():
    """Test episode saving/loading"""
    import tempfile
    import json
    
    print("\nTesting episode save/load...")
    
    with tempfile.TemporaryDirectory() as tmpdir:
        # Create fake recorder
        recorder = TeleopRecorder(
            leader_ip='192.168.1.2',
            save_dir=tmpdir
        )
        
        # Skip initialization (no hardware)
        recorder.mj_model = None
        recorder.mj_data = None
        recorder.renderer = None
        
        # Create fake episode data
        recorder.start_recording("test_episode")
        
        # Add fake observations
        for i in range(10):
            obs = {
                'cam_high': np.random.randint(0, 255, (128, 128, 3), dtype=np.uint8),
                'cam_low': np.random.randint(0, 255, (128, 128, 3), dtype=np.uint8),
                'cam_left_wrist': np.random.randint(0, 255, (128, 128, 3), dtype=np.uint8),
                'cam_right_wrist': np.random.randint(0, 255, (128, 128, 3), dtype=np.uint8),
                'qpos': np.random.randn(8),
                'qvel': np.random.randn(8)
            }
            action = np.random.randn(7)
            
            recorder.current_episode_data['observations'].append(obs)
            recorder.current_episode_data['actions'].append(action)
        
        # Save
        episode_path = recorder.stop_recording()
        
        if episode_path is None:
            print("✗ Failed to save episode")
            return False
        
        episode_dir = Path(episode_path)
        
        # Check files exist
        assert (episode_dir / 'observations.npz').exists(), "observations.npz missing"
        assert (episode_dir / 'actions.npy').exists(), "actions.npy missing"
        assert (episode_dir / 'meta.json').exists(), "meta.json missing"
        
        print(f"  ✓ Episode saved to {episode_dir}")
        
        # Load and verify
        obs_npz = np.load(episode_dir / 'observations.npz')
        actions = np.load(episode_dir / 'actions.npy')
        
        with open(episode_dir / 'meta.json') as f:
            meta = json.load(f)
        
        assert obs_npz['cam_high'].shape == (10, 128, 128, 3), f"Wrong cam_high shape: {obs_npz['cam_high'].shape}"
        assert obs_npz['qpos'].shape == (10, 8), f"Wrong qpos shape: {obs_npz['qpos'].shape}"
        assert actions.shape == (10, 7), f"Wrong actions shape: {actions.shape}"
        assert meta['num_steps'] == 10, f"Wrong num_steps: {meta['num_steps']}"
        
        print("  ✓ Episode loaded and verified")
        print(f"    Cameras: {[k for k in obs_npz.keys() if k.startswith('cam_')]}")
        print(f"    States: qpos {obs_npz['qpos'].shape}, qvel {obs_npz['qvel'].shape}")
        print(f"    Actions: {actions.shape}")
    
    print("✓ Episode save/load test passed")
    return True


def test_robot_state_extraction():
    """Test robot state extraction from MuJoCo"""
    import mujoco
    
    xml_path = Path(__file__).parent.parent.parent / "trossen_sim" / "trossen_sim" / "envs" / "xmls" / "trossen_ai_scene_joint.xml"
    
    if not xml_path.exists():
        print("\n✗ XML not found, skipping robot state test")
        return False
    
    print("\nTesting robot state extraction...")
    
    # Load model
    mj_model = mujoco.MjModel.from_xml_path(str(xml_path))
    mj_data = mujoco.MjData(mj_model)
    
    # Step once
    mujoco.mj_step(mj_model, mj_data)
    
    # Extract robot state (first 8: 6 arm + 2 gripper)
    qpos = mj_data.qpos[:8].copy()
    qvel = mj_data.qvel[:8].copy()
    
    assert qpos.shape == (8,), f"Wrong qpos shape: {qpos.shape}"
    assert qvel.shape == (8,), f"Wrong qvel shape: {qvel.shape}"
    
    print(f"  ✓ qpos: {qpos.shape} {qpos.dtype}")
    print(f"  ✓ qvel: {qvel.shape} {qvel.dtype}")
    print(f"    qpos range: [{qpos.min():.3f}, {qpos.max():.3f}]")
    print(f"    qvel range: [{qvel.min():.3f}, {qvel.max():.3f}]")
    
    print("✓ Robot state extraction test passed")
    return True


def main():
    print("="*60)
    print("TESTING: integrated_teleop_recorder.py")
    print("="*60)
    
    tests = [
        ("Camera Capture", test_camera_capture),
        ("Episode Save/Load", test_episode_save_load),
        ("Robot State Extraction", test_robot_state_extraction)
    ]
    
    results = []
    for name, test_func in tests:
        try:
            result = test_func()
            results.append((name, result))
        except Exception as e:
            print(f"\n✗ {name} FAILED with exception: {e}")
            import traceback
            traceback.print_exc()
            results.append((name, False))
    
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    
    for name, result in results:
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"{status:8s} {name}")
    
    all_passed = all(result for _, result in results)
    
    if all_passed:
        print("\n✅ ALL TESTS PASSED")
        return 0
    else:
        print("\n❌ SOME TESTS FAILED")
        return 1


if __name__ == '__main__':
    exit(main())
