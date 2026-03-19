import os
from tqdm import tqdm
import json 
import os
import pandas as pd
from pathlib import Path
import pandas as pd
import os
import glob
import pandas as pd
import json
from pathlib import Path
import numpy as np
import os
from pathlib import Path
import pandas as pd
from typing import Optional, Tuple, List
import argparse

DEFAULT_BAD_EPISODES = [
    1863, 1864, 1866, 1868, 1871,
    1853, 1855, 1859, 1860,
    1863, 1866,
]

def is_mp4_corrupted(file_path):
    """Check MP4 more safely by looking at both start and end of file."""
    try:
        size = os.path.getsize(file_path)
        with open(file_path, "rb") as f:
            start = f.read(1024*1024)  # first 1 MB
            f.seek(max(0, size - 1024*1024))
            end = f.read(1024*1024)  # last 1 MB
            return b"moov" not in start and b"moov" not in end
    except Exception:
        return True

def make_list_corrupted_episodes(base_dir, video_subdirs): 

    # Collect all video files first for a single tqdm bar
    all_videos = []
    for chunk_name in os.listdir(base_dir):
        chunk_path = os.path.join(base_dir, chunk_name)
        if not os.path.isdir(chunk_path):
            continue
        for subdir in video_subdirs:
            video_dir = os.path.join(chunk_path, subdir)
            if not os.path.isdir(video_dir):
                print("video_dir : ",video_dir)
                continue
            for f in os.listdir(video_dir):
                if f.endswith(".mp4"):
                    all_videos.append(os.path.join(video_dir, f))

    corrupted_files = []

    # Check all videos with tqdm progress bar
    for file_path in tqdm(all_videos, desc="Checking MP4s"):
        if is_mp4_corrupted(file_path):
            corrupted_files.append(file_path)

    print("number of corrupted files : ", len(corrupted_files))
    episode_indices = [os.path.basename(f).replace(".mp4", "") for f in corrupted_files]
    print("\nCorrupted videos:")
    for index in episode_indices:
        print(index)

    return episode_indices

def remove_problematic_episodes(corrupted_episodes, data_base, video_base, video_subdirs) : 
    print("removing the problematic episodes")

    import os

    # Base directories
    video_base = video_base

    # Remove corrupted data parquet files
    for chunk in os.listdir(data_base):
        chunk_path = os.path.join(data_base, chunk)
        if not os.path.isdir(chunk_path):
            print("chunk_path not found 1 : ", chunk_path)
            continue
        for ep in corrupted_episodes:
            print("ep : ", ep)
            parquet_file = os.path.join(chunk_path, f"{ep}.parquet")
            if os.path.exists(parquet_file):
                os.remove(parquet_file)
                print(f"Removed data file: {parquet_file}")
            else : 
                print("file not found")

    # Remove corrupted videos in all subfolders
    for chunk in os.listdir(video_base):
        chunk_path = os.path.join(video_base, chunk)
        if not os.path.isdir(chunk_path):
            print("chunk_path not found : ", chunk_path)
            continue
        for subdir in video_subdirs:
            video_dir = os.path.join(chunk_path, subdir)
            if not os.path.isdir(video_dir):
                print("video_dir not found : ", video_dir)
                continue
            for ep in corrupted_episodes:
                mp4_file = os.path.join(video_dir, f"{ep}.mp4")
                if os.path.exists(mp4_file):
                    os.remove(mp4_file)
                    print(f"Removed video file: {mp4_file}")

def remove_corrupted_episodes_from_metadata(meta_dir, corrupted_episodes):
    """
    Remove lines from metadata JSONL files corresponding to corrupted episodes.
    
    Args:
        meta_dir (str): Path to meta folder.
        corrupted_episodes (list): List of corrupted episodes in the form 'episode_001863'.
    """
    # Convert episode names to integer indices
    corrupted_indices = [int(ep.replace("episode_", "")) for ep in corrupted_episodes]
    print(f"Removing indices from metadata: {corrupted_indices}")

    # Metadata files to clean
    jsonl_files = [
        "episodes.jsonl",
        "episodes_stats.jsonl",
        "episodes_custom_metadata.jsonl"
    ]

    for file_name in jsonl_files:
        file_path = os.path.join(meta_dir, file_name)
        if not os.path.exists(file_path):
            print("file not found : ", file_path)
            continue

        # Read all lines, keep only those not corrupted
        new_lines = []
        with open(file_path, "r") as f:
            for line in f:
                try:
                    data = json.loads(line)
                    ep_idx = data.get("episode_index")
                    if ep_idx not in corrupted_indices:
                        new_lines.append(line)
                except json.JSONDecodeError:
                    # Keep the line if JSON is invalid (optional)
                    new_lines.append(line)
                    print("error json")

        # Overwrite file with cleaned lines
        with open(file_path, "w") as f:
            f.writelines(new_lines)

        print(f"Updated {file_name}, removed {len(corrupted_indices)} episodes")

def reindex_metadata_jsonl(meta_dir):
    """
    Reindex episode_index sequentially starting from 0 in all JSONL metadata files.
    
    Args:
        meta_dir (str): Path to the meta folder.
    """
    jsonl_files = [
        "episodes.jsonl",
        "episodes_stats.jsonl",
        "episodes_custom_metadata.jsonl"
    ]

    for file_name in jsonl_files:
        file_path = os.path.join(meta_dir, file_name)
        if not os.path.exists(file_path):
            continue

        # Read all lines as JSON objects
        data_list = []
        with open(file_path, "r") as f:
            for line in f:
                try:
                    data = json.loads(line)
                    data_list.append(data)
                except json.JSONDecodeError:
                    continue  # skip invalid lines

        # Reassign episode_index sequentially
        for new_idx, data in enumerate(data_list):
            data["episode_index"] = new_idx

        # Write back to file
        with open(file_path, "w") as f:
            for data in data_list:
                f.write(json.dumps(data) + "\n")

        print(f"Reindexed {file_name}, total episodes: {len(data_list)}")


def reindex_data_and_videos(data_base, video_base, video_subdirs):
    """
    Reindex all remaining data parquet files and video files sequentially
    so that episode indices are continuous starting from 0.
    """
    # --- Step 1: Reindex data parquet files ---
    for chunk in os.listdir(data_base):
        chunk_path = os.path.join(data_base, chunk)
        if not os.path.isdir(chunk_path):
            continue

        # Get all remaining parquet files sorted
        parquet_files = sorted([f for f in os.listdir(chunk_path) if f.endswith(".parquet")])
        for new_idx, old_file in enumerate(parquet_files):
            old_path = os.path.join(chunk_path, old_file)
            new_file = f"episode_{new_idx:06d}.parquet"
            new_path = os.path.join(chunk_path, new_file)
            os.rename(old_path, new_path)
            print(f"Renamed {old_file} -> {new_file}")

    # --- Step 2: Reindex video files in all subfolders ---
    for chunk in os.listdir(video_base):
        chunk_path = os.path.join(video_base, chunk)
        if not os.path.isdir(chunk_path):
            continue
        for subdir in video_subdirs:
            video_dir = os.path.join(chunk_path, subdir)
            if not os.path.isdir(video_dir):
                continue

            # Get all remaining video files sorted
            video_files = sorted([f for f in os.listdir(video_dir) if f.endswith(".mp4")])
            for new_idx, old_file in enumerate(video_files):
                old_path = os.path.join(video_dir, old_file)
                new_file = f"episode_{new_idx:06d}.mp4"
                new_path = os.path.join(video_dir, new_file)
                os.rename(old_path, new_path)
                print(f"Renamed {subdir}/{old_file} -> {new_file}")




def count_total_steps(data_base):
    """
    Count total steps in all parquet files under data_base.
    
    Args:
        data_base (str): Path to the 'data' folder containing chunks.
    
    Returns:
        int: Total number of steps across all episodes.
    """
    total_steps = 0

    for chunk in os.listdir(data_base):
        chunk_path = os.path.join(data_base, chunk)
        if not os.path.isdir(chunk_path):
            continue

        for file in os.listdir(chunk_path):
            if file.endswith(".parquet"):
                parquet_path = os.path.join(chunk_path, file)
                try:
                    df = pd.read_parquet(parquet_path)
                    # Assuming 'length' column contains number of steps
                    if 'length' in df.columns:
                        total_steps += df['length'].sum()
                    else:
                        # If dataframe itself is one row per step, count rows
                        total_steps += len(df)
                except Exception as e:
                    print(f"Error reading {parquet_path}: {e}")

    return total_steps


def count_total_videos(video_base, video_subdirs):
    """
    Count total number of video files in all subfolders of video_base.

    Args:
        video_base (str): Path to the 'videos' folder.
        video_subdirs (list): List of video subfolders (e.g., ['video', 'video_2', ...]).

    Returns:
        int: Total number of videos.
    """
    total_videos = 0

    for chunk in os.listdir(video_base):
        chunk_path = os.path.join(video_base, chunk)
        if not os.path.isdir(chunk_path):
            continue

        for subdir in video_subdirs:
            video_dir = os.path.join(chunk_path, subdir)
            if not os.path.isdir(video_dir):
                continue

            # Count all .mp4 files in this subfolder
            mp4_files = [f for f in os.listdir(video_dir) if f.endswith(".mp4")]
            total_videos += len(mp4_files)

    return total_videos


def count_total_parquet_files(data_base):
    """
    Count total number of .parquet files in all chunks of the data folder.

    Args:
        data_base (str): Path to the 'data' folder.

    Returns:
        int: Total number of .parquet files.
    """
    total_files = 0

    for chunk in os.listdir(data_base):
        chunk_path = os.path.join(data_base, chunk)
        if not os.path.isdir(chunk_path):
            continue

        # Count all .parquet files in this chunk
        parquet_files = [f for f in os.listdir(chunk_path) if f.endswith(".parquet")]
        total_files += len(parquet_files)

    return total_files


import os
import json

def check_metadata_indices(meta_dir, expected_count=2012):
    """
    Check if episode indices in JSONL metadata files are present and sequential.
    
    Args:
        meta_dir (str): Path to the 'meta' folder.
        expected_count (int): Expected total number of episodes.
        
    Returns:
        dict: A dictionary with file names as keys and True/False for validity.
    """
    jsonl_files = [
        "episodes.jsonl",
        "episodes_stats.jsonl",
        "episodes_custom_metadata.jsonl"
    ]

    results = {}

    for file_name in jsonl_files:
        file_path = os.path.join(meta_dir, file_name)
        if not os.path.exists(file_path):
            results[file_name] = False
            continue

        indices = []
        with open(file_path, "r") as f:
            for line in f:
                try:
                    data = json.loads(line)
                    idx = data.get("episode_index")
                    if idx is not None:
                        indices.append(idx)
                except json.JSONDecodeError:
                    continue  # skip invalid lines

        # Check if indices start at 0 and increase by 1
        is_sequential = indices == list(range(len(indices)))
        has_correct_count = len(indices) == expected_count

        results[file_name] = is_sequential and has_correct_count

    return results

import os
import json

def debug_metadata_indices(meta_dir, expected_count=2012):
    """
    Debug metadata JSONL files to find missing or unordered episode indices.
    
    Args:
        meta_dir (str): Path to the 'meta' folder.
        expected_count (int): Expected number of episodes.
        
    Returns:
        None. Prints debug info.
    """
    jsonl_files = [
        "episodes.jsonl",
        "episodes_stats.jsonl",
        "episodes_custom_metadata.jsonl"
    ]

    for file_name in jsonl_files:
        file_path = os.path.join(meta_dir, file_name)
        if not os.path.exists(file_path):
            print(f"{file_name} is missing!")
            continue

        indices = []
        with open(file_path, "r") as f:
            for line in f:
                try:
                    data = json.loads(line)
                    idx = data.get("episode_index")
                    if idx is not None:
                        indices.append(idx)
                except json.JSONDecodeError:
                    continue

        if not indices:
            print(f"{file_name} has no valid episode_index!")
            continue

        # Check for sequential order
        missing_indices = sorted(set(range(indices[0], indices[-1]+1)) - set(indices))
        first_non_sequential = next((i for i, v in enumerate(indices) if v != i), None)

        print(f"\nFile: {file_name}")
        print(f"Total episodes found: {len(indices)}")
        print(f"Expected total: {expected_count}")
        if missing_indices:
            print(f"Missing indices: {missing_indices}")
        else:
            print("No missing indices.")

        if first_non_sequential is not None and first_non_sequential < len(indices):
            print(f"First non-sequential index at line {first_non_sequential}: {indices[first_non_sequential]}")
        else:
            print("All indices sequential starting from 0.")


def check_metadata_indices_2(meta_dir):
    """
    Check if episode indices in JSONL metadata files are sequential starting from 0.
    
    Returns True if sequential, False otherwise.
    """
    jsonl_files = [
        "episodes.jsonl",
        "episodes_stats.jsonl",
        "episodes_custom_metadata.jsonl"
    ]

    results = {}

    for file_name in jsonl_files:
        file_path = os.path.join(meta_dir, file_name)
        if not os.path.exists(file_path):
            results[file_name] = False
            continue

        indices = []
        with open(file_path, "r") as f:
            for line in f:
                try:
                    data = json.loads(line)
                    idx = data.get("episode_index")
                    if idx is not None:
                        indices.append(idx)
                except json.JSONDecodeError:
                    continue

        # Check if indices are sequential starting from 0
        is_sequential = indices == list(range(len(indices)))
        results[file_name] = is_sequential

    return results

import os

def check_parquet_indices(data_base):
    """
    Check if all .parquet files in data folder are sequentially indexed starting from 0.

    Args:
        data_base (str): Path to the 'data' folder.

    Returns:
        dict: {chunk_name: True/False} indicating if each chunk is sequential.
    """
    results = {}

    for chunk in os.listdir(data_base):
        chunk_path = os.path.join(data_base, chunk)
        if not os.path.isdir(chunk_path):
            continue

        # List all parquet files and sort
        parquet_files = sorted([f for f in os.listdir(chunk_path) if f.endswith(".parquet")])

        # Extract numeric indices
        indices = []
        for f in parquet_files:
            try:
                idx = int(f.replace("episode_", "").replace(".parquet", ""))
                indices.append(idx)
            except ValueError:
                continue

        # Check sequential
        is_sequential = indices == list(range(len(indices)))
        results[chunk] = is_sequential

        if not is_sequential:
            print(f"Chunk {chunk} parquet indices are NOT sequential: {indices}")

    return results

def check_video_indices(video_base, video_subdirs):
    """
    Check if all .mp4 files in video folders are sequentially indexed starting from 0.

    Args:
        video_base (str): Path to 'videos' folder.
        video_subdirs (list): List of video subfolders (['video', 'video_2', ...]).

    Returns:
        dict: {(chunk, subdir): True/False} indicating sequential check.
    """
    results = {}

    for chunk in os.listdir(video_base):
        chunk_path = os.path.join(video_base, chunk)
        if not os.path.isdir(chunk_path):
            continue

        for subdir in video_subdirs:
            video_dir = os.path.join(chunk_path, subdir)
            if not os.path.isdir(video_dir):
                continue

            # List all .mp4 files and sort
            video_files = sorted([f for f in os.listdir(video_dir) if f.endswith(".mp4")])

            # Extract numeric indices
            indices = []
            for f in video_files:
                try:
                    idx = int(f.replace("episode_", "").replace(".mp4", ""))
                    indices.append(idx)
                except ValueError:
                    continue

            # Check sequential
            is_sequential = indices == list(range(len(indices)))
            results[(chunk, subdir)] = is_sequential

            if not is_sequential:
                print(f"Chunk {chunk}, subdir {subdir} video indices are NOT sequential: {indices}")

    return results

import json
from typing import Dict, List, Tuple


def validate_episode_indices(
    file_path: str,
    start_index: int = 0,
    end_index: int = 2111,
) -> Dict[str, object]:
    """
    Validate that episode_index values in a JSONL file form a consecutive
    sequence from start_index to end_index (inclusive).

    Parameters
    ----------
    file_path : str
        Path to the JSONL file.
    start_index : int, optional
        Expected starting index (default: 0).
    end_index : int, optional
        Expected ending index (default: 2111).

    Returns
    -------
    Dict[str, object]
        Dictionary containing validation results:
        - is_valid (bool): True if indices are consecutive and complete
        - total_found (int): Number of episode indices found
        - missing_indices (List[int]): Missing indices, if any
        - extra_indices (List[int]): Unexpected indices, if any
        - breaks (List[Tuple[int, int]]): Non-consecutive index pairs
    """
    expected_indices = list(range(start_index, end_index + 1))
    found_indices: List[int] = []

    with open(file_path, "r", encoding="utf-8") as file:
        for line_number, line in enumerate(file, start=1):
            data = json.loads(line)
            if "episode_index" not in data:
                raise KeyError(
                    f"Missing 'episode_index' at line {line_number}"
                )
            found_indices.append(data["episode_index"])

    found_indices_sorted = sorted(found_indices)

    missing_indices = sorted(set(expected_indices) - set(found_indices_sorted))
    extra_indices = sorted(set(found_indices_sorted) - set(expected_indices))

    breaks = [
        (found_indices_sorted[i], found_indices_sorted[i + 1])
        for i in range(len(found_indices_sorted) - 1)
        if found_indices_sorted[i + 1] != found_indices_sorted[i] + 1
    ]

    is_valid = (
        not missing_indices
        and not extra_indices
        and not breaks
        and found_indices_sorted[0] == start_index
        and found_indices_sorted[-1] == end_index
    )

    return {
        "is_valid": is_valid,
        "total_found": len(found_indices_sorted),
        "missing_indices": missing_indices,
        "extra_indices": extra_indices,
        "breaks": breaks,
    }

def check_parquet_file(path):
    import pandas as pd

    df = pd.read_parquet(path, engine="pyarrow")
    print(df.info())
    print("index : ")
    print(df["index"])
    print("timestamp : ")
    print(df["timestamp"])
    print("episode_index : ")
    print(df["episode_index"])
    print("task_index : ")
    print(df["task_index"])
    print("frame_index : ")
    print(df["frame_index"])
    print("task_index : ")
    print(df["task_index"])




def reindex_parquet_episodes(data_base):
    """
    Reindex episode_index and global frame_index across all parquet files.
    
    - episode_index: sequential per file (0, 1, 2, ...)
    - frame_index: global across all episodes
    
    Args:
        data_base (str): Path to the 'data' folder containing chunk-* directories
    """
    global_frame_counter = 0

    for chunk in sorted(os.listdir(data_base)):
        chunk_path = os.path.join(data_base, chunk)
        if not os.path.isdir(chunk_path):
            continue

        # IMPORTANT: sorted order
        parquet_files = sorted(
            f for f in os.listdir(chunk_path) if f.endswith(".parquet")
        )

        for new_episode_idx, parquet_file in enumerate(parquet_files):
            parquet_path = os.path.join(chunk_path, parquet_file)

            # Load parquet
            df = pd.read_parquet(parquet_path)

            n_frames = len(df)

            # --- Fix episode_index ---
            df["episode_index"] = new_episode_idx

            # --- Fix global frame_index ---
            df["index"] = range(
                global_frame_counter,
                global_frame_counter + n_frames
            )

            global_frame_counter += n_frames

            # Write back (overwrite safely)
            df.to_parquet(parquet_path, index=False)

            print(
                f"Updated {parquet_file} | "
                f"episode_index={new_episode_idx} | "
                f"frames={n_frames} | "
                f"global_frame_end={global_frame_counter - 1}"
            )

    print(f"\n✅ Done. Total global frames: {global_frame_counter}")




def check_parquet_files_have_same_columns(directory_path: str):
    if not os.path.isdir(directory_path):
        raise ValueError(f"Not a directory: {directory_path}")

    parquet_files = sorted(
        f for f in os.listdir(directory_path)
        if f.endswith(".parquet")
    )

    if not parquet_files:
        print("⚠️ No parquet files found.")
        return [], []

    # Reference file (index 0)
    reference_path = os.path.join(directory_path, parquet_files[0])

    try:
        reference_df = pd.read_parquet(reference_path)
    except Exception as e:
        raise RuntimeError(f"Failed to read reference file: {e}")

    reference_columns = set(reference_df.columns)

    indices_missing_columns = []
    indices_extra_columns = []

    for idx, file in enumerate(parquet_files[1:], start=1):
        file_path = os.path.join(directory_path, file)
        print(f"\n📄 Checking [{idx}]: {file}")

        try:
            df = pd.read_parquet(file_path)
        except Exception as e:
            print(f"❌ Failed to read {file}: {e}")
            raise RuntimeError("stop")

        columns = set(df.columns)

        missing = reference_columns - columns
        extra = columns - reference_columns

        if missing:
            indices_missing_columns.append(idx)

        if extra:
            indices_extra_columns.append(idx)

        if not missing and not extra:
            print("✅ Columns match reference.")

    return indices_missing_columns, indices_extra_columns


# ===========================================================================
# Core Functions
# ==============================================================================





def clean_corrupted_videos(
    base_dir,
    data_base,
    video_base,
    meta_dir,
    video_subdirs,
):
    """
    Remove corrupted mp4/parquet episodes and reindex everything.
    """

    episode_indices = make_list_corrupted_episodes(video_base, video_subdirs)
    print("episode_indices : ", episode_indices)
    remove_corrupted_episodes_from_metadata(meta_dir, episode_indices)

    reindex_metadata_jsonl(meta_dir)

    video_subdirs = ["video", "video_2", "wrist_video", "wrist_video_2"]

    remove_problematic_episodes(episode_indices, data_base, video_base, video_subdirs)

    reindex_data_and_videos(data_base, video_base, video_subdirs)

    total_steps = count_total_steps(data_base)
    print("Total steps in dataset:", total_steps)

    total_videos = count_total_videos(video_base, video_subdirs)
    print("Total videos in dataset:", total_videos)

    print("Parquet files:", count_total_parquet_files(data_base))


    print(check_metadata_indices_2(meta_dir))
    print(check_parquet_indices(data_base))
    print(check_video_indices(video_base, video_subdirs))

    reindex_parquet_episodes(data_base)

    data_path = f"{meta_dir}/episodes.jsonl"
    print(validate_episode_indices(data_path))


def clean_cut_videos(
    base_dir, 
    data_base,
    video_base,
    meta_dir,
    video_subdirs,
    bad_episode_indices,
):
    """
    Remove episodes with shortened videos and reindex everything.
    """

    episode_indices = sorted(set(bad_episode_indices))
    episode_indices = [f"episode_{ep:06d}" for ep in episode_indices]

    print("episode_indices : ",episode_indices)

    remove_corrupted_episodes_from_metadata(meta_dir, episode_indices)

    reindex_metadata_jsonl(meta_dir)

    video_subdirs = ["video", "video_2", "wrist_video", "wrist_video_2"]

    remove_problematic_episodes(episode_indices, data_base, video_base, video_subdirs)

    reindex_data_and_videos(data_base, video_base, video_subdirs)

    total_steps = count_total_steps(data_base)
    print("Total steps in dataset:", total_steps)

    total_videos = count_total_videos(video_base, video_subdirs)
    print("Total videos in dataset:", total_videos)

    print("Parquet files:", count_total_parquet_files(data_base))

    print(check_metadata_indices_2(meta_dir))
    print(check_parquet_indices(data_base))
    print(check_video_indices(video_base, video_subdirs))

    reindex_parquet_episodes(data_base)

    data_path = f"{meta_dir}/episodes.jsonl"
    print(validate_episode_indices(data_path))
    total_steps = count_total_steps(data_base)
    print("total steps : ", total_steps)


def clean_missing_columns(
    base_dir, 
    data_base,
    video_base,
    meta_dir,
    video_subdirs,
):
    """
    Remove episodes with shortened videos and reindex everything.
    """

    directory = f"{data_base}/chunk-000"

    indices_missing, indices_extra = check_parquet_files_have_same_columns(directory)

    episode_indices = sorted(set(indices_missing))
    episode_indices = [f"episode_{ep:06d}" for ep in episode_indices]

    print("episode_indices : ",episode_indices)

    remove_corrupted_episodes_from_metadata(meta_dir, episode_indices)

    reindex_metadata_jsonl(meta_dir)

    video_subdirs = ["video", "video_2", "wrist_video", "wrist_video_2"]

    remove_problematic_episodes(episode_indices, data_base, video_base, video_subdirs)

    reindex_data_and_videos(data_base, video_base, video_subdirs)

    total_steps = count_total_steps(data_base)
    print("Total steps in dataset:", total_steps)

    total_videos = count_total_videos(video_base, video_subdirs)
    print("Total videos in dataset:", total_videos)

    print("Parquet files:", count_total_parquet_files(data_base))

    print(check_metadata_indices_2(meta_dir))
    print(check_parquet_indices(data_base))
    print(check_video_indices(video_base, video_subdirs))

    reindex_parquet_episodes(data_base)

    data_path = f"{meta_dir}/episodes.jsonl"
    print(validate_episode_indices(data_path))
    total_steps = count_total_steps(data_base)
    print("total steps : ", total_steps)




import argparse


def main():
    parser = argparse.ArgumentParser(
        description="Clean LeRobot dataset (corrupted videos, cut videos, or missing columns)"
    )

    # --- common paths ---
    parser.add_argument("--base-dir", required=True)
    parser.add_argument("--data-dir", required=True)
    parser.add_argument("--video-dir", required=True)
    parser.add_argument("--meta-dir", required=True)

    parser.add_argument(
        "--video-subdirs",
        nargs="+",
        default=["video", "video_2", "wrist_video", "wrist_video_2"],
    )

    # --- task selection ---
    parser.add_argument(
        "--task",
        required=True,
        choices=[
            "clean_corrupted_videos",
            "clean_cut_videos",
            "clean_missing_columns",
        ],
        help="Choose which cleaning operation to run",
    )

    parser.add_argument(
        "--bad-episodes",
        nargs="*",
        type=int,
        help="Episode indices with cut videos (only for clean_cut_videos)",
    )

    args = parser.parse_args()

    # -------------------- DISPATCH --------------------
    if args.task == "clean_corrupted_videos":
        clean_corrupted_videos(
            base_dir=args.base_dir,
            data_base=args.data_dir,
            video_base=args.video_dir,
            meta_dir=args.meta_dir,
            video_subdirs=args.video_subdirs,
        )

    elif args.task == "clean_cut_videos":
        bad_episodes = (
            args.bad_episodes
            if args.bad_episodes is not None
            else DEFAULT_BAD_EPISODES
        )

        clean_cut_videos(
            base_dir=args.base_dir,
            data_base=args.data_dir,
            video_base=args.video_dir,
            meta_dir=args.meta_dir,
            video_subdirs=args.video_subdirs,
            bad_episode_indices=bad_episodes,
        )

    elif args.task == "clean_missing_columns":
        clean_missing_columns(
            base_dir=args.base_dir,
            data_base=args.data_dir,
            video_base=args.video_dir,
            meta_dir=args.meta_dir,
            video_subdirs=args.video_subdirs,
        )



if __name__ == "__main__":
    main()


"""
python your_script.py \
    --base-dir /path/to/base \
    --data-dir /path/to/data \
    --video-dir /path/to/video \
    --meta-dir /path/to/meta \
    --task TASK_NAME \
    [--video-subdirs video video_2 wrist_video wrist_video_2] \
    [--expected-count 2112] \
    [--bad-episodes 1 2 3]
"""
