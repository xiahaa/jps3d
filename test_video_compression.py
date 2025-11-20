"""
Test video compression for first move matrices.

This script:
1. Generates all possible first move matrices (one for each valid goal position)
2. Compresses them using MPEG video encoding
3. Compares file size with CPD compression
4. Verifies correctness by decompressing and comparing
"""

import numpy as np
import cv2
import os
import sys
import subprocess
import tempfile
import shutil
from pathlib import Path
from typing import List, Tuple, Dict
import json
from tqdm import tqdm


try:
    import cpp_first_move_matrix
    CPP_AVAILABLE = True
except ImportError:
    print("Warning: cpp_first_move_matrix not available. Will use Python implementation.")
    CPP_AVAILABLE = False
    sys.path.append('python_impl')
    from generate_first_move_matrix import compute_first_move_matrix_dijkstra


def load_map_from_octile(map_path: str) -> Tuple[np.ndarray, int, int]:
    """
    Load map from octile format (.map file).

    Returns:
        (map_data, width, height) where map_data[y, x] = True for free, False for obstacle
    """
    with open(map_path, 'r') as f:
        lines = f.readlines()

    # Parse header
    width, height = None, None
    map_start_idx = 0
    for i, line in enumerate(lines):
        if line.startswith('width'):
            width = int(line.split()[1])
        elif line.startswith('height'):
            height = int(line.split()[1])
        elif line.strip() == 'map':
            map_start_idx = i + 1
            break

    if width is None or height is None:
        raise ValueError("Could not parse map dimensions")

    # Parse map data
    map_data = np.zeros((height, width), dtype=bool)
    y = 0
    for i in range(map_start_idx, len(lines)):
        line = lines[i].strip()
        if not line:
            continue
        for x, char in enumerate(line):
            if x >= width:
                break
            if y >= height:
                break
            # '.' or 'G' or 'S' = free, '@' or 'T' = obstacle
            map_data[y, x] = (char == '.' or char == 'G' or char == 'S')
        y += 1
        if y >= height:
            break

    return map_data, width, height


def get_all_free_cells(map_data: np.ndarray) -> List[Tuple[int, int]]:
    """Get all free cell positions (x, y) in the map."""
    free_cells = []
    height, width = map_data.shape
    for y in range(height):
        for x in range(width):
            if map_data[y, x]:
                free_cells.append((x, y))
    return free_cells


def matrix_to_frame(matrix: np.ndarray) -> np.ndarray:
    """
    Convert first move matrix to a single-channel frame for compression.

    Encoding (uint8):
    - 0: Obstacle (-1)
    - 1: Unreachable (-2)
    - 2-17: Valid directions (0-15) mapped to value + 2
    """
    frame = np.zeros(matrix.shape, dtype=np.uint8)
    frame[matrix == -1] = 0  # Obstacles
    frame[matrix == -2] = 1  # Unreachable

    valid_mask = matrix >= 0
    frame[valid_mask] = matrix[valid_mask].astype(np.uint8) + 2
    return frame


def frame_to_matrix(frame: np.ndarray) -> np.ndarray:
    """
    Convert single-channel frame back to first move matrix.

    This is the inverse of matrix_to_frame.
    """
    if frame.ndim == 3:
        # Convert color frame to grayscale
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    matrix = np.full(frame.shape, -2, dtype=np.int32)
    matrix[frame == 0] = -1  # Obstacles
    matrix[frame == 1] = -2  # Unreachable

    valid_mask = frame >= 2
    matrix[valid_mask] = frame[valid_mask].astype(np.int32) - 2
    return matrix


def generate_all_first_move_matrices(
    map_path: str,
    preprocessed_file: str,
    free_cells: List[Tuple[int, int]],
    frames_dir: str,
    use_cpp: bool = True,
    max_goals: int = None,
    batch_size: int = 100
) -> List[Tuple[int, int]]:
    """
    Generate first move matrices for all (or a subset of) free cells.
    Writes each matrix to disk immediately as an image to avoid memory issues.

    Args:
        map_path: Path to .map file
        preprocessed_file: Path to preprocessed CPD file
        free_cells: List of (x, y) free cell positions
        frames_dir: Directory to save frame images
        use_cpp: Whether to use C++ implementation
        max_goals: Maximum number of goals to process (None = all)
        batch_size: Number of matrices to extract at once when using C++ batch function

    Returns:
        List of (x, y) goals that were successfully processed (in order)
    """
    os.makedirs(frames_dir, exist_ok=True)

    if max_goals is not None:
        free_cells = free_cells[:max_goals]

    print(f"Generating first move matrices for {len(free_cells)} goals...")
    print(f"Writing frames to: {frames_dir}")

    processed_goals = []
    sorted_goals = sorted(free_cells)  # Sort for consistent ordering

    # Check if batch extraction is available
    use_batch = False
    if use_cpp and CPP_AVAILABLE:
        try:
            # Check if the batch function exists
            if hasattr(cpp_first_move_matrix, 'extract_first_move_matrices'):
                use_batch = True
                print(f"Using batch extraction with batch size {batch_size}")
        except:
            pass

    if use_batch:
        # Process in batches for efficiency
        total_batches = (len(sorted_goals) + batch_size - 1) // batch_size
        frame_idx = 0

        for batch_num in tqdm(range(total_batches)):
            start_idx = batch_num * batch_size
            end_idx = min(start_idx + batch_size, len(sorted_goals))
            batch_goals = sorted_goals[start_idx:end_idx]

            try:
                # Extract batch of matrices
                matrices = cpp_first_move_matrix.extract_first_move_matrices(
                    preprocessed_file,
                    map_path,
                    batch_goals
                )

                # Write each matrix to disk
                for i, (goal_x, goal_y) in enumerate(batch_goals):
                    if i < len(matrices):
                        matrix = matrices[i]
                        if not isinstance(matrix, np.ndarray):
                            matrix = np.array(matrix, dtype=np.int32)

                        # Convert to image and write immediately to disk
                        frame = matrix_to_frame(matrix)
                        frame_path = os.path.join(frames_dir, f"frame_{frame_idx:06d}.png")
                        cv2.imwrite(frame_path, frame)

                        processed_goals.append((goal_x, goal_y))
                        frame_idx += 1

                        # Free memory
                        del matrix
                        del frame

                # Clear batch from memory
                del matrices

            except Exception as e:
                print(f"Error in batch {batch_num}: {e}")
                # Fall back to individual extraction for this batch
                for goal_x, goal_y in batch_goals:
                    try:
                        matrix = cpp_first_move_matrix.extract_first_move_matrix(
                            preprocessed_file,
                            map_path,
                            goal_x,
                            goal_y
                        )
                        if not isinstance(matrix, np.ndarray):
                            matrix = np.array(matrix, dtype=np.int32)

                        frame = matrix_to_frame(matrix)
                        frame_path = os.path.join(frames_dir, f"frame_{frame_idx:06d}.png")
                        cv2.imwrite(frame_path, frame)

                        processed_goals.append((goal_x, goal_y))
                        frame_idx += 1

                        del matrix
                        del frame
                    except Exception as e2:
                        print(f"Error generating matrix for goal ({goal_x}, {goal_y}): {e2}")
                        continue
    else:
        # Process one at a time
        for frame_idx, (goal_x, goal_y) in enumerate(tqdm(sorted_goals, desc="Generating matrices")):
            try:
                if use_cpp and CPP_AVAILABLE:
                    matrix = cpp_first_move_matrix.extract_first_move_matrix(
                        preprocessed_file,
                        map_path,
                        goal_x,
                        goal_y
                    )
                    if not isinstance(matrix, np.ndarray):
                        matrix = np.array(matrix, dtype=np.int32)
                else:
                    # Use Python implementation
                    sys.path.insert(0, 'python_impl')
                    from generate_first_move_matrix import compute_first_move_matrix_dijkstra
                    # Try to load map image, or create from .map file
                    map_img_path = map_path.replace('.map', '.png')
                    if os.path.exists(map_img_path):
                        map_img = cv2.imread(map_img_path, cv2.IMREAD_GRAYSCALE)
                    else:
                        # Create map image from .map file
                        map_data, width, height = load_map_from_octile(map_path)
                        map_img = (~map_data).astype(np.uint8) * 255
                    matrix = compute_first_move_matrix_dijkstra(map_img, (goal_x, goal_y), True)

                # Convert to image and write immediately to disk
                frame = matrix_to_frame(matrix)
                frame_path = os.path.join(frames_dir, f"frame_{frame_idx:06d}.png")
                cv2.imwrite(frame_path, frame)

                processed_goals.append((goal_x, goal_y))

                # Free memory
                del matrix
                del frame

            except Exception as e:
                print(f"Error generating matrix for goal ({goal_x}, {goal_y}): {e}")
                continue

    return processed_goals


def create_video_from_frames(
    frames_dir: str,
    goal_order: List[Tuple[int, int]],
    output_video_path: str,
    fps: int = 10
) -> bool:
    """
    Create MPEG video from frame images on disk.

    Args:
        frames_dir: Directory containing frame images (frame_000000.png, frame_000001.png, ...)
        goal_order: List of goals in frame order
        output_video_path: Path to output video file
        fps: Frames per second (default 1)

    Returns:
        True if successful, False otherwise
    """
    # Check if frames directory exists and has frames
    frame_files = sorted(Path(frames_dir).glob('frame_*.png'))
    if not frame_files:
        print(f"No frames found in {frames_dir}")
        return False

    if len(frame_files) != len(goal_order):
        print(f"Warning: Frame count ({len(frame_files)}) doesn't match goal count ({len(goal_order)})")

    # Check if ffmpeg is available
    try:
        subprocess.run(['ffmpeg', '-version'], capture_output=True, check=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("Error: ffmpeg not found. Please install ffmpeg.")
        return False

    # Save goal-to-frame mapping
    mapping_file = output_video_path.replace('.mp4', '_mapping.json')
    with open(mapping_file, 'w') as f:
        json.dump([{'goal': list(goal), 'frame': i} for i, goal in enumerate(goal_order)], f, indent=2)
    print(f"Saved goal-to-frame mapping to {mapping_file}")

    # Encode video using ffmpeg
    print(f"Encoding video with {len(frame_files)} frames from {frames_dir}...")
    try:
        # Use H.264 codec (MPEG-4 Part 10) for good compression
        cmd = [
            'ffmpeg',
            '-y',  # Overwrite output file
            '-framerate', str(fps),
            '-i', os.path.join(frames_dir, 'frame_%06d.png'),
            '-c:v', 'libx264',  # H.264 codec
            '-preset', 'slow',  # Better compression
            '-crf', '0',  # Higher quality (lower = better quality, larger file)
            '-pix_fmt', 'gray',  # Single channel video
            output_video_path
        ]

        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            check=True
        )
        print(f"Video encoded successfully: {output_video_path}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"Error encoding video: {e}")
        print(f"stderr: {e.stderr}")
        return False


def extract_frames_from_video(video_path: str, output_dir: str) -> List[str]:
    """
    Extract frames from video using ffmpeg.

    Returns:
        List of frame file paths
    """
    os.makedirs(output_dir, exist_ok=True)

    cmd = [
        'ffmpeg',
        '-i', video_path,
        os.path.join(output_dir, 'frame_%06d.png')
    ]

    try:
        subprocess.run(cmd, capture_output=True, check=True)
        frame_files = sorted(Path(output_dir).glob('frame_*.png'))
        return [str(f) for f in frame_files]
    except subprocess.CalledProcessError as e:
        print(f"Error extracting frames: {e}")
        return []


def verify_correctness(
    frames_dir: str,
    video_path: str,
    goal_order: List[Tuple[int, int]],
    map_path: str,
    preprocessed_file: str,
    use_cpp: bool = True
) -> Dict:
    """
    Verify correctness by comparing original frames with decompressed video frames.

    Args:
        frames_dir: Directory containing original frame images
        video_path: Path to video file
        goal_order: List of goals in frame order
        map_path: Path to map file (for regenerating if needed)
        preprocessed_file: Path to preprocessed CPD file (for regenerating if needed)
        use_cpp: Whether to use C++ implementation

    Returns:
        Dictionary with verification results
    """
    print("Verifying correctness...")

    with tempfile.TemporaryDirectory() as tmpdir:
        # Extract frames from video
        frame_files = extract_frames_from_video(video_path, tmpdir)

        if len(frame_files) != len(goal_order):
            return {
                'success': False,
                'error': f'Frame count mismatch: {len(frame_files)} vs {len(goal_order)}'
            }

        # Get original frames
        original_frame_files = sorted(Path(frames_dir).glob('frame_*.png'))
        if len(original_frame_files) != len(frame_files):
            return {
                'success': False,
                'error': f'Original frame count mismatch: {len(original_frame_files)} vs {len(frame_files)}'
            }

        total_cells = 0
        matching_cells = 0
        errors = []

        for i, goal in enumerate(tqdm(goal_order, desc="Verifying")):
            if i >= len(frame_files) or i >= len(original_frame_files):
                break

            # Load original and decoded frames
            original_image = cv2.imread(str(original_frame_files[i]), cv2.IMREAD_UNCHANGED)
            decoded_image = cv2.imread(frame_files[i], cv2.IMREAD_UNCHANGED)

            if original_image is None or decoded_image is None:
                errors.append({
                    'goal': list(goal),
                    'error': 'Could not load frame'
                })
                continue

            # Convert to matrices for comparison
            original_matrix = frame_to_matrix(original_image)
            decoded_matrix = frame_to_matrix(decoded_image)

            # Compare
            if original_matrix.shape != decoded_matrix.shape:
                errors.append({
                    'goal': list(goal),
                    'error': 'Shape mismatch'
                })
                continue

            # Compare cell by cell
            match_mask = (original_matrix == decoded_matrix)
            total_cells += original_matrix.size
            matching_cells += np.sum(match_mask)

            # Check for significant errors
            mismatch_count = np.sum(~match_mask)
            if mismatch_count > original_matrix.size * 0.01:  # More than 1% mismatch
                errors.append({
                    'goal': list(goal),
                    'mismatch_count': int(mismatch_count),
                    'mismatch_rate': float(mismatch_count / original_matrix.size)
                })

        accuracy = matching_cells / total_cells if total_cells > 0 else 0.0

        return {
            'success': True,
            'total_cells': int(total_cells),
            'matching_cells': int(matching_cells),
            'accuracy': float(accuracy),
            'errors': errors[:10]  # Sample of errors
        }


def get_file_size(file_path: str) -> int:
    """Get file size in bytes."""
    if os.path.exists(file_path):
        return os.path.getsize(file_path)
    return 0


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Test video compression for first move matrices')
    parser.add_argument('--map', type=str, default='data/map.map',
                        help='Path to map file (.map format)')
    parser.add_argument('--preprocessed', type=str, default='data/first_move_matrix_cpp.map',
                        help='Path to preprocessed CPD file')
    parser.add_argument('--output-video', type=str, default='data/first_move_matrices.mp4',
                        help='Output video file path')
    parser.add_argument('--max-goals', type=int, default=None,
                        help='Maximum number of goals to process (for testing)')
    parser.add_argument('--use-cpp', action='store_true', default=True,
                        help='Use C++ implementation (default: True)')
    parser.add_argument('--frames-dir', type=str, default='data/first_move_frames',
                        help='Directory to store frame images (default: data/first_move_frames)')
    parser.add_argument('--skip-generation', action='store_true',
                        help='Skip matrix generation (use existing frames)')
    parser.add_argument('--skip-verification', action='store_true',
                        help='Skip correctness verification')
    parser.add_argument('--keep-frames', action='store_true',
                        help='Keep frame images after video creation (default: delete)')
    parser.add_argument('--batch-size', type=int, default=100,
                        help='Batch size for batch extraction (default: 100)')
    parser.add_argument('--fps', type=int, default=30,
                        help='Frames per second for the video (default: 30)')

    args = parser.parse_args()

    # Load map
    print(f"Loading map from {args.map}...")
    try:
        map_data, width, height = load_map_from_octile(args.map)
        print(f"Map size: {width}x{height}")
    except Exception as e:
        print(f"Error loading map: {e}")
        return 1

    # Get all free cells
    free_cells = get_all_free_cells(map_data)
    print(f"Found {len(free_cells)} free cells")

    if args.max_goals:
        print(f"Limiting to {args.max_goals} goals for testing")
        free_cells = free_cells[:args.max_goals]

    # Generate all first move matrices and write to disk
    goal_order = []
    if args.skip_generation and os.path.exists(args.frames_dir):
        print(f"Using existing frames from {args.frames_dir}")
        # Load goal order from mapping file if available
        mapping_file = args.output_video.replace('.mp4', '_mapping.json')
        if os.path.exists(mapping_file):
            with open(mapping_file, 'r') as f:
                mapping_data = json.load(f)
                goal_order = [tuple(item['goal']) for item in mapping_data]
        else:
            # Try to infer from frame files
            frame_files = sorted(Path(args.frames_dir).glob('frame_*.png'))
            print(f"Found {len(frame_files)} existing frames (goal order unknown)")
    else:
        goal_order = generate_all_first_move_matrices(
            args.map,
            args.preprocessed,
            free_cells,
            args.frames_dir,
            use_cpp=args.use_cpp,
            max_goals=args.max_goals,
            batch_size=args.batch_size
        )
        print(f"Generated {len(goal_order)} first move matrices")

    # Create video from frames
    if not args.skip_generation or not os.path.exists(args.output_video):
        print(f"\nCreating video: {args.output_video}")
        success = create_video_from_frames(args.frames_dir, goal_order, args.output_video, fps=args.fps)
        if not success:
            print("Failed to create video")
            return 1
    else:
        print(f"Using existing video: {args.output_video}")

    # Get file sizes
    video_size = get_file_size(args.output_video)
    cpd_size = get_file_size(args.preprocessed)

    # Calculate uncompressed size (raw matrices)
    # Estimate from first frame if available, or use map dimensions
    frame_files = sorted(Path(args.frames_dir).glob('frame_*.png'))
    if frame_files:
        sample_image = cv2.imread(str(frame_files[0]), cv2.IMREAD_UNCHANGED)
        if sample_image is not None:
            frame_height, frame_width = sample_image.shape[:2]
        else:
            frame_height, frame_width = height, width
        uncompressed_size = len(goal_order) * frame_height * frame_width * 4  # int32 = 4 bytes
    else:
        uncompressed_size = len(goal_order) * height * width * 4 if goal_order else 0

    print("\n" + "="*60)
    print("COMPRESSION RESULTS")
    print("="*60)
    print(f"Number of matrices: {len(goal_order)}")
    print(f"Uncompressed size: {uncompressed_size / (1024*1024):.2f} MB")
    print(f"CPD file size: {cpd_size / (1024*1024):.2f} MB")
    print(f"Video file size: {video_size / (1024*1024):.2f} MB")
    if uncompressed_size > 0:
        print(f"CPD compression ratio: {uncompressed_size / cpd_size:.2f}x")
        print(f"Video compression ratio: {uncompressed_size / video_size:.2f}x")
        print(f"Video vs CPD size ratio: {video_size / cpd_size:.2f}x")

    # Verify correctness
    verification_results = None
    if not args.skip_verification and goal_order:
        print("\n" + "="*60)
        print("VERIFICATION")
        print("="*60)
        verification_results = verify_correctness(
            args.frames_dir,
            args.output_video,
            goal_order,
            args.map,
            args.preprocessed,
            args.use_cpp
        )

        if verification_results.get('success'):
            print(f"Total cells compared: {verification_results['total_cells']}")
            print(f"Matching cells: {verification_results['matching_cells']}")
            print(f"Accuracy: {verification_results['accuracy']*100:.2f}%")
            if verification_results['errors']:
                print(f"Errors found: {len(verification_results['errors'])}")
                for err in verification_results['errors'][:5]:
                    print(f"  {err}")
        else:
            print(f"Verification failed: {verification_results.get('error', 'Unknown error')}")

    # Clean up frames if requested
    if not args.keep_frames and os.path.exists(args.frames_dir) and not args.skip_generation:
        print(f"\nCleaning up frame directory: {args.frames_dir}")
        try:
            shutil.rmtree(args.frames_dir)
            print("Frame directory deleted")
        except Exception as e:
            print(f"Warning: Could not delete frame directory: {e}")

    # Save results
    results = {
        'num_matrices': len(goal_order),
        'uncompressed_size_mb': uncompressed_size / (1024*1024),
        'cpd_size_mb': cpd_size / (1024*1024),
        'video_size_mb': video_size / (1024*1024),
        'cpd_compression_ratio': uncompressed_size / cpd_size if cpd_size > 0 else 0,
        'video_compression_ratio': uncompressed_size / video_size if video_size > 0 else 0,
        'video_vs_cpd_ratio': video_size / cpd_size if cpd_size > 0 else 0,
    }

    if verification_results:
        results['verification'] = verification_results

    results_file = args.output_video.replace('.mp4', '_results.json')
    with open(results_file, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\nResults saved to {results_file}")

    return 0


if __name__ == '__main__':
    sys.exit(main())
