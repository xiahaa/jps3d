"""
Compare First Move Matrix implementations:
- Python implementation (generate_first_move_matrix.py)
- C++ implementation (FirstMoveCompression)

This script compares the results from both implementations to ensure they match.
"""

import numpy as np
import cv2
import subprocess
import os
import sys
import tempfile
from typing import Tuple, Optional
from generate_first_move_matrix import (
    compute_first_move_matrix_bfs,
    compute_first_move_matrix_dijkstra,
    OBSTACLE_VALUE,
    UNREACHABLE_VALUE,
    DIRECTIONS_4,
    DIRECTIONS_8
)
import cpp_first_move_matrix



def convert_map_to_octile_format(map_img: np.ndarray, output_path: str):
    """
    Convert map image to octile format (.map file) for C++ code.

    Args:
        map_img: 2D numpy array where 0=free, >200 or 1=obstacle
        output_path: Path to save .map file
    """
    height, width = map_img.shape

    # Determine obstacle threshold
    obstacle_mask = (map_img > 200) if map_img.max() > 1 else (map_img == 1)

    with open(output_path, 'w') as f:
        f.write("type octile\n")
        f.write(f"height {height}\n")
        f.write(f"width {width}\n")
        f.write("map\n")

        for y in range(height):
            for x in range(width):
                if obstacle_mask[y, x]:
                    f.write("@")  # Obstacle
                else:
                    f.write(".")  # Free
            f.write("\n")


def extract_cpp_first_move_matrix(
    map_path: str,
    goal: Tuple[int, int]) -> Optional[np.ndarray]:
    """
    Extract first move matrix from C++ implementation for a given goal using pybind11 wrapper.

    Args:
        map_path: Path to .map file
        goal: (x, y) goal position

    Returns:
        2D numpy array of first move directions, or None if extraction fails
    """
    # Try to import pybind11 module
    try:
        import cpp_first_move_matrix
    except ImportError:
        print("Warning: cpp_first_move_matrix module not found.")
        print("  Please build the pybind11 wrapper:")
        print("  cd 3rdparty/Polyanya-main/gppc/gppc-2014/entries/FirstMoveCompression")
        print("  pip install pybind11")
        print("  python setup.py build_ext --inplace")
        return None

    # matrix = cpp_first_move_matrix.extract_first_move_matrix(
    # './data/map.map', 70, 100)

    # Extract first move matrix using pybind11 wrapper
    print(f"Extracting first move matrix from C++ implementation...")
    try:
        matrix = cpp_first_move_matrix.extract_first_move_matrix(
            map_path,
            goal[0],
            goal[1]
        )
        # Convert to numpy array if needed (should already be numpy array from pybind11)
        if not isinstance(matrix, np.ndarray):
            matrix = np.array(matrix, dtype=np.int32)
        return matrix

    except Exception as e:
        print(f"Error extracting C++ first move matrix: {e}")
        import traceback
        traceback.print_exc()
        return None


def compare_first_move_matrices(
    python_matrix: np.ndarray,
    cpp_matrix: Optional[np.ndarray],
    map_img: np.ndarray,
    goal: Tuple[int, int],
    use_8_connected: bool = False
) -> dict:
    """
    Compare first move matrices from Python and C++ implementations.

    Args:
        python_matrix: First move matrix from Python implementation
        cpp_matrix: First move matrix from C++ implementation (can be None)
        map_img: Original map image
        goal: Goal position used
        use_8_connected: Whether 8-connected was used

    Returns:
        Dictionary with comparison results
    """
    height, width = map_img.shape
    obstacle_mask = (map_img > 200) if map_img.max() > 1 else (map_img == 1)

    results = {
        'total_cells': int(height * width),
        'obstacle_cells': int(np.sum(obstacle_mask)),
        'free_cells': int(height * width - np.sum(obstacle_mask)),
        'python_stats': {},
        'cpp_available': cpp_matrix is not None,
        'comparison': {}
    }

    # Python matrix statistics
    unique, counts = np.unique(python_matrix, return_counts=True)
    for val, count in zip(unique, counts):
        val_int = int(val)  # Convert numpy type to Python int
        count_int = int(count)  # Convert numpy type to Python int
        if val_int == OBSTACLE_VALUE:
            results['python_stats']['obstacles'] = count_int
        elif val_int == UNREACHABLE_VALUE:
            results['python_stats']['unreachable'] = count_int
        else:
            results['python_stats'][f'direction_{val_int}'] = count_int

    # Compare with C++ if available
    if cpp_matrix is not None:
        if python_matrix.shape != cpp_matrix.shape:
            results['comparison']['error'] = "Shape mismatch"
            return results

        # Compare cell by cell
        matches = (python_matrix == cpp_matrix)
        # Ignore obstacles and unreachable in comparison
        valid_mask = ~obstacle_mask
        valid_matches = matches[valid_mask]

        valid_matches_sum = int(np.sum(valid_matches))
        valid_mask_sum = int(np.sum(valid_mask))
        results['comparison'] = {
            'total_matches': int(np.sum(matches)),
            'valid_matches': valid_matches_sum,
            'valid_cells': valid_mask_sum,
            'match_rate': float(valid_matches_sum / valid_mask_sum) if valid_mask_sum > 0 else 0.0,
            'mismatches': []
        }

        # Find mismatches
        mismatch_mask = valid_mask & ~matches
        mismatch_indices = np.where(mismatch_mask)
        num_mismatches = len(mismatch_indices[0])

        # Sample some mismatches for reporting
        if num_mismatches > 0:
            sample_size = min(10, num_mismatches)
            sample_indices = np.random.choice(num_mismatches, sample_size, replace=False)
            for idx in sample_indices:
                y, x = mismatch_indices[0][idx], mismatch_indices[1][idx]
                results['comparison']['mismatches'].append({
                    'position': (int(x), int(y)),
                    'python': int(python_matrix[y, x]),
                    'cpp': int(cpp_matrix[y, x])
                })
    else:
        results['comparison'] = {
            'note': 'C++ matrix not available for comparison'
        }

    return results


def main():
    """Main comparison function."""
    import argparse

    parser = argparse.ArgumentParser(description='Compare first move matrix implementations')
    parser.add_argument('--map', type=str, default='data/image.png',
                        help='Path to map image file')
    parser.add_argument('--goal', type=int, nargs=2, default=None,
                        help='Goal position (x y). If not provided, uses center of map.')
    parser.add_argument('--use-8-connected', action='store_true',
                        help='Use 8-connected grid instead of 4-connected')
    parser.add_argument('--use-dijkstra', action='store_true',
                        help='Use Dijkstra instead of BFS')
    parser.add_argument('--cpp-binary', type=str, default=None,
                        help='Path to C++ binary')
    parser.add_argument('--output', type=str, default='first_move_comparison_results.json',
                        help='Output file for comparison results')

    args = parser.parse_args()

    # Load map
    print(f"Loading map from {args.map}...")
    map_img = cv2.imread(args.map, cv2.IMREAD_GRAYSCALE)
    if map_img is None:
        print(f"Error: Could not load map from {args.map}")
        return 1

    # Resize if needed (for consistency with other scripts)
    if map_img.shape[0] != 256 or map_img.shape[1] != 256:
        map_img = cv2.resize(map_img, (256, 256))
        print(f"Resized map to 256x256")

    height, width = map_img.shape
    print(f"Map size: {width}x{height}")

    # Determine goal
    if args.goal:
        goal = tuple(args.goal)
    else:
        goal = (3 * width // 4, 3 * height // 4)

    print(f"Goal position: {goal}")

    # Generate Python first move matrix
    print("\nGenerating Python first move matrix...")
    if args.use_dijkstra:
        python_matrix = compute_first_move_matrix_dijkstra(
            map_img, goal, args.use_8_connected
        )
    else:
        python_matrix = compute_first_move_matrix_bfs(
            map_img, goal, args.use_8_connected
        )

    print("Python matrix generated.")

    # Try to extract C++ matrix
    print("\nAttempting to extract C++ first move matrix...")
    with tempfile.TemporaryDirectory() as tmpdir:
        map_file = os.path.join('./data', "map.map")
        print(f"Map file: {map_file}")

        convert_map_to_octile_format(map_img, map_file)

        matrix = cpp_first_move_matrix.extract_first_move_matrix(
            'tmp.', map_file, goal[0], goal[1]
        )
        if not isinstance(matrix, np.ndarray):
            matrix = np.array(matrix, dtype=np.int32)
        cpp_matrix = matrix

    # Compare
    print("\nComparing matrices...")
    comparison_results = compare_first_move_matrices(
        python_matrix, cpp_matrix, map_img, goal, args.use_8_connected
    )

    # Print results
    print("\n" + "="*60)
    print("COMPARISON RESULTS")
    print("="*60)
    print(f"Total cells: {comparison_results['total_cells']}")
    print(f"Obstacle cells: {comparison_results['obstacle_cells']}")
    print(f"Free cells: {comparison_results['free_cells']}")
    print("\nPython matrix statistics:")
    for key, value in comparison_results['python_stats'].items():
        print(f"  {key}: {value}")

    if comparison_results['cpp_available']:
        print("\nComparison with C++:")
        comp = comparison_results['comparison']
        print(f"  Valid matches: {comp['valid_matches']}/{comp['valid_cells']}")
        print(f"  Match rate: {comp['match_rate']*100:.2f}%")
        if comp['mismatches']:
            print(f"  Sample mismatches (showing {len(comp['mismatches'])} of {len(comp.get('all_mismatches', []))}):")
            for mm in comp['mismatches'][:5]:
                print(f"    Position {mm['position']}: Python={mm['python']}, C++={mm['cpp']}")
    else:
        print("\nC++ comparison: Not available (requires Python wrapper)")
        print("Note: To enable C++ comparison, create a Python wrapper")
        print("      using ctypes or pybind11 to interface with the C++ code.")

    # Save results
    import json

    # Convert numpy types to native Python types for JSON serialization
    def convert_to_python_types(obj):
        """Recursively convert numpy types to Python native types."""
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, dict):
            return {key: convert_to_python_types(value) for key, value in obj.items()}
        elif isinstance(obj, (list, tuple)):
            return [convert_to_python_types(item) for item in obj]
        else:
            return obj

    comparison_results_serializable = convert_to_python_types(comparison_results)

    with open(args.output, 'w') as f:
        json.dump(comparison_results_serializable, f, indent=2)
    print(f"\nResults saved to {args.output}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
