"""
Batch Generate First Move Matrix Training Data

This script processes scenario files from gppc-2014 and generates first move
matrices as training data for neural networks. It can process multiple maps
and scenarios, saving the results in a format suitable for training.
"""

import numpy as np
import os
import argparse
from pathlib import Path
from typing import List, Tuple, Optional
from generate_first_move_matrix import (
    generate_training_sample,
    compute_first_move_matrix_bfs,
    OBSTACLE_VALUE,
    UNREACHABLE_VALUE
)


def load_map_file(map_path: str) -> Tuple[np.ndarray, int, int]:
    """
    Load a .map file in the standard format.
    
    Format:
        type octile
        height H
        width W
        map
        [map data - characters]
    
    Args:
        map_path: Path to .map file
    
    Returns:
        Tuple of (map_array, width, height)
        map_array: 2D numpy array where 0=free, 255=obstacle
    """
    with open(map_path, 'r') as f:
        lines = f.readlines()
    
    # Parse header
    height = None
    width = None
    map_start_idx = 0
    
    for i, line in enumerate(lines):
        if line.startswith('type'):
            # Should be 'type octile'
            continue
        elif line.startswith('height'):
            height = int(line.split()[1])
        elif line.startswith('width'):
            width = int(line.split()[1])
        elif line.strip() == 'map':
            map_start_idx = i + 1
            break
    
    if height is None or width is None:
        raise ValueError(f"Could not parse map header from {map_path}")
    
    # Parse map data
    map_chars = []
    for line in lines[map_start_idx:]:
        for char in line:
            if char in [' ', '\t', '\n', '\r']:
                continue
            map_chars.append(char)
            if len(map_chars) >= height * width:
                break
        if len(map_chars) >= height * width:
            break
    
    if len(map_chars) < height * width:
        raise ValueError(f"Map file has insufficient data: expected {height*width}, got {len(map_chars)}")
    
    # Convert to numpy array
    # Obstacles: '@', 'T', 'S', 'W', 'O'
    # Free: '.', 'G'
    map_array = np.zeros((height, width), dtype=np.uint8)
    for y in range(height):
        for x in range(width):
            char = map_chars[y * width + x]
            if char in ['@', 'T', 'S', 'W', 'O']:
                map_array[y, x] = 255  # Obstacle
            else:
                map_array[y, x] = 0  # Free
    
    return map_array, width, height


def load_scenario_file(scen_path: str) -> List[dict]:
    """
    Load a .scen scenario file.
    
    Format (version 1.0):
        version 1
        bucket map sizeX sizeY xs ys xg yg distance
    
    Args:
        scen_path: Path to .scen file
    
    Returns:
        List of scenario dictionaries with keys:
        - bucket: bucket number
        - map: map filename
        - sizeX, sizeY: map dimensions
        - start: (xs, ys) start position
        - goal: (xg, yg) goal position
        - distance: optimal distance
    """
    scenarios = []
    
    with open(scen_path, 'r') as f:
        lines = f.readlines()
    
    # Check version
    if lines[0].strip().startswith('version'):
        version = float(lines[0].strip().split()[1])
        start_idx = 1
    else:
        version = 0.0
        start_idx = 0
    
    for line in lines[start_idx:]:
        line = line.strip()
        if not line:
            continue
        
        parts = line.split()
        if version == 1.0:
            # bucket map sizeX sizeY xs ys xg yg distance
            if len(parts) >= 9:
                bucket = int(parts[0])
                map_name = parts[1]
                sizeX = int(parts[2])
                sizeY = int(parts[3])
                xs = int(parts[4])
                ys = int(parts[5])
                xg = int(parts[6])
                yg = int(parts[7])
                distance = float(parts[8])
                
                scenarios.append({
                    'bucket': bucket,
                    'map': map_name,
                    'sizeX': sizeX,
                    'sizeY': sizeY,
                    'start': (xs, ys),
                    'goal': (xg, yg),
                    'distance': distance
                })
        else:
            # version 0.0: bucket map xs ys xg yg distance
            if len(parts) >= 7:
                bucket = int(parts[0])
                map_name = parts[1]
                xs = int(parts[2])
                ys = int(parts[3])
                xg = int(parts[4])
                yg = int(parts[5])
                distance = float(parts[6])
                
                scenarios.append({
                    'bucket': bucket,
                    'map': map_name,
                    'sizeX': None,  # Not specified in v0.0
                    'sizeY': None,
                    'start': (xs, ys),
                    'goal': (xg, yg),
                    'distance': distance
                })
    
    return scenarios


def process_scenario(
    map_array: np.ndarray,
    scenario: dict,
    use_8_connected: bool = False,
    use_bfs: bool = True
) -> Optional[dict]:
    """
    Process a single scenario and generate training sample.
    
    Args:
        map_array: Map array (height x width)
        scenario: Scenario dictionary
        use_8_connected: Whether to use 8-connected grid
        use_bfs: Whether to use BFS (faster) or Dijkstra
    
    Returns:
        Training sample dictionary or None if invalid
    """
    start = scenario['start']
    goal = scenario['goal']
    
    # Validate positions
    height, width = map_array.shape
    if (start[0] < 0 or start[0] >= width or start[1] < 0 or start[1] >= height or
        goal[0] < 0 or goal[0] >= width or goal[1] < 0 or goal[1] >= height):
        return None
    
    # Check if start/goal are obstacles
    if map_array[start[1], start[0]] > 200 or map_array[goal[1], goal[0]] > 200:
        return None
    
    # Generate training sample
    try:
        sample = generate_training_sample(
            map_array, start, goal,
            use_8_connected=use_8_connected,
            use_bfs=use_bfs
        )
        return sample
    except Exception as e:
        print(f"Error processing scenario: {e}")
        return None


def batch_process(
    scenarios_dir: str,
    output_dir: str,
    max_scenarios: Optional[int] = None,
    use_8_connected: bool = False,
    use_bfs: bool = True,
    map_filter: Optional[str] = None
):
    """
    Batch process scenarios and generate training data.
    
    Args:
        scenarios_dir: Directory containing .map and .scen files
        output_dir: Directory to save training data
        max_scenarios: Maximum number of scenarios to process (None = all)
        use_8_connected: Whether to use 8-connected grid
        use_bfs: Whether to use BFS
        map_filter: Optional filter for map names (e.g., "maze-*")
    """
    scenarios_dir = Path(scenarios_dir)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Find all scenario files
    scen_files = list(scenarios_dir.glob("*.scen"))
    print(f"Found {len(scen_files)} scenario files")
    
    all_samples = []
    processed_count = 0
    skipped_count = 0
    
    for scen_file in scen_files:
        print(f"\nProcessing {scen_file.name}...")
        
        # Load scenarios
        try:
            scenarios = load_scenario_file(str(scen_file))
        except Exception as e:
            print(f"  Error loading scenario file: {e}")
            continue
        
        # Load corresponding map
        map_name = None
        for scen in scenarios:
            if scen['map']:
                map_name = scen['map']
                break
        
        if map_name is None:
            print(f"  No map name found in scenarios")
            continue
        
        # Apply map filter if specified
        if map_filter and not Path(map_name).match(map_filter):
            continue
        
        map_path = scenarios_dir / map_name
        if not map_path.exists():
            print(f"  Map file not found: {map_path}")
            continue
        
        # Load map
        try:
            map_array, map_width, map_height = load_map_file(str(map_path))
            print(f"  Loaded map: {map_name} ({map_width}x{map_height})")
        except Exception as e:
            print(f"  Error loading map: {e}")
            continue
        
        # Process each scenario
        for i, scenario in enumerate(scenarios):
            if max_scenarios and processed_count >= max_scenarios:
                break
            
            # Verify map name matches
            if scenario['map'] != map_name:
                continue
            
            sample = process_scenario(
                map_array, scenario,
                use_8_connected=use_8_connected,
                use_bfs=use_bfs
            )
            
            if sample is None:
                skipped_count += 1
                continue
            
            all_samples.append(sample)
            processed_count += 1
            
            if processed_count % 100 == 0:
                print(f"  Processed {processed_count} samples...")
        
        if max_scenarios and processed_count >= max_scenarios:
            break
    
    print(f"\n=== Summary ===")
    print(f"Total samples generated: {processed_count}")
    print(f"Total samples skipped: {skipped_count}")
    
    # Save training data
    if all_samples:
        output_file = output_dir / "first_move_training_data.npz"
        print(f"\nSaving training data to {output_file}...")
        
        # Convert to arrays for efficient storage
        maps = np.array([s['map'] for s in all_samples])
        starts = np.array([s['start'] for s in all_samples])
        goals = np.array([s['goal'] for s in all_samples])
        first_move_matrices = np.array([s['first_move_matrix'] for s in all_samples])
        
        np.savez_compressed(
            output_file,
            maps=maps,
            starts=starts,
            goals=goals,
            first_move_matrices=first_move_matrices,
            use_8_connected=use_8_connected
        )
        
        print(f"Saved {len(all_samples)} training samples")
        print(f"Map shape: {maps.shape}")
        print(f"First move matrix shape: {first_move_matrices.shape}")
    else:
        print("No samples to save!")


def main():
    parser = argparse.ArgumentParser(
        description="Batch generate first move matrix training data from scenario files"
    )
    parser.add_argument(
        "--scenarios-dir",
        type=str,
        default="3rdparty/Polyanya-main/gppc/gppc-2014/scenarios",
        help="Directory containing .map and .scen files"
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="training_data",
        help="Output directory for training data"
    )
    parser.add_argument(
        "--max-scenarios",
        type=int,
        default=None,
        help="Maximum number of scenarios to process (default: all)"
    )
    parser.add_argument(
        "--8-connected",
        action="store_true",
        help="Use 8-connected grid instead of 4-connected"
    )
    parser.add_argument(
        "--dijkstra",
        action="store_true",
        help="Use Dijkstra instead of BFS (slower but handles weighted graphs)"
    )
    parser.add_argument(
        "--map-filter",
        type=str,
        default=None,
        help="Filter map names (e.g., 'maze-*' to only process maze maps)"
    )
    
    args = parser.parse_args()
    
    batch_process(
        scenarios_dir=args.scenarios_dir,
        output_dir=args.output_dir,
        max_scenarios=args.max_scenarios,
        use_8_connected=args.__dict__['8_connected'],
        use_bfs=not args.dijkstra,
        map_filter=args.map_filter
    )


if __name__ == "__main__":
    main()

