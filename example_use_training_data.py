"""
Example: How to Load and Use First Move Matrix Training Data

This script demonstrates how to load the generated training data and use it
for training a neural network.
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


def load_training_data(data_path: str):
    """
    Load training data from .npz file.
    
    Args:
        data_path: Path to .npz file
    
    Returns:
        Dictionary with keys:
        - maps: Array of maps (N, H, W)
        - starts: Array of start positions (N, 2) - (x, y)
        - goals: Array of goal positions (N, 2) - (x, y)
        - first_move_matrices: Array of first move matrices (N, H, W)
        - use_8_connected: Boolean indicating if 8-connected was used
    """
    data = np.load(data_path, allow_pickle=True)
    return {
        'maps': data['maps'],
        'starts': data['starts'],
        'goals': data['goals'],
        'first_move_matrices': data['first_move_matrices'],
        'use_8_connected': bool(data['use_8_connected'])
    }


def visualize_sample(data: dict, index: int):
    """
    Visualize a single training sample.
    
    Args:
        data: Training data dictionary
        index: Index of sample to visualize
    """
    map_img = data['maps'][index]
    start = data['starts'][index]
    goal = data['goals'][index]
    first_move_matrix = data['first_move_matrices'][index]
    
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    
    # Original map
    axes[0].imshow(map_img, cmap='gray')
    axes[0].scatter([start[0]], [start[1]], c='green', s=100, marker='o', label='Start')
    axes[0].scatter([goal[0]], [goal[1]], c='red', s=100, marker='x', label='Goal')
    axes[0].set_title('Map with Start/Goal')
    axes[0].legend()
    
    # First move matrix
    im = axes[1].imshow(first_move_matrix, cmap='viridis')
    axes[1].set_title('First Move Matrix')
    plt.colorbar(im, ax=axes[1], label='Direction')
    
    # Overlay
    axes[2].imshow(map_img, cmap='gray', alpha=0.5)
    im2 = axes[2].imshow(first_move_matrix, cmap='viridis', alpha=0.5)
    axes[2].scatter([start[0]], [start[1]], c='green', s=100, marker='o', label='Start')
    axes[2].scatter([goal[0]], [goal[1]], c='red', s=100, marker='x', label='Goal')
    axes[2].set_title('Overlay')
    axes[2].legend()
    plt.colorbar(im2, ax=axes[2], label='Direction')
    
    plt.tight_layout()
    plt.show()


def get_training_batch(data: dict, batch_size: int = 32, start_idx: int = 0):
    """
    Get a batch of training data.
    
    Args:
        data: Training data dictionary
        batch_size: Size of batch
        start_idx: Starting index
    
    Returns:
        Tuple of (maps_batch, starts_batch, goals_batch, targets_batch)
        where targets_batch is the first_move_matrices
    """
    end_idx = min(start_idx + batch_size, len(data['maps']))
    
    maps_batch = data['maps'][start_idx:end_idx]
    starts_batch = data['starts'][start_idx:end_idx]
    goals_batch = data['goals'][start_idx:end_idx]
    targets_batch = data['first_move_matrices'][start_idx:end_idx]
    
    return maps_batch, starts_batch, goals_batch, targets_batch


def prepare_for_neural_network(data: dict):
    """
    Prepare data for neural network training.
    
    This function:
    1. Normalizes map values (0-255 -> 0-1)
    2. One-hot encodes first move directions
    3. Creates input features combining map, start, and goal
    
    Args:
        data: Training data dictionary
    
    Returns:
        Dictionary with prepared data
    """
    # Normalize maps
    maps_normalized = data['maps'].astype(np.float32) / 255.0
    
    # Get dimensions
    num_samples, height, width = maps_normalized.shape
    
    # Determine number of direction classes
    if data['use_8_connected']:
        num_directions = 8
    else:
        num_directions = 4
    
    # One-hot encode first move matrices
    # Shape: (N, H, W, num_directions + 2)  # +2 for obstacle and unreachable
    first_move_onehot = np.zeros(
        (num_samples, height, width, num_directions + 2),
        dtype=np.float32
    )
    
    for i in range(num_samples):
        fmm = data['first_move_matrices'][i]
        for y in range(height):
            for x in range(width):
                val = fmm[y, x]
                if val == -1:  # OBSTACLE
                    first_move_onehot[i, y, x, num_directions] = 1.0
                elif val == -2:  # UNREACHABLE
                    first_move_onehot[i, y, x, num_directions + 1] = 1.0
                elif 0 <= val < num_directions:
                    first_move_onehot[i, y, x, val] = 1.0
    
    # Create position maps (one channel for start, one for goal)
    start_maps = np.zeros((num_samples, height, width), dtype=np.float32)
    goal_maps = np.zeros((num_samples, height, width), dtype=np.float32)
    
    for i in range(num_samples):
        sx, sy = data['starts'][i]
        gx, gy = data['goals'][i]
        if 0 <= sx < width and 0 <= sy < height:
            start_maps[i, sy, sx] = 1.0
        if 0 <= gx < width and 0 <= gy < height:
            goal_maps[i, gy, gx] = 1.0
    
    # Combine into input: (map, start_map, goal_map) -> (N, H, W, 3)
    inputs = np.stack([maps_normalized, start_maps, goal_maps], axis=-1)
    
    return {
        'inputs': inputs,  # (N, H, W, 3)
        'targets': first_move_onehot,  # (N, H, W, num_directions + 2)
        'targets_class': data['first_move_matrices']  # (N, H, W) for class indices
    }


# Example usage
if __name__ == "__main__":
    # Load training data
    data_path = "training_data/first_move_training_data.npz"
    
    if not Path(data_path).exists():
        print(f"Training data not found at {data_path}")
        print("Please run batch_generate_first_move_training_data.py first")
        exit(1)
    
    print("Loading training data...")
    data = load_training_data(data_path)
    
    print(f"Loaded {len(data['maps'])} samples")
    print(f"Map shape: {data['maps'].shape}")
    print(f"First move matrix shape: {data['first_move_matrices'].shape}")
    print(f"Using 8-connected: {data['use_8_connected']}")
    
    # Visualize a few samples
    print("\nVisualizing sample 0...")
    visualize_sample(data, 0)
    
    if len(data['maps']) > 1:
        print("Visualizing sample 1...")
        visualize_sample(data, 1)
    
    # Prepare for neural network
    print("\nPreparing data for neural network...")
    prepared = prepare_for_neural_network(data)
    
    print(f"Input shape: {prepared['inputs'].shape}")
    print(f"Target shape: {prepared['targets'].shape}")
    print(f"Target class shape: {prepared['targets_class'].shape}")
    
    # Example: Get a batch
    print("\nGetting a training batch...")
    maps_batch, starts_batch, goals_batch, targets_batch = get_training_batch(
        data, batch_size=4
    )
    print(f"Batch shapes:")
    print(f"  Maps: {maps_batch.shape}")
    print(f"  Starts: {starts_batch.shape}")
    print(f"  Goals: {goals_batch.shape}")
    print(f"  Targets: {targets_batch.shape}")
    
    print("\nData is ready for neural network training!")

