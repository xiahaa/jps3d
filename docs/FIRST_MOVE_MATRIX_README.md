# First Move Matrix Training Data Generation

This directory contains scripts to generate first move matrices for pathfinding neural network training.

## Overview

Instead of run-length encoding, these scripts generate **first move matrices** that indicate the direction of the first move from each cell to reach the goal. This is ideal for training neural networks to predict pathfinding behavior.

## Files

- `generate_first_move_matrix.py`: Core functions to compute first move matrices from a map, start, and goal
- `batch_generate_first_move_training_data.py`: Batch processing script for generating training data from scenario files
- `example_use_training_data.py`: Example script showing how to load and use the generated training data

## Quick Start

### 1. Generate Training Data from Scenarios

```bash
# Process all scenarios in the gppc-2014 directory
python batch_generate_first_move_training_data.py \
    --scenarios-dir 3rdparty/Polyanya-main/gppc/gppc-2014/scenarios \
    --output-dir training_data

# Process only first 1000 scenarios
python batch_generate_first_move_training_data.py \
    --scenarios-dir 3rdparty/Polyanya-main/gppc/gppc-2014/scenarios \
    --output-dir training_data \
    --max-scenarios 1000

# Process only maze maps with 8-connected grid
python batch_generate_first_move_training_data.py \
    --scenarios-dir 3rdparty/Polyanya-main/gppc/gppc-2014/scenarios \
    --output-dir training_data \
    --map-filter "maze-*" \
    --8-connected
```

### 2. Generate Single Sample

```python
from generate_first_move_matrix import generate_training_sample
import cv2

# Load map
map_img = cv2.imread("data/image.png", cv2.IMREAD_GRAYSCALE)

# Define start and goal
start = (100, 100)  # (x, y)
goal = (500, 500)   # (x, y)

# Generate first move matrix
sample = generate_training_sample(
    map_img, start, goal,
    use_8_connected=False,  # Use 4-connected grid
    use_bfs=True           # Use BFS (faster than Dijkstra)
)

# Access results
first_move_matrix = sample['first_move_matrix']
print(f"First move matrix shape: {first_move_matrix.shape}")
```

### 3. Load and Use Training Data

```python
from example_use_training_data import load_training_data, prepare_for_neural_network

# Load data
data = load_training_data("training_data/first_move_training_data.npz")

# Prepare for neural network
prepared = prepare_for_neural_network(data)

# Use in training loop
inputs = prepared['inputs']      # (N, H, W, 3) - map, start, goal channels
targets = prepared['targets']     # (N, H, W, num_classes) - one-hot encoded
```

## First Move Matrix Format

The first move matrix is a 2D array where each cell contains:

- **Direction codes** (0-3 for 4-connected, 0-7 for 8-connected):
  - 0: North (up, -y)
  - 1: South (down, +y)
  - 2: East (right, +x)
  - 3: West (left, -x)
  - 4-7: Diagonal directions (only for 8-connected)
- **-1**: Obstacle cell
- **-2**: Unreachable cell (no path to goal)

### 4-Connected Directions

```
    0 (North)
        ↑
3 ← (West) (East) → 2
        ↓
    1 (South)
```

### 8-Connected Directions

```
5 (NW)  0 (N)  4 (NE)
        ↑
3 (W)   •   2 (E)
        ↓
7 (SW)  1 (S)  6 (SE)
```

## Algorithm

The first move matrix is computed using **BFS (Breadth-First Search)** or **Dijkstra's algorithm** backwards from the goal:

1. Start at the goal position
2. For each cell, record the direction taken to reach it from the goal
3. This direction is the "first move" from that cell toward the goal

This is more efficient than computing paths from every cell to the goal individually.

## Data Format

The generated `.npz` file contains:

- `maps`: Array of maps (N, H, W) - uint8, 0=free, 255=obstacle
- `starts`: Array of start positions (N, 2) - (x, y) coordinates
- `goals`: Array of goal positions (N, 2) - (x, y) coordinates
- `first_move_matrices`: Array of first move matrices (N, H, W) - int32
- `use_8_connected`: Boolean flag

## Neural Network Input/Output

### Input
- **Map**: (H, W) - occupancy grid
- **Start position**: (H, W) - binary map with 1 at start
- **Goal position**: (H, W) - binary map with 1 at goal

Combined: (H, W, 3) tensor

### Output
- **First move matrix**: (H, W, num_classes) - one-hot encoded
  - num_classes = num_directions + 2 (for obstacle and unreachable)

For 4-connected: (H, W, 6) - 4 directions + obstacle + unreachable
For 8-connected: (H, W, 10) - 8 directions + obstacle + unreachable

## Example Neural Network Architecture

```python
import torch
import torch.nn as nn

class FirstMovePredictor(nn.Module):
    def __init__(self, num_classes=6):  # 4 directions + obstacle + unreachable
        super().__init__()
        self.encoder = nn.Sequential(
            nn.Conv2d(3, 64, 3, padding=1),  # 3 input channels: map, start, goal
            nn.ReLU(),
            nn.Conv2d(64, 128, 3, padding=1),
            nn.ReLU(),
            nn.Conv2d(128, 256, 3, padding=1),
            nn.ReLU(),
        )
        self.decoder = nn.Sequential(
            nn.Conv2d(256, 128, 3, padding=1),
            nn.ReLU(),
            nn.Conv2d(128, 64, 3, padding=1),
            nn.ReLU(),
            nn.Conv2d(64, num_classes, 1),  # Output: direction classes
        )
    
    def forward(self, x):
        # x: (B, 3, H, W)
        features = self.encoder(x)
        output = self.decoder(features)
        return output  # (B, num_classes, H, W)
```

## Performance Notes

- **BFS** is faster for uniform-cost grids (recommended for most cases)
- **Dijkstra** handles weighted graphs but is slower
- **4-connected** is faster and uses less memory than 8-connected
- Processing time scales with map size and number of scenarios

## Troubleshooting

### Map file not found
- Ensure `.map` and `.scen` files are in the same directory
- Check that map filenames in scenario files match actual map files

### Out of memory
- Reduce `--max-scenarios` to process fewer samples
- Process maps in smaller batches
- Use 4-connected instead of 8-connected

### Invalid positions
- Some scenarios may have start/goal positions outside map bounds or on obstacles
- These are automatically skipped during processing

## Citation

If you use the gppc-2014 scenarios, please cite:

```
@inproceedings{DBLP:conf/socs/SturtevantTTUKS15,
  author    = {Nathan R. Sturtevant and others},
  title     = {The Grid-Based Path Planning Competition: 2014 Entries and Results},
  booktitle = {Proceedings of the Eighth Annual Symposium on Combinatorial Search},
  year      = {2015}
}
```

