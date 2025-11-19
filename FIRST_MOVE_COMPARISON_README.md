# First Move Matrix Comparison

This document explains how to compare the Python and C++ implementations of the first move matrix generation.

## Overview

The comparison framework allows you to:
1. Generate first move matrices using the Python implementation (`generate_first_move_matrix.py`)
2. Extract first move matrices from the C++ implementation (FirstMoveCompression)
3. Compare the results to ensure both implementations produce the same results

## Setup

### 1. Build Pybind11 Wrapper

First, build the pybind11 wrapper module:

```bash
cd 3rdparty/Polyanya-main/gppc/gppc-2014/entries/FirstMoveCompression
pip install pybind11
python setup.py build_ext --inplace
```

This will create a `cpp_first_move_matrix` module (`.so` on Linux, `.pyd` on Windows) that can be imported in Python.

### 2. Compile C++ Preprocessing Binary (if not already done)

```bash
cd 3rdparty/Polyanya-main/gppc/gppc-2014/entries/FirstMoveCompression
./build.sh
```

This will create binaries like `dfs_src`, `detailed_dfs_src`, etc.

## Usage

### Basic Comparison

```bash
python compare_first_move_matrix.py --map data/image.png --goal 512 512
```

This will:
- Load the map from `data/image.png`
- Generate a first move matrix using the Python implementation for goal (512, 512)
- Attempt to extract the first move matrix from the C++ implementation
- Compare the results and generate a report

### Options

- `--map PATH`: Path to map image file (default: `data/image.png`)
- `--goal X Y`: Goal position in pixel coordinates (default: center of map)
- `--use-8-connected`: Use 8-connected grid instead of 4-connected
- `--use-dijkstra`: Use Dijkstra's algorithm instead of BFS (slower but handles weighted graphs)
- `--cpp-binary PATH`: Path to C++ preprocessing binary (auto-detected if not specified)
- `--output PATH`: Output file for comparison results (default: `first_move_comparison_results.json`)

### Example

```bash
# Compare with 8-connected grid
python compare_first_move_matrix.py \
    --map data/image.png \
    --goal 768 256 \
    --use-8-connected \
    --output comparison_results.json
```

## Understanding the Results

The comparison script outputs:

1. **Statistics**: Total cells, obstacle cells, free cells
2. **Python Matrix Statistics**: Distribution of directions in the Python matrix
3. **Comparison Results** (if C++ matrix available):
   - Match rate: Percentage of cells where both implementations agree
   - Mismatches: Sample positions where implementations differ

### Value Encoding

- `-1`: Obstacle cell
- `-2`: Unreachable cell
- `0-3`: Direction codes for 4-connected (North, South, East, West)
- `0-7`: Direction codes for 8-connected (adds diagonals)

## Differences Between Implementations

### Python Implementation
- Works on a grid representation (all cells, including obstacles)
- Computes first move from all cells to a single goal
- Uses BFS or Dijkstra backwards from the goal
- Stores result as a 2D numpy array

### C++ Implementation
- Works on a graph representation (only free cells are nodes)
- Precomputes first moves from all nodes to all nodes
- Uses Dijkstra from each source node
- Stores result in compressed format (CPD - Compressed Path Database)
- Can return any valid first move if multiple shortest paths exist

### Expected Differences

1. **Multiple Shortest Paths**: If there are multiple shortest paths, the C++ implementation may choose a different first move than Python. Both are valid.

2. **Node vs Grid**: C++ only stores first moves for free cells (nodes), while Python stores for all cells including obstacles.

3. **Direction Encoding**: Both use similar direction encodings, but the exact mapping should be verified.

## Troubleshooting

### C++ Binary Not Found

If you see "C++ preprocessing binary not found":
- Make sure you've compiled the C++ code using `build.sh`
- Or specify the path manually with `--cpp-binary`

### C++ Extraction Binary Not Found

If you see "C++ extraction binary not found":
- Compile `extract_first_move_matrix.cpp` as shown in Setup section
- Make sure the binary is in the FirstMoveCompression directory

### Preprocessing Takes Too Long

For large maps, preprocessing can take a long time. The script has a 5-minute timeout. For very large maps, you may need to:
- Preprocess manually first
- Use a smaller test map
- Increase the timeout in the script

## Files

- `compare_first_move_matrix.py`: Main comparison script
- `generate_first_move_matrix.py`: Python implementation
- `extract_first_move_matrix.cpp`: C++ helper to extract first move matrix
- `3rdparty/.../FirstMoveCompression/`: C++ implementation directory

## Next Steps

To fully validate the implementations:
1. Run comparisons on multiple maps
2. Test with different goal positions
3. Compare both 4-connected and 8-connected grids
4. Verify that any differences are due to multiple valid shortest paths
