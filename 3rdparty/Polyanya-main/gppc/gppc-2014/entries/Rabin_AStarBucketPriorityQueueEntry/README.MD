# Rabin A* Bucket Priority Queue Entry

This directory contains an implementation of the A* search algorithm using a bucket-based priority queue, as submitted by Daniel Rabin for the GPPC 2014 competition.

## Overview

The A* algorithm is a popular pathfinding and graph traversal algorithm, widely used due to its performance and accuracy. This implementation utilizes a bucket-based priority queue to optimize the management of open nodes, potentially improving performance in certain scenarios.

## Features

- Efficient bucket-based priority queue for managing open nodes.
- Designed for grid-based pathfinding problems.
- Compatible with the GPPC 2014 benchmark framework.

## Usage

### Compilation

You can build the project using the provided `Makefile`. Run one of the following commands in this directory:

```sh
make dev      # Build with debug flags
make fast     # Build with optimizations
make profile  # Build with profiling flags
```

The compiled binaries and libraries will be placed in the `bin/` and `lib/` directories.

Alternatively, you can compile manually:
```sh
g++ -O2 -std=c++11 -o astar_bucket main.cpp
```

### Running

Execute the compiled binary with the required input files:
```sh
./astar_bucket <input_map> <start_x> <start_y> <goal_x> <goal_y>
```
Or, if using the Makefile output:
```sh
./bin/main <input_map> <start_x> <start_y> <goal_x> <goal_y>
```

### Integration

This entry is designed to be used within the GPPC 2014 evaluation framework. Refer to the main Polyanya or GPPC documentation for integration details.

## File Structure

- `main.cpp` – Entry point and main logic for the algorithm.
- `bucket_priority_queue.h` – Implementation of the bucket-based priority queue.
- `Makefile` – Build instructions.
- `README.MD` – This documentation file.

## References

- [GPPC 2014 Competition](https://movingai.com/GPPC/)
- [A* Search Algorithm (Wikipedia)](https://en.wikipedia.org/wiki/A*_search_algorithm)

## Author

Daniel Rabin
