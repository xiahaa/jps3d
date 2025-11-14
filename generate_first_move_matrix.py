"""
Generate First Move Matrix for Pathfinding Training Data

This script generates a first move matrix given a map, start point, and goal point.
The first move matrix indicates the direction of the first move from each cell
to reach the goal, which can be used as training data for neural networks.
"""

import numpy as np
import cv2
import heapq
from typing import Tuple, List, Optional
from collections import deque


# Direction encoding for 4-connected grid
# 0: North (up, -y), 1: South (down, +y), 2: East (right, +x), 3: West (left, -x)
DIRECTIONS_4 = [
    (0, -1),   # 0: North
    (0, 1),    # 1: South
    (1, 0),    # 2: East
    (-1, 0),   # 3: West
]

# Direction encoding for 8-connected grid
DIRECTIONS_8 = [
    (0, -1),   # 0: North
    (0, 1),    # 1: South
    (1, 0),    # 2: East
    (-1, 0),   # 3: West
    (1, -1),   # 4: Northeast
    (-1, -1),  # 5: Northwest
    (1, 1),    # 6: Southeast
    (-1, 1),   # 7: Southwest
]

OBSTACLE_VALUE = -1
UNREACHABLE_VALUE = -2


def compute_first_move_matrix_dijkstra(
    map_img: np.ndarray,
    goal: Tuple[int, int],
    use_8_connected: bool = False
) -> np.ndarray:
    """
    Compute first move matrix using Dijkstra's algorithm backwards from the goal.

    Args:
        map_img: 2D numpy array where 0=free, >200=obstacle (or binary: 0=free, 1=obstacle)
        goal: (x, y) goal position in pixel coordinates
        use_8_connected: If True, use 8-connected grid, else 4-connected

    Returns:
        2D numpy array of first move directions:
        - Direction codes (0-3 for 4-connected, 0-7 for 8-connected)
        - OBSTACLE_VALUE (-1) for obstacle cells
        - UNREACHABLE_VALUE (-2) for unreachable cells
    """
    height, width = map_img.shape
    directions = DIRECTIONS_8 if use_8_connected else DIRECTIONS_4

    # Initialize first move matrix
    first_move_matrix = np.full((height, width), UNREACHABLE_VALUE, dtype=np.int32)

    # Mark obstacles
    obstacle_mask = (map_img > 200) if map_img.max() > 1 else (map_img == 1)
    first_move_matrix[obstacle_mask] = OBSTACLE_VALUE

    # Check if goal is valid
    goal_y, goal_x = goal[1], goal[0]  # Note: goal is (x, y), but array is (y, x)
    if goal_x < 0 or goal_x >= width or goal_y < 0 or goal_y >= height:
        return first_move_matrix
    if obstacle_mask[goal_y, goal_x]:
        return first_move_matrix

    # Dijkstra backwards from goal
    # We'll store (distance, y, x, first_move_direction)
    # For the goal itself, we use a special value (e.g., 0 meaning "arrived")
    pq = [(0, goal_y, goal_x, 0)]  # distance, y, x, first_move (0 for goal itself)
    visited = np.zeros((height, width), dtype=bool)
    distances = np.full((height, width), np.inf)
    distances[goal_y, goal_x] = 0

    # For goal cell, mark as 0 (arrived)
    first_move_matrix[goal_y, goal_x] = 0

    while pq:
        dist, y, x, first_move = heapq.heappop(pq)

        if visited[y, x]:
            continue
        visited[y, x] = True

        # Explore neighbors
        for dir_idx, (dx, dy) in enumerate(directions):
            nx, ny = x + dx, y + dy

            # Check bounds
            if nx < 0 or nx >= width or ny < 0 or ny >= height:
                continue

            # Check if obstacle
            if obstacle_mask[ny, nx]:
                continue

            # Check if already visited
            if visited[ny, nx]:
                continue

            # Calculate new distance
            # For 8-connected, diagonal moves cost sqrt(2), else 1
            move_cost = np.sqrt(2) if (dx != 0 and dy != 0) else 1.0
            new_dist = dist + move_cost

            # Update if we found a shorter path
            if new_dist < distances[ny, nx]:
                distances[ny, nx] = new_dist
                # The first move from (ny, nx) is the direction we came from
                # But we need to reverse it: if we came from (y, x) to (ny, nx),
                # the first move from (ny, nx) to goal is the opposite direction
                # Actually, we're going backwards, so the first move from neighbor
                # to goal is the direction we took to get to the neighbor
                # Wait, let me reconsider...
                # We're at (y, x), and we're exploring (ny, nx).
                # From (ny, nx), to get to goal, we first move in direction (dx, dy)
                # which is direction dir_idx
                first_move_matrix[ny, nx] = dir_idx
                heapq.heappush(pq, (new_dist, ny, nx, dir_idx))

    return first_move_matrix


def compute_first_move_matrix_bfs(
    map_img: np.ndarray,
    goal: Tuple[int, int],
    use_8_connected: bool = False
) -> np.ndarray:
    """
    Compute first move matrix using BFS backwards from the goal.
    This is faster than Dijkstra for uniform-cost grids.

    Args:
        map_img: 2D numpy array where 0=free, >200=obstacle (or binary: 0=free, 1=obstacle)
        goal: (x, y) goal position in pixel coordinates
        use_8_connected: If True, use 8-connected grid, else 4-connected

    Returns:
        2D numpy array of first move directions
    """
    height, width = map_img.shape
    directions = DIRECTIONS_8 if use_8_connected else DIRECTIONS_4

    # Initialize first move matrix
    first_move_matrix = np.full((height, width), UNREACHABLE_VALUE, dtype=np.int32)

    # Mark obstacles
    obstacle_mask = (map_img > 200) if map_img.max() > 1 else (map_img == 1)
    first_move_matrix[obstacle_mask] = OBSTACLE_VALUE

    # Check if goal is valid
    goal_y, goal_x = goal[1], goal[0]  # Note: goal is (x, y), but array is (y, x)
    if goal_x < 0 or goal_x >= width or goal_y < 0 or goal_y >= height:
        return first_move_matrix
    if obstacle_mask[goal_y, goal_x]:
        return first_move_matrix

    # BFS backwards from goal
    queue = deque([(goal_y, goal_x)])
    visited = np.zeros((height, width), dtype=bool)
    visited[goal_y, goal_x] = True

    # For goal cell, mark as 0 (arrived)
    first_move_matrix[goal_y, goal_x] = 0

    while queue:
        y, x = queue.popleft()

        # Explore neighbors
        for dir_idx, (dx, dy) in enumerate(directions):
            nx, ny = x + dx, y + dy

            # Check bounds
            if nx < 0 or nx >= width or ny < 0 or ny >= height:
                continue

            # Check if obstacle
            if obstacle_mask[ny, nx]:
                continue

            # Check if already visited
            if visited[ny, nx]:
                continue

            # Mark as visited and set first move direction
            visited[ny, nx] = True
            first_move_matrix[ny, nx] = dir_idx
            queue.append((ny, nx))

    return first_move_matrix


def visualize_first_move_matrix(
    map_img: np.ndarray,
    first_move_matrix: np.ndarray,
    start: Optional[Tuple[int, int]] = None,
    goal: Optional[Tuple[int, int]] = None,
    use_8_connected: bool = False
) -> np.ndarray:
    """
    Visualize the first move matrix as an RGB image.

    Args:
        map_img: Original map image
        first_move_matrix: First move matrix
        start: Optional start position for visualization
        goal: Optional goal position for visualization
        use_8_connected: Whether 8-connected was used

    Returns:
        RGB image for visualization
    """
    height, width = first_move_matrix.shape
    vis_img = np.zeros((height, width, 3), dtype=np.uint8)

    # Color map for directions
    if use_8_connected:
        colors = [
            (0, 0, 255),    # 0: North - Red
            (0, 255, 0),    # 1: South - Green
            (255, 0, 0),    # 2: East - Blue
            (255, 255, 0),  # 3: West - Cyan
            (255, 0, 255),  # 4: Northeast - Magenta
            (0, 255, 255),  # 5: Northwest - Yellow
            (128, 128, 0),  # 6: Southeast
            (128, 0, 128),  # 7: Southwest
        ]
    else:
        colors = [
            (0, 0, 255),    # 0: North - Red
            (0, 255, 0),    # 1: South - Green
            (255, 0, 0),    # 2: East - Blue
            (255, 255, 0),  # 3: West - Cyan
        ]

    # Draw directions
    for y in range(height):
        for x in range(width):
            val = first_move_matrix[y, x]
            if val == OBSTACLE_VALUE:
                vis_img[y, x] = (128, 128, 128)  # Gray for obstacles
            elif val == UNREACHABLE_VALUE:
                vis_img[y, x] = (0, 0, 0)  # Black for unreachable
            elif 0 <= val < len(colors):
                vis_img[y, x] = colors[val]

    # Draw start and goal
    if start is not None:
        sx, sy = start[0], start[1]
        if 0 <= sx < width and 0 <= sy < height:
            cv2.circle(vis_img, (sx, sy), 5, (0, 255, 255), -1)  # Yellow circle
    if goal is not None:
        gx, gy = goal[0], goal[1]
        if 0 <= gx < width and 0 <= gy < height:
            cv2.circle(vis_img, (gx, gy), 5, (255, 255, 255), -1)  # White circle

    return vis_img


def generate_training_sample(
    map_img: np.ndarray,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    use_8_connected: bool = False,
    use_bfs: bool = True
) -> dict:
    """
    Generate a single training sample with first move matrix.

    Args:
        map_img: 2D numpy array map (0=free, >200 or 1=obstacle)
        start: (x, y) start position
        goal: (x, y) goal position
        use_8_connected: Whether to use 8-connected grid
        use_bfs: If True, use BFS (faster for uniform cost), else Dijkstra

    Returns:
        Dictionary containing:
        - 'map': map image array
        - 'start': start position
        - 'goal': goal position
        - 'first_move_matrix': first move matrix
        - 'use_8_connected': whether 8-connected was used
    """
    if use_bfs:
        first_move_matrix = compute_first_move_matrix_bfs(
            map_img, goal, use_8_connected
        )
    else:
        first_move_matrix = compute_first_move_matrix_dijkstra(
            map_img, goal, use_8_connected
        )

    return {
        'map': map_img.copy(),
        'start': start,
        'goal': goal,
        'first_move_matrix': first_move_matrix,
        'use_8_connected': use_8_connected
    }


# Example usage
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # Load map
    map_path = "data/image.png"
    map_img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    # resize to 1024x1024
    map_img = cv2.resize(map_img, (1024, 1024))

    if map_img is None:
        print(f"Error: Could not load map from {map_path}")
        exit(1)

    print(f"Map size: {map_img.shape}")

    # Example start and goal
    height, width = map_img.shape
    start = (width // 4, height // 4)
    goal = (3 * width // 4, 3 * height // 4)

    # Generate first move matrix
    print("Generating first move matrix...")
    sample = generate_training_sample(
        map_img, start, goal,
        use_8_connected=False,  # Use 4-connected for now
        use_bfs=True
    )

    first_move_matrix = sample['first_move_matrix']

    # Visualize
    vis_img = visualize_first_move_matrix(
        map_img, first_move_matrix, start, goal,
        use_8_connected=False
    )

    # Save visualization
    cv2.imwrite("first_move_matrix_visualization.png", vis_img)
    print("Saved visualization to first_move_matrix_visualization.png")

    # Save first move matrix as numpy array
    np.save("first_move_matrix.npy", first_move_matrix)
    print("Saved first move matrix to first_move_matrix.npy")

    # Print statistics
    unique, counts = np.unique(first_move_matrix, return_counts=True)
    print("\nFirst move matrix statistics:")
    for val, count in zip(unique, counts):
        if val == OBSTACLE_VALUE:
            print(f"  Obstacles: {count}")
        elif val == UNREACHABLE_VALUE:
            print(f"  Unreachable: {count}")
        else:
            print(f"  Direction {val}: {count}")

    # Display
    plt.figure(figsize=(15, 5))

    plt.subplot(1, 3, 1)
    plt.imshow(map_img, cmap='gray')
    plt.scatter([start[0]], [start[1]], c='green', s=100, marker='o', label='Start')
    plt.scatter([goal[0]], [goal[1]], c='red', s=100, marker='x', label='Goal')
    plt.title('Original Map')
    plt.legend()

    plt.subplot(1, 3, 2)
    plt.imshow(first_move_matrix, cmap='viridis')
    plt.colorbar(label='First Move Direction')
    plt.title('First Move Matrix')

    plt.subplot(1, 3, 3)
    plt.imshow(vis_img)
    plt.title('First Move Visualization')

    plt.tight_layout()
    plt.savefig("first_move_matrix_comparison.png", dpi=150)
    print("Saved comparison figure to first_move_matrix_comparison.png")
    plt.show()
