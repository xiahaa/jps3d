import cv2
import numpy as np
from typing import List, Tuple
# planner
import jps_planner_bindings
import ThetaStarPlanner
import BL_JPS
from datetime import datetime
# import warthog
bljps = BL_JPS.BL_JPS()

def bresenham_line(x0, y0, x1, y1):
    """
    Generates the coordinates of a line using Bresenham's algorithm.

    Args:
        x0, y0: Coordinates of the starting point.
        x1, y1: Coordinates of the ending point.

    Yields:
        Tuples (x, y) representing the pixels on the line.
    """
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy

def uncompress_bljps_path(path):
    if not path:
        return []
    uncompressed_path = []
    for p_id in range(len(path)-1):
        current_p = [path[p_id][0],path[p_id][1]]
        uncompressed_path.append(tuple(current_p))

        while current_p[0] != path[p_id+1][0] or current_p[1] != path[p_id+1][1]:
            if path[p_id+1][0] != current_p[0]:
                if path[p_id+1][0] > current_p[0]:
                    current_p[0] += 1
                else:
                    current_p[0] -= 1
                uncompressed_path.append(tuple(current_p))
            if path[p_id+1][1] != current_p[1]:
                if path[p_id+1][1] > current_p[1]:
                    current_p[1] += 1
                else:
                    current_p[1] -= 1
                uncompressed_path.append(tuple(current_p))
    # print (path, uncompressed_path)
    return uncompressed_path

# 假设这是你的底层路径规划函数，比如 A*
def plan(map_img: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
    """
    输入一个二值地图图像（0=障碍，255=自由），起点和终点坐标，返回路径点列表。
    示例：使用 A* 等算法
    """
    # 此处仅为示例逻辑
    print("Calling low-level path planner...")
    jps_path = []
    time_spent = 0.0
    origin = (0, 0)
    dim = [map_img.shape[1], map_img.shape[0]]  # width, height in pixels
    resolution = 1  # meters per pixel (assumed)
    map_data = []
    # transform image to map data, for each pixel, if the value is over 200, it is marked as 100 occipied, otherwise 0 free
    for r in range(dim[1]):  # rows (y)
        row_data = []
        for c in range(dim[0]):  # cols (x)
            pixel_value = map_img[r, c]
            if isinstance(pixel_value, np.ndarray):
                # If pixel is multi-channel (e.g., RGB), take the first channel
                pixel_value = pixel_value[0]
            if pixel_value > 200:  # Assuming a threshold for occupied
                row_data.append(1)  # Occupied
            else:
                row_data.append(0)
        map_data.extend(row_data)

    result = bljps.plan_2d(map_data, width=dim[0], height=dim[1], startX=start[0], startY=start[1], endX=goal[0], endY=goal[1], originX=origin[0], originY=origin[1], resolution=resolution)
    plan_time = result.time_spent
    jps_path = uncompress_bljps_path(result.path)
    time_spent = result.time_spent

    # call bresenham to get traversed points
    final_path = []
    if jps_path:
        # 使用 Bresenham 算法获取路径点
        bresenham_path = []
        for i in range(len(jps_path) - 1):
            x0, y0 = jps_path[i]
            x1, y1 = jps_path[i + 1]
            line_coords = list(bresenham_line(int(x0), int(y0), int(x1), int(y1)))
            final_path.extend(line_coords)

    # show map data
    # import matplotlib.pyplot as plt
    # plt.imshow(np.array(map_data).reshape(dim[1], dim[0]), cmap='gray', origin='lower')
    # plt.colorbar(label='Occupancy (0=free, 100=occupied)')
    # plt.scatter(start[0], start[1], c='green', label='Start', marker='o')
    # plt.scatter(goal[0], goal[1], c='blue', label='Goal', marker='x')
    # if final_path:
    #     jps_path_np = np.array(final_path)
    #     plt.plot(jps_path_np[:, 0], jps_path_np[:, 1], 'r-', label='JPS Path')
    # plt.legend()
    # plt.title('Path Planning Map')
    # plt.xlabel('X (pixels)')
    # plt.ylabel('Y (pixels)')
    # plt.show()

    return final_path

# 构建地图金字塔（每层是上层的 1/4）
def build_pyramid(
    map_img: np.ndarray,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    base_size: Tuple[int, int] = (1000, 1000),
    scale_factor: int = 2
) -> List[np.ndarray]:
    pyramid = {'Map': [], 'Start': [], 'Goal': []}
    curr_map = map_img.copy()
    scale = 1
    while True:
        h, w = curr_map.shape[:2]
        pyramid['Map'].append(curr_map)
        pyramid['Start'].append((start[0] // scale, start[1] // scale))
        pyramid['Goal'].append((goal[0] // scale, goal[1] // scale))
        if h <= base_size[0] and w <= base_size[1]:
            break
        next_h, next_w = h // scale_factor, w // scale_factor
        scale *= scale_factor
        next_map = cv2.resize(curr_map, (next_w, next_h), interpolation=cv2.INTER_AREA)
        next_map[next_map != 255] = 0
        next_map[next_map == 255] = 255
        curr_map = next_map.copy()
    return pyramid

# 映射粗粒度路径到细粒度地图
def map_coarse_to_fine(coarse_path: List[Tuple[int, int]], scale_factor: int) -> List[Tuple[int, int]]:
    fine_path = []
    # 4 neighboring points for each coarse point
    # 这里假设 coarse_path 中的点是整数坐标
    # 将粗粒度坐标映射到细粒度坐标

    # generate [-1-1]*scale_factor, [-1-1]*scale_factor meshgrid
    minx = -1 * scale_factor
    miny = -1 * scale_factor
    maxx = 1 * scale_factor
    maxy = 1 * scale_factor
    dx, dy = np.meshgrid(np.arange(minx, maxx + 1, 1), np.arange(miny, maxy + 1, 1))
    dx = dx.flatten()
    dy = dy.flatten()

    for x, y in coarse_path:
        # 将粗粒度坐标映射到细粒度坐标
        fine_x = x * scale_factor
        fine_y = y * scale_factor

        affected_points = [(fine_x + dx[i], fine_y + dy[i]) for i in range(len(dx))]
        fine_path.extend(affected_points)

    return fine_path

# 膨胀 mask
def expand_mask(mask: np.ndarray, radius: int = 1) -> np.ndarray:
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2 * radius + 1, 2 * radius + 1))
    expanded = cv2.dilate(mask.astype(np.uint8), kernel, iterations=30)
    return expanded > 0

# 使用 mask 规划路径
def plan_with_mask(
    map_img: np.ndarray,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    mask_points: List[Tuple[int, int]],
    expand_radius: int = 5
) -> List[Tuple[int, int]]:
    h, w = map_img.shape
    mask = np.zeros_like(map_img, dtype=np.uint8)
    print(f"Masking {len(mask_points)} points on the map of size {w}x{h}")
    for x, y in mask_points:
        if 0 <= x < w and 0 <= y < h:
            mask[y, x] = 255

    # show the masked map
    # import matplotlib.pyplot as plt
    # plt.imshow(mask, cmap='gray', origin='lower')
    # plt.colorbar(label='Occupancy (0=free, 255=occupied)')
    # plt.scatter(start[0], start[1], c='green', label='Start',   marker='o')
    # plt.scatter(goal[0], goal[1], c='blue', label='Goal', marker='x')
    # plt.legend()
    # plt.title('Masked Map for Path Planning')
    # plt.xlabel('X (pixels)')
    # plt.ylabel('Y (pixels)')
    # plt.show()

    expanded_mask = expand_mask(mask, radius=expand_radius)

    masked_map = map_img.copy()


    masked_map[~expanded_mask] = 255  # 非 mask 区域屏蔽为障碍

    path = plan(masked_map, start, goal)
    return path

# Coarse-to-Fine 主函数
def coarse_to_fine_path_planning(
    map_img: np.ndarray,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    base_size: Tuple[int, int] = (1000, 1000),
    expand_radius: int = 3
) -> List[Tuple[int, int]]:
    h, w = map_img.shape
    if h <= base_size[0] and w <= base_size[1]:
        print("直接调用底层路径规划")
        path = plan(map_img, start, goal)
        return path
    scale_factor = 4
    pyramid = build_pyramid(map_img, start, goal, base_size, scale_factor=scale_factor)
    # print(f"构建了 {len(pyramid['Map'])} 层地图金字塔")
    # for i, p in enumerate(pyramid['Map']):
    #     print(f"第 {i} 层地图大小: {p.shape}")

    # 从最粗层开始规划
    from datetime import datetime
    stime = datetime.now()
    coarse_path = plan(pyramid['Map'][-1],
                       pyramid['Start'][-1],
                       pyramid['Goal'][-1])
    etime = datetime.now()
    print(f"粗粒度路径规划耗时: {(etime - stime).total_seconds()} 秒")

    # 逐层细化, 0-1, 1-0
    number_layers = len(pyramid['Map'])
    for level in reversed(range(number_layers-1)):
        print(f"细化到第 {level} 层")
        fine_map = pyramid['Map'][level]
        fine_start = pyramid['Start'][level]
        fine_goal = pyramid['Goal'][level]

        # 映射路径点到当前层
        stime = datetime.now()
        mapped_points = map_coarse_to_fine(coarse_path, scale_factor)
        # 规划新路径
        coarse_path = plan_with_mask(
            fine_map,
            fine_start,
            fine_goal,
            mapped_points,
            expand_radius=expand_radius
        )
        etime = datetime.now()
        print(f"细化层 {level} 路径规划耗时: {(etime - stime).total_seconds()} 秒")

    # 最终在原始地图上精修

    final_path = coarse_path

    return final_path


# 示例主程序
if __name__ == "__main__":
    # 加载地图
    map_path = "./data/image.png"
    raw_map = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    dim = [raw_map.shape[1], raw_map.shape[0]]  # width, height in pixels
    resolution = 1  # meters per pixel (assumed)
    origin = [0.0,0.0]  # x, y of the map origin

    map_data = []
    # transform image to map data, for each pixel, if the value is over 200, it is marked as 100 occipied, otherwise 0 free
    for r in range(dim[1]):  # rows (y)
        row_data = []
        for c in range(dim[0]):  # cols (x)
            pixel_value = raw_map[r, c]
            if isinstance(pixel_value, np.ndarray):
                # If pixel is multi-channel (e.g., RGB), take the first channel
                pixel_value = pixel_value[0]
            if pixel_value > 200:  # Assuming a threshold for occupied
                row_data.append(1)  # Occupied
            else:
                row_data.append(0)
        map_data.extend(row_data)

    # random pick start and goal points that are free and at least 1000 pixels apart
    free_indices = [i for i, v in enumerate(map_data) if v == 0]

    import random
    start_index = random.choice(free_indices)
    goal_index = random.choice(free_indices)
    while abs(start_index - goal_index) < 1000:  # Ensure at least 1000 pixels apart
        goal_index = random.choice(free_indices)
    start_w = [start_index % dim[0] * resolution, start_index // dim[0] * resolution]
    goal_w = [goal_index % dim[0] * resolution, goal_index // dim[0] * resolution]

    print(f"Map Origin: {origin}")
    print(f"Map Dimensions (cells): {dim}")
    print(f"Map Resolution: {resolution}")
    print(f"Start (world): {start_w}")
    print(f"Goal (world): {goal_w}")

    # 调用路径规划
    final_path = coarse_to_fine_path_planning(raw_map, start_w, goal_w)

    # print("最终路径长度:", len(final_path))
    # print("路径示例:", final_path[:10])

    # show planning result using matplotlib
    import matplotlib.pyplot as plt
    plt.imshow(np.array(map_data).reshape(dim[1], dim[0]), cmap='gray', origin='lower')
    plt.colorbar(label='Occupancy (0=free, 100=occupied)')
    if final_path:
        jps_path_np = np.array(final_path)
        plt.plot(jps_path_np[:, 0] / resolution, jps_path_np[:, 1] / resolution, 'r-', label='JPS Path')
    plt.scatter(start_w[0] / resolution, start_w[1] / resolution, c='green', label='Start', marker='o')
    plt.scatter(goal_w[0] / resolution, goal_w[1] / resolution, c='blue', label='Goal', marker='x')
    plt.legend()
    plt.title('JPS Path Planning Result')
    plt.xlabel('X (cells)')
    plt.ylabel('Y (cells)')
    plt.show()
