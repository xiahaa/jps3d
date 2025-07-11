import pygame
import numpy as np
import jps_planner_bindings
import sys
import time
import ThetaStarPlanner

# --- CONFIG ---
IMG_PATH = "data/image.png"
RESOLUTION = 1.0  # 1 pixel = 1 cell, adjust if needed

# --- INIT ---
pygame.init()
img_raw = pygame.image.load(IMG_PATH)
img_height, img_width = img_raw.get_height(), img_raw.get_width()
window_size = [min(800, img_width), min(600, img_height)]
screen = pygame.display.set_mode(window_size, pygame.RESIZABLE)
pygame.display.set_caption("JPS Path Planning Interactive")
img = img_raw.convert()
img_array = pygame.surfarray.array3d(img)

# Convert image to map_data: 0=free (black), 100=obstacle (white)
gray = np.dot(img_array[...,:3], [0.299, 0.587, 0.114])
map_data = []
for y in range(img_height):
    for x in range(img_width):
        if gray[x, y] > 200:  # white = obstacle
            map_data.append(1)
        else:
            map_data.append(0)

origin = [0, 0]
dim = [img_width, img_height]

start = None
goal = None
path = []
plan_time = None  # Store last planning time in seconds

def get_scale():
    win_w, win_h = screen.get_size()
    scale_x = win_w / img_width
    scale_y = win_h / img_height
    return scale_x, scale_y

def draw():
    scale_x, scale_y = get_scale()
    scaled_img = pygame.transform.smoothscale(img, screen.get_size())
    screen.blit(scaled_img, (0, 0))
    # Draw Refresh button
    button_rect = pygame.Rect(10, 10, 80, 30)
    pygame.draw.rect(screen, (200, 200, 200), button_rect)
    font = pygame.font.SysFont(None, 28)  # Larger font size
    text = font.render("Refresh", True, (0, 0, 0))
    screen.blit(text, (button_rect.x + 10, button_rect.y + 5))
    # Draw planning time if available
    if plan_time is not None:
        time_font = pygame.font.SysFont(None, 36)  # Even larger for time
        time_text = time_font.render(f"Time: {plan_time:.2f} ms", True, (0, 0, 255))  # Blue color
        screen.blit(time_text, (10, 50))
    # Draw path
    if path:
        for i in range(1, len(path)):
            p1 = (int(path[i-1][0]*scale_x), int(path[i-1][1]*scale_y))
            p2 = (int(path[i][0]*scale_x), int(path[i][1]*scale_y))
            pygame.draw.line(screen, (255, 0, 0), p1, p2, 3)
    # Draw start/goal
    if start:
        pygame.draw.circle(screen, (0, 255, 0), (int(start[0]*scale_x), int(start[1]*scale_y)), 6)
    if goal:
        pygame.draw.circle(screen, (0, 0, 255), (int(goal[0]*scale_x), int(goal[1]*scale_y)), 6)
    pygame.display.flip()

def pixel_to_world(pt):
    # pt is in image pixel coordinates
    return [pt[0] * RESOLUTION, pt[1] * RESOLUTION]

def world_to_pixel(pt):
    return [int(pt[0] / RESOLUTION), int(pt[1] / RESOLUTION)]

def window_to_image_coords(win_pos):
    scale_x, scale_y = get_scale()
    return [int(win_pos[0] / scale_x), int(win_pos[1] / scale_y)]

running = True
selecting = "start"
while running:
    draw()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            break
        elif event.type == pygame.VIDEORESIZE:
            screen = pygame.display.set_mode(event.size, pygame.RESIZABLE)
        elif event.type == pygame.MOUSEBUTTONDOWN:
            mouse_pos = pygame.mouse.get_pos()
            # Check if Refresh button is clicked
            if pygame.Rect(10, 10, 80, 30).collidepoint(mouse_pos):
                start, goal, path = None, None, []
                plan_time = None
                selecting = "start"
                continue
            img_pos = window_to_image_coords(mouse_pos)
            if selecting == "start":
                start = img_pos
                selecting = "goal"
            elif selecting == "goal":
                goal = img_pos
                selecting = "plan"
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_r:
                start, goal, path = None, None, []
                plan_time = None
                selecting = "start"
            if event.key == pygame.K_ESCAPE:
                running = False
                break

    if selecting == "plan" and start and goal:
        # Call planner
        start_w = pixel_to_world(start)
        goal_w = pixel_to_world(goal)
        try:
            # t0 = time.time()
            # result = jps_planner_bindings.plan_2d(
            #     origin, dim, map_data, start_w, goal_w, RESOLUTION, True
            # )
            # t1 = time.time()
            # path = [world_to_pixel(p) for p in result.path]
            # plan_time = result.time_spent #(t1 - t0) * 1000  # ms
            result = ThetaStarPlanner.plan_2d(
                origin, dim, map_data, start_w, goal_w, RESOLUTION, True
            )
            _, path_ret, time_spent = result[0], result[1], result[2]
            path = [world_to_pixel(p) for p in path_ret]
            plan_time = time_spent
        except Exception as e:
            print("Planning failed:", e)
            path = []
            plan_time = None
        selecting = "done"

pygame.quit()