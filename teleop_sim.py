# import pygame
# import numpy as np
# import math
# import json
# from scipy.ndimage import gaussian_filter
# from a_star_planner import a_star
# from world import make_environment
# from robot import Robot

# # === Constants ===
# WINDOW_W, WINDOW_H = 1000, 500
# ENV_SIZE_M = 10
# MAP_RES = 0.02
# GRID_SIZE = int(ENV_SIZE_M / MAP_RES)
# CELL_SIZE = 1
# RESOLUTION = 50
# DT = 0.001

# # === Pygame Setup ===
# pygame.init()
# screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
# pygame.display.set_caption("Multi-Robot SLAM + A*")
# clock = pygame.time.Clock()
# half_w = WINDOW_W // 2

# # === Environment ===
# env = make_environment(grid_size=ENV_SIZE_M, resolution=RESOLUTION)

# # === Robots ===
# robots = [
#     Robot("R1", x=2, y=2, theta=0, environment=env),
#     Robot("R2", x=4, y=2, theta=np.pi, environment=env),
#     Robot("R3", x=5, y=5, theta=0, environment=env)
# ]
# robot_astar_counter = [0 for _ in robots]
# robot_colors = [(0, 255, 0), (255, 165, 0), (0, 0, 255)]
# robot_goals = [None for _ in robots]
# robot_paths = [[] for _ in robots]
# robot_autonomous = [False for _ in robots]
# selected_robot_idx = 0  # Start with R1 selected

# # === SLAM Map (shared) ===
# slam_grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)

# # === SLAM Update ===
# def update_slam(scan, pose, decay=0.999, strength=1.0, blur_sigma=2.0, threshold=0.03):
#     global slam_grid
#     slam_grid *= decay
#     angles = np.linspace(0, 2 * np.pi, num=len(scan), endpoint=False)
#     for i, d in enumerate(scan):
#         if np.isnan(d) or d <= 0: continue
#         angle = angles[i] + pose[2]
#         x = pose[0] + d * np.cos(angle)
#         y = pose[1] + d * np.sin(angle)
#         gx = int(x / MAP_RES)
#         gy = int(y / MAP_RES)
#         if 0 <= gx < slam_grid.shape[1] and 0 <= gy < slam_grid.shape[0]:
#             slam_grid[gy, gx] += strength
#             slam_grid[gy, gx] = min(slam_grid[gy, gx], 1.0)
#     blurred = gaussian_filter(slam_grid, sigma=blur_sigma)
#     return (blurred > threshold).astype(np.uint8)

# # === Utilities ===
# def pose_to_cell(pose): return int(pose[1] / MAP_RES), int(pose[0] / MAP_RES)
# def cell_to_screen(r, c): return c * CELL_SIZE, WINDOW_H - r * CELL_SIZE
# def world_to_screen(x, y): return int(x * RESOLUTION), int(WINDOW_H - y * RESOLUTION)

# def draw_grid_lines(surf, spacing, color=(180, 180, 180)):
#     w, h = surf.get_size()
#     for x in range(0, w, spacing):
#         pygame.draw.line(surf, color, (x, 0), (x, h))
#     for y in range(0, h, spacing):
#         pygame.draw.line(surf, color, (0, y), (w, y))

# def draw_world(surf, env, robots):
#     surf.fill((240, 240, 240))
#     draw_grid_lines(surf, RESOLUTION)
#     for obj in env.get_objects():
#         if obj.type == "circle":
#             cx, cy = world_to_screen(*obj.position)
#             r = int((obj.size / 2) * RESOLUTION)
#             pygame.draw.circle(surf, (180,180,180), (cx, cy), r)
#         elif obj.type == "rectangle":
#             cx, cy = obj.position
#             w, h = obj.size
#             top_left = world_to_screen(cx - w/2, cy + h/2)
#             pygame.draw.rect(surf, (180,180,180), pygame.Rect(top_left[0], top_left[1], int(w*RESOLUTION), int(h*RESOLUTION)))
#     for i, robot in enumerate(robots):
#         rx, ry = world_to_screen(robot.x, robot.y)
#         pygame.draw.circle(surf, robot_colors[i], (rx, ry), 5)
#         hx = rx + int(10 * math.cos(robot.theta))
#         hy = ry - int(10 * math.sin(robot.theta))
#         pygame.draw.line(surf, (0, 180, 0), (rx, ry), (hx, hy), 2)
#         label = pygame.font.SysFont(None, 18).render(robot.name, True, robot_colors[i])
#         surf.blit(label, (rx + 6, ry - 10))

#         if i == selected_robot_idx:
#             pygame.draw.circle(surf, (255, 255, 0), (rx, ry + 10), 3)

# def draw_slam(surf, slam_map, paths, goals):
#     surf.fill((255, 255, 255))
#     for y in range(slam_map.shape[0]):
#         for x in range(slam_map.shape[1]):
#             if slam_map[y, x]:
#                 rect = pygame.Rect(x, WINDOW_H - y, 1, 1)
#                 pygame.draw.rect(surf, (0, 0, 0), rect)
#     for i, path in enumerate(paths):
#         for r, c in path:
#             px, py = cell_to_screen(r, c)
#             pygame.draw.rect(surf, robot_colors[i], (px, py, 2, 2))
#     for i, goal in enumerate(goals):
#         if goal:
#             gx, gy = cell_to_screen(*goal)
#             pygame.draw.circle(surf, robot_colors[i], (gx, gy), 4)

# # === Main Loop ===
# running = True
# while running:
#     for e in pygame.event.get():
#         if e.type == pygame.QUIT:
#             running = False
#         elif e.type == pygame.MOUSEBUTTONDOWN:
#             if e.button == 3:  # Right click: cycle selected robot
#                 selected_robot_idx = (selected_robot_idx + 1) % len(robots)
#                 print(f"Selected robot: {robots[selected_robot_idx].name}")
#             elif e.button == 1:  # Left click: assign goal to selected robot
#                 mx, my = pygame.mouse.get_pos()
#                 if mx > half_w:
#                     gx = (mx - half_w) // CELL_SIZE
#                     gy = (WINDOW_H - my) // CELL_SIZE
#                     goal = (gy, gx)
#                     robot_goals[selected_robot_idx] = goal
#                     robot_autonomous[selected_robot_idx] = True

#     keys = pygame.key.get_pressed()
#     if not any(robot_autonomous):
#         dx = 0.05 if keys[pygame.K_d] else -0.05 if keys[pygame.K_a] else 0
#         dy = 0.05 if keys[pygame.K_w] else -0.05 if keys[pygame.K_s] else 0
#         dtheta = 0.1 if keys[pygame.K_q] else -0.1 if keys[pygame.K_e] else 0
#         robots[0].move(dx, dy, dtheta)

#     # SLAM update from each robot
#     for i, robot in enumerate(robots):
#         if robot_astar_counter[i] % 4 == 0:
#             scan = robot.update_scan()
#             pose = robot.get_pose()
#             update_slam(scan, pose)

#     astar_map = update_slam(np.zeros(360), robots[0].get_pose())  # thresholded version

#     for i, robot in enumerate(robots):
#         if robot_autonomous[i]:
#             robot_astar_counter[i] += 1
#             if robot_astar_counter[i] >= 20:
#                 start = pose_to_cell(robot.get_pose())
#                 goal = robot_goals[i]
#                 obstacle_map = astar_map.copy()
#                 for j, other in enumerate(robots):
#                     if j == i: continue
#                     r, c = pose_to_cell(other.get_pose())
#                     if 0 <= r < GRID_SIZE and 0 <= c < GRID_SIZE:
#                         obstacle_map[r, c] = 1
#                 path = a_star(obstacle_map, start, goal, landmarks_dict={})
#                 robot_astar_counter[i] = 0
#                 if path and len(path) > 1:
#                     robot_paths[i] = path
#             if robot_paths[i]:
#                 robots[i].move(
#                     robot_paths[i][0][1] * MAP_RES - robot.x,
#                     robot_paths[i][0][0] * MAP_RES - robot.y,
#                     0
#                 )
#                 robot_paths[i] = robot_paths[i][1:]
#             else:
#                 robot_autonomous[i] = False

#     screen.fill((0, 0, 0))
#     draw_world(screen.subsurface((0, 0, half_w, WINDOW_H)), env, robots)
#     draw_slam(screen.subsurface((half_w, 0, half_w, WINDOW_H)), astar_map, robot_paths, robot_goals)
#     pygame.display.flip()
#     clock.tick(int(1 / DT))

# pygame.quit()

# import pygame
# import numpy as np
# import math
# from scipy.ndimage import gaussian_filter
# from a_star_planner import a_star
# from wcbs import wcbs_plan
# from world import make_environment
# from robot import Robot

# # Constants
# WINDOW_W, WINDOW_H = 1000, 500
# ENV_SIZE_M = 5
# MAP_RES = 0.01
# GRID_SIZE = int(ENV_SIZE_M / MAP_RES)
# CELL_SIZE = 1
# RESOLUTION = 100
# DT = 0.001

# pygame.init()
# screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
# pygame.display.set_caption("WCBS Multi-Robot Sim")
# clock = pygame.time.Clock()
# half_w = WINDOW_W // 2

# env = make_environment(grid_size=ENV_SIZE_M, resolution=RESOLUTION)
# robots = [
#     Robot("R1", x=1.0, y=1.0, theta=0, environment=env),
#     Robot("R2", x=1.0, y=2.0, theta=0, environment=env)
# ]
# robot_colors = [(0, 255, 0), (255, 165, 0)]
# robot_goals = [None for _ in robots]
# robot_paths = [[] for _ in robots]
# robot_autonomous = [False for _ in robots]
# slam_grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
# selected_robot_idx = 0

# def update_slam(scan, pose, decay=0.999, strength=1.0, blur_sigma=2.0, threshold=0.03):
#     global slam_grid
#     slam_grid *= decay
#     angles = np.linspace(0, 2*np.pi, num=len(scan), endpoint=False)
#     for i, d in enumerate(scan):
#         if np.isnan(d) or d <= 0: continue
#         angle = angles[i] + pose[2]
#         x = pose[0] + d * np.cos(angle)
#         y = pose[1] + d * np.sin(angle)
#         gx = int(x / MAP_RES)
#         gy = int(y / MAP_RES)
#         if 0 <= gx < slam_grid.shape[1] and 0 <= gy < slam_grid.shape[0]:
#             slam_grid[gy, gx] += strength
#             slam_grid[gy, gx] = min(slam_grid[gy, gx], 1.0)
#     blurred = gaussian_filter(slam_grid, sigma=blur_sigma)
#     return (blurred > threshold).astype(np.uint8)

# def pose_to_cell(pose): return int(pose[1] / MAP_RES), int(pose[0] / MAP_RES)
# def cell_to_screen(r, c): return c * CELL_SIZE, WINDOW_H - r * CELL_SIZE
# def world_to_screen(x, y): return int(x * RESOLUTION), int(WINDOW_H - y * RESOLUTION)

# def draw_world(surf, env, robots):
#     surf.fill((240, 240, 240))
#     for obj in env.get_objects():
#         if obj.type == "circle":
#             cx, cy = world_to_screen(*obj.position)
#             r = int((obj.size / 2) * RESOLUTION)
#             pygame.draw.circle(surf, (180,180,180), (cx, cy), r)
#         elif obj.type == "rectangle":
#             cx, cy = obj.position
#             w, h = obj.size
#             top_left = world_to_screen(cx - w/2, cy + h/2)
#             pygame.draw.rect(surf, (180,180,180), pygame.Rect(top_left[0], top_left[1], int(w*RESOLUTION), int(h*RESOLUTION)))
#     for i, robot in enumerate(robots):
#         rx, ry = world_to_screen(robot.x, robot.y)
#         pygame.draw.circle(surf, robot_colors[i], (rx, ry), 5)
#         hx = rx + int(10 * math.cos(robot.theta))
#         hy = ry - int(10 * math.sin(robot.theta))
#         pygame.draw.line(surf, (0,180,0), (rx, ry), (hx, hy), 2)
#         if i == selected_robot_idx:
#             pygame.draw.circle(surf, (255, 255, 0), (rx, ry + 10), 3)

# def draw_slam(surf, slam_map, paths, goals):
#     surf.fill((255, 255, 255))
#     for y in range(slam_map.shape[0]):
#         for x in range(slam_map.shape[1]):
#             if slam_map[y, x]:
#                 rect = pygame.Rect(x, WINDOW_H - y, 1, 1)
#                 pygame.draw.rect(surf, (0, 0, 0), rect)
#     for i, path in enumerate(paths):
#         for r, c in path:
#             px, py = cell_to_screen(r, c)
#             pygame.draw.rect(surf, robot_colors[i], (px, py, 2, 2))
#     for i, goal in enumerate(goals):
#         if goal:
#             gx, gy = cell_to_screen(*goal)
#             pygame.draw.circle(surf, robot_colors[i], (gx, gy), 4)

# running = True
# while running:
#     for e in pygame.event.get():
#         if e.type == pygame.QUIT:
#             running = False
#         elif e.type == pygame.MOUSEBUTTONDOWN:
#             if e.button == 3:
#                 selected_robot_idx = (selected_robot_idx + 1) % len(robots)
#             elif e.button == 1:
#                 mx, my = pygame.mouse.get_pos()
#                 if mx > half_w:
#                     gx = (mx - half_w) // CELL_SIZE
#                     gy = (WINDOW_H - my) // CELL_SIZE
#                     robot_goals[selected_robot_idx] = (gy, gx)
#                     robot_autonomous[selected_robot_idx] = True

#     for i, robot in enumerate(robots):
#         scan = robot.update_scan()
#         pose = robot.get_pose()
#         update_slam(scan, pose)

#     astar_map = update_slam(np.zeros(360), robots[0].get_pose())

#     if any(robot_autonomous):
#         starts = [pose_to_cell(r.get_pose()) for r in robots]
#         paths = wcbs_plan(starts, robot_goals, astar_map, a_star)
#         for i in range(len(robots)):
#             robot_paths[i] = paths[i]
#             robot_autonomous[i] = bool(paths[i])

#     for i, robot in enumerate(robots):
#         if robot_paths[i]:
#             target = robot_paths[i].pop(0)
#             robot.x = target[1] * MAP_RES
#             robot.y = target[0] * MAP_RES
#             if not robot_paths[i]:
#                 robot_autonomous[i] = False

#     screen.fill((0, 0, 0))
#     draw_world(screen.subsurface((0, 0, half_w, WINDOW_H)), env, robots)
#     draw_slam(screen.subsurface((half_w, 0, half_w, WINDOW_H)), astar_map, robot_paths, robot_goals)
#     pygame.display.flip()
#     clock.tick(int(1 / DT))

# pygame.quit()
import pygame
import numpy as np
import math
import os
from scipy.ndimage import gaussian_filter
from a_star_planner import a_star, build_reservations
from wcbs import wcbs_plan
from world import make_environment
from robot import Robot

# === Constants ===
WINDOW_W, WINDOW_H = 1000, 500
ENV_SIZE_M = 5
MAP_RES = 0.01
GRID_SIZE = int(ENV_SIZE_M / MAP_RES)
CELL_SIZE = 1
RESOLUTION = 100
DT = 0.001

# === Frame saving directory ===
FRAME_DIR = "pygame_frames"
os.makedirs(FRAME_DIR, exist_ok=True)

# === Pygame Setup ===
pygame.init()
screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
pygame.display.set_caption("Multi-Robot SLAM + WCBS")
clock = pygame.time.Clock()
half_w = WINDOW_W // 2

# === Environment ===
env = make_environment(grid_size=ENV_SIZE_M, resolution=RESOLUTION)

# === Robots ===
robots = [
    Robot("R1", x=2, y=2, theta=0, environment=env),
    Robot("R2", x=2, y=3, theta=np.pi, environment=env),
    Robot("R3", x=3, y=3, theta=np.pi, environment=env),
    Robot("R4", x=3, y=4, theta=np.pi, environment=env),
    Robot("R5", x=4, y=3, theta=np.pi, environment=env),
    Robot("R6", x=2.5, y=2.5, theta=np.pi, environment=env)
]
robot_colors = [(0, 255, 0), (255, 165, 0), (0, 0, 255), (255, 0, 255), (0, 255, 255), (255, 0, 0)]
robot_goals = [None for _ in robots]
robot_paths = [[] for _ in robots]
robot_autonomous = [False for _ in robots]
selected_robot_idx = 0

# === SLAM Map (shared) ===
slam_grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)

def update_slam(scan, pose, decay=0.999, strength=1.0, blur_sigma=2.0, threshold=0.03):
    global slam_grid
    slam_grid *= decay
    angles = np.linspace(0, 2 * np.pi, num=len(scan), endpoint=False)
    for i, d in enumerate(scan):
        if np.isnan(d) or d <= 0: continue
        angle = angles[i] + pose[2]
        x = pose[0] + d * np.cos(angle)
        y = pose[1] + d * np.sin(angle)
        gx = int(x / MAP_RES)
        gy = int(y / MAP_RES)
        if 0 <= gx < slam_grid.shape[1] and 0 <= gy < slam_grid.shape[0]:
            slam_grid[gy, gx] += strength
            slam_grid[gy, gx] = min(slam_grid[gy, gx], 1.0)
    blurred = gaussian_filter(slam_grid, sigma=blur_sigma)
    return (blurred > threshold).astype(np.uint8)

def pose_to_cell(pose): return int(pose[1] / MAP_RES), int(pose[0] / MAP_RES)
def cell_to_screen(r, c): return c * CELL_SIZE, WINDOW_H - r * CELL_SIZE
def world_to_screen(x, y): return int(x * RESOLUTION), int(WINDOW_H - y * RESOLUTION)

def draw_grid_lines(surf, spacing, color=(180, 180, 180)):
    w, h = surf.get_size()
    for x in range(0, w, spacing):
        pygame.draw.line(surf, color, (x, 0), (x, h))
    for y in range(0, h, spacing):
        pygame.draw.line(surf, color, (0, y), (w, y))

def draw_world(surf, env, robots):
    surf.fill((240, 240, 240))
    draw_grid_lines(surf, RESOLUTION)
    for obj in env.get_objects():
        if obj.type == "circle":
            cx, cy = world_to_screen(*obj.position)
            r = int((obj.size / 2) * RESOLUTION)
            pygame.draw.circle(surf, (180,180,180), (cx, cy), r)
        elif obj.type == "rectangle":
            cx, cy = obj.position
            w, h = obj.size
            top_left = world_to_screen(cx - w/2, cy + h/2)
            pygame.draw.rect(surf, (180,180,180), pygame.Rect(top_left[0], top_left[1], int(w*RESOLUTION), int(h*RESOLUTION)))
    for i, robot in enumerate(robots):
        rx, ry = world_to_screen(robot.x, robot.y)
        pygame.draw.circle(surf, robot_colors[i], (rx, ry), 5)
        hx = rx + int(10 * math.cos(robot.theta))
        hy = ry - int(10 * math.sin(robot.theta))
        pygame.draw.line(surf, (0, 180, 0), (rx, ry), (hx, hy), 2)
        label = pygame.font.SysFont(None, 18).render(robot.name, True, robot_colors[i])
        surf.blit(label, (rx + 6, ry - 10))
        if i == selected_robot_idx:
            pygame.draw.circle(surf, (255, 255, 0), (rx, ry + 10), 3)

def draw_slam(surf, slam_map, paths, goals):
    surf.fill((255, 255, 255))
    for y in range(slam_map.shape[0]):
        for x in range(slam_map.shape[1]):
            if slam_map[y, x]:
                rect = pygame.Rect(x, WINDOW_H - y, 1, 1)
                pygame.draw.rect(surf, (0, 0, 0), rect)
    for i, path in enumerate(paths):
        for r, c in path:
            px, py = cell_to_screen(r, c)
            pygame.draw.rect(surf, robot_colors[i], (px, py, 2, 2))
    for i, goal in enumerate(goals):
        if goal:
            gx, gy = cell_to_screen(*goal)
            pygame.draw.circle(surf, robot_colors[i], (gx, gy), 4)

# === Main Loop ===
running = True
step_counter = 0
while running:
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            running = False
        elif e.type == pygame.MOUSEBUTTONDOWN:
            if e.button == 3:
                selected_robot_idx = (selected_robot_idx + 1) % len(robots)
                print(f"Selected robot: {robots[selected_robot_idx].name}")
            elif e.button == 1:
                mx, my = pygame.mouse.get_pos()
                if mx > half_w:
                    gx = (mx - half_w) // CELL_SIZE
                    gy = (WINDOW_H - my) // CELL_SIZE
                    goal = (gy, gx)
                    robot_goals[selected_robot_idx] = goal
                    robot_autonomous[selected_robot_idx] = True

    keys = pygame.key.get_pressed()
    if not any(robot_autonomous):
        dx = 0.05 if keys[pygame.K_d] else -0.05 if keys[pygame.K_a] else 0
        dy = 0.05 if keys[pygame.K_w] else -0.05 if keys[pygame.K_s] else 0
        dtheta = 0.1 if keys[pygame.K_q] else -0.1 if keys[pygame.K_e] else 0
        robots[0].move(dx, dy, dtheta)

    for robot in robots:
        scan = robot.update_scan()
        update_slam(scan, robot.get_pose())
    astar_map = update_slam(np.zeros(360), robots[0].get_pose())

    if step_counter % 10 == 0:
        starts = [pose_to_cell(r.get_pose()) for r in robots]
        if all(g is not None for g in robot_goals):
            robot_paths = wcbs_plan(starts, robot_goals, astar_map, a_star)

    for i, robot in enumerate(robots):
        if robot_paths and robot_paths[i]:
            next_cell = robot_paths[i][0]
            goal_x, goal_y = next_cell[1] * MAP_RES, next_cell[0] * MAP_RES
            robot.move(goal_x - robot.x, goal_y - robot.y, 0)
            robot_paths[i] = robot_paths[i][1:]

    screen.fill((0, 0, 0))
    draw_world(screen.subsurface((0, 0, half_w, WINDOW_H)), env, robots)
    draw_slam(screen.subsurface((half_w, 0, half_w, WINDOW_H)), astar_map, robot_paths, robot_goals)
    pygame.display.flip()

    # === Save frame ===
    frame_path = os.path.join(FRAME_DIR, f"frame_{step_counter:04d}.png")
    pygame.image.save(screen, frame_path)

    clock.tick(int(1 / DT))
    step_counter += 1

pygame.quit()
