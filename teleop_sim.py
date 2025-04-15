# import pygame
# import numpy as np
# import json
# from environment import Environment, Object
# from robot import Robot
# from world import make_environment

# # Constants
# WINDOW_SIZE = 1000
# ENV_SIZE_M = 10
# RESOLUTION = 100  # pixels per meter
# DT = 0.1  # simulation time step (s)
# LINEAR_VEL = 1.0  # m/s
# ANGULAR_VEL = np.radians(90)  # rad/s

# def world_to_screen(x, y):
#     return int(x * RESOLUTION), int(WINDOW_SIZE - y * RESOLUTION)

# def draw_environment(screen, env):
#     for obj in env.get_objects():
#         if obj.type == "circle":
#             cx, cy = world_to_screen(*obj.position)
#             radius = int((obj.size / 2) * RESOLUTION)
#             pygame.draw.circle(screen, (200, 200, 200), (cx, cy), radius)
#         elif obj.type == "rectangle":
#             cx, cy = obj.position
#             w, h = obj.size

#             # Convert top-left corner to screen coordinates
#             rect_x = cx - w / 2
#             rect_y = cy + h / 2
#             top_left = world_to_screen(rect_x, rect_y)

#             rect = pygame.Rect(
#                 top_left[0],
#                 top_left[1],
#                 int(w * RESOLUTION),
#                 int(h * RESOLUTION)
#             )
#             pygame.draw.rect(screen, (200, 200, 200), rect)

# def draw_grid(screen, grid_size, resolution, color=(50, 50, 50)):
#     for i in range(grid_size + 1):
#         x = i * resolution
#         pygame.draw.line(screen, color, (x, 0), (x, WINDOW_SIZE))  # vertical lines
#         pygame.draw.line(screen, color, (0, x), (WINDOW_SIZE, x))  # horizontal lines


# def main():
#     pygame.init()
#     screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
#     pygame.display.set_caption("LiDAR Robot Teleop")
#     clock = pygame.time.Clock()

#     # Initialize environment and robot
#     env = make_environment(grid_size=ENV_SIZE_M, resolution=RESOLUTION)
#     robot = Robot("R1", x=5, y=5, theta=np.radians(0), environment=env)

#     log = []
#     running = True
#     while running:
#         screen.fill((30, 30, 30))

#         keys = pygame.key.get_pressed()
#         lv, av = 0.0, 0.0

#         if keys[pygame.K_w]: lv += LINEAR_VEL
#         if keys[pygame.K_s]: lv -= LINEAR_VEL
#         if keys[pygame.K_q]: av += ANGULAR_VEL
#         if keys[pygame.K_e]: av -= ANGULAR_VEL

#         robot.move(lv, av, DT)
#         scan = robot.update_scan()
#         pose = robot.get_pose()
#         hits = robot.get_lidar_hits()

#         # --- Drawing ---
#         draw_grid(screen, ENV_SIZE_M, RESOLUTION)
#         draw_environment(screen, env)

#         # Draw robot
#         rx, ry = world_to_screen(robot.x, robot.y)
#         pygame.draw.circle(screen, (0, 255, 0), (rx, ry), 5)

#         # Draw heading
#         hx = rx + int(15 * np.cos(robot.theta))
#         hy = ry - int(15 * np.sin(robot.theta))
#         pygame.draw.line(screen, (0, 255, 255), (rx, ry), (hx, hy), 2)

#         # Draw LiDAR hits
#         for (x, y) in hits:
#             sx, sy = world_to_screen(x, y)
#             pygame.draw.circle(screen, (255, 0, 0), (sx, sy), 2)

#         # --- Logging ---
#         log.append({
#             "pose": [robot.x, robot.y, robot.theta],
#             "scan": scan,
#             "lidar_rotation": robot.lidar.rotation
#         })

#         pygame.display.flip()
#         clock.tick(int(1 / DT))

#         for event in pygame.event.get():
#             if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_x):
#                 running = False

#     # Save log
#     with open("log.json", "w") as f:
#         json.dump(log, f, indent=2)
#     print("Log saved to log.json")

#     pygame.quit()

# if __name__ == "__main__":
#     main()



import pygame
import numpy as np
import math
import json
from a_star_planner import a_star, load_map
from world import make_environment

# === Setup ===
pygame.init()
win_w, win_h = 1000, 500
cell_size = 1
screen = pygame.display.set_mode((win_w, win_h))
pygame.display.set_caption("Teleop World + SLAM Map Autonav")
clock = pygame.time.Clock()

# === SLAM Map ===
slam_map = load_map("slam_map.npy", inflate_radius=5)
map_h, map_w = slam_map.shape
map_res = 0.02
half_w = win_w // 2

# === Environment ===
ENV_SIZE_M = 10
RESOLUTION = 50
env = make_environment(grid_size=ENV_SIZE_M, resolution=RESOLUTION)

# === Robot Pose ===
with open("log.json", "r") as f:
    pose_data = json.load(f)
pose = pose_data[-1]["pose"]
path = []
goal = None
autonomous = False
robot_path_history = []

# === Utilities ===
def pose_to_cell(x, y):
    return int(y / map_res), int(x / map_res)

def cell_to_screen(row, col, offset=0, flip_y=False):
    sx = col * cell_size + offset
    sy = row * cell_size
    if flip_y:
        sy = win_h - sy
    return sx, sy

def world_to_screen(x, y):
    return int(x * RESOLUTION), int(win_h - y * RESOLUTION)

def draw_environment(screen, env):
    for obj in env.get_objects():
        if obj.type == "circle":
            cx, cy = world_to_screen(*obj.position)
            radius = int((obj.size / 2) * RESOLUTION)
            pygame.draw.circle(screen, (180, 180, 180), (cx, cy), radius)
        elif obj.type == "rectangle":
            cx, cy = obj.position
            w, h = obj.size
            top_left = world_to_screen(cx - w / 2, cy + h / 2)
            rect = pygame.Rect(
                top_left[0], top_left[1],
                int(w * RESOLUTION), int(h * RESOLUTION)
            )
            pygame.draw.rect(screen, (180, 180, 180), rect)

def follow_path(pose, path):
    if not path:
        return pose, path

    target_row, target_col = path[0]
    target_x = target_col * map_res
    target_y = target_row * map_res

    dx = target_x - pose[0]
    dy = target_y - pose[1]
    dist = np.hypot(dx, dy)

    if dist < 0.05:
        path.pop(0)
    else:
        step_size = min(0.05, dist)
        dir_x = dx / dist
        dir_y = dy / dist
        pose[0] += step_size * dir_x
        pose[1] += step_size * dir_y
        pose[2] = math.atan2(dy, dx)

    return pose, path

# === Drawing ===
def draw_grid_lines(surface, spacing, offset_x=0, offset_y=0, color=(100, 100, 100)):
    width, height = surface.get_size()
    for x in range(0, width, spacing):
        pygame.draw.line(surface, color, (x, 0), (x, height))
    for y in range(0, height, spacing):
        pygame.draw.line(surface, color, (0, y), (width, y))

# Updated draw_world function
def draw_world(surf, pose):
    surf.fill((230, 230, 230))
    draw_grid_lines(surf, RESOLUTION)
    draw_environment(surf, env)
    for hist_pose in robot_path_history:
        px, py = world_to_screen(hist_pose[0], hist_pose[1])
        pygame.draw.circle(surf, (173, 216, 230), (px, py), 2)
    rx, ry = world_to_screen(pose[0], pose[1])
    pygame.draw.circle(surf, (0, 255, 0), (rx, ry), 6)
    hx = rx + int(10 * math.cos(pose[2]))
    hy = ry - int(10 * math.sin(pose[2]))
    pygame.draw.line(surf, (0, 180, 0), (rx, ry), (hx, hy), 2)

# Updated draw_slam function
def draw_slam(surf, slam_map, path=None, goal=None):
    surf.fill((255, 255, 255))

    for y in range(map_h):
        for x in range(map_w):
            val = slam_map[y, x]
            color = (0, 0, 0) if val else (255, 255, 255)
            rect = pygame.Rect(x * cell_size, win_h - y * cell_size, cell_size, cell_size)
            pygame.draw.rect(surf, color, rect)

    if path:
        for p in path:
            px, py = cell_to_screen(*p, offset=half_w, flip_y=True)
            pygame.draw.rect(surf, (0, 255, 255), (px, py, cell_size, cell_size))

    if goal:
        gx, gy = cell_to_screen(*goal, offset=half_w, flip_y=True)
        pygame.draw.circle(surf, (255, 0, 0), (gx, gy), 5)

    rx, ry = pose_to_cell(pose[0], pose[1])
    sx, sy = cell_to_screen(rx, ry, offset=half_w, flip_y=True)
    pygame.draw.circle(surf, (0, 255, 0), (sx, sy), 4)
    hx = sx + int(10 * math.cos(pose[2]))
    hy = sy - int(10 * math.sin(pose[2]))
    pygame.draw.line(surf, (0, 180, 0), (sx, sy), (hx, hy), 2)
    draw_grid_lines(surf, spacing=int(1 / map_res), color=(200, 200, 200))

# === Main loop ===
running = True
while running:
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            running = False
        elif e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
            mx, my = pygame.mouse.get_pos()
            if mx >= half_w:
                gx = (mx - half_w) // cell_size
                gy = (win_h - my) // cell_size
                goal = (gy, gx)
                start = pose_to_cell(pose[0], pose[1])
                path = a_star(slam_map, start, goal, landmarks_dict={})
                autonomous = bool(path)
                print(f"Planned path to {goal} from {start}")

    keys = pygame.key.get_pressed()
    if not autonomous:
        if keys[pygame.K_w]:
            pose[1] += 0.05
        if keys[pygame.K_s]:
            pose[1] -= 0.05
        if keys[pygame.K_a]:
            pose[0] -= 0.05
        if keys[pygame.K_d]:
            pose[0] += 0.05
    else:
        pose, path = follow_path(pose, path)
        if not path:
            autonomous = False
            print("Reached goal.")

    robot_path_history.append((pose[0], pose[1]))

    screen.fill((0, 0, 0))
    draw_world(screen.subsurface((0, 0, half_w, win_h)), pose)
    draw_slam(screen.subsurface((half_w, 0, half_w, win_h)), slam_map, path, goal)
    pygame.display.flip()
    clock.tick(30)

pygame.quit()
