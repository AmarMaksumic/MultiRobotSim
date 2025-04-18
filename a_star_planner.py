# import numpy as np
# import matplotlib.pyplot as plt
# import heapq
# from scipy.ndimage import binary_dilation

# def load_map(filename="slam_map.npy", inflate_radius=2):
#     grid = np.load(filename)
#     if inflate_radius > 0:
#         structure = np.ones((inflate_radius * 2 + 1, inflate_radius * 2 + 1))
#         grid = binary_dilation(grid, structure=structure).astype(np.uint8)
#     return grid

# # === Define landmarks by name and cell coordinate ===
# landmarks = {
#     "charging": (50, 50),
#     "goal_A": (450, 450),
#     "goal_B": (320, 100)
# }

# # === Landmark-aware heuristic ===
# def landmark_heuristic(a, b, landmark_list):
#     a = np.array(a)
#     b = np.array(b)
#     min_cost = np.linalg.norm(a - b)  # fallback direct
#     for l in landmark_list:
#         l = np.array(l)
#         cost = np.linalg.norm(a - l) + np.linalg.norm(l - b)
#         min_cost = min(min_cost, cost)
#     return min_cost

# def a_star(grid, start, goal, landmarks_dict):
#     h, w = grid.shape
#     open_set = []
#     heapq.heappush(open_set, (0 + landmark_heuristic(start, goal, list(landmarks_dict.values())), 0, start))
#     came_from = {}
#     cost_so_far = {start: 0}
#     neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]

#     while open_set:
#         _, cost, current = heapq.heappop(open_set)

#         if current == goal:
#             path = [current]
#             while current in came_from:
#                 current = came_from[current]
#                 path.append(current)
#             path.reverse()
#             return path

#         for dx, dy in neighbors:
#             nx, ny = current[0] + dx, current[1] + dy
#             if 0 <= nx < h and 0 <= ny < w and grid[nx, ny] == 0:
#                 next_node = (nx, ny)
#                 new_cost = cost + 1
#                 if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
#                     cost_so_far[next_node] = new_cost
#                     priority = new_cost + landmark_heuristic(next_node, goal, list(landmarks_dict.values()))
#                     heapq.heappush(open_set, (priority, new_cost, next_node))
#                     came_from[next_node] = current
#     return None

# if __name__ == "__main__":
#   # === Plan between two named landmarks ===
#   start_name = "charging"
#   goal_name = "goal_A"

#   slam_grid = load_map("slam_map.npy", inflate_radius=2)
#   start = landmarks[start_name]
#   goal = landmarks[goal_name]

#   path = a_star(slam_grid, start, goal, landmarks)

#   # === Visualization ===
#   plt.imshow(slam_grid, cmap='gray', origin='lower')
#   if path:
#       px, py = zip(*path)
#       plt.plot(py, px, color='cyan', linewidth=2, label="Planned Path")
#   plt.scatter([start[1]], [start[0]], c='green', s=40, label=f"Start: {start_name}")
#   plt.scatter([goal[1]], [goal[0]], c='red', s=40, label=f"Goal: {goal_name}")
#   plt.title("Landmark-Aware A* Path Planning")
#   plt.legend()
#   plt.show()


import heapq

def heuristic(a, b):
    if a is None or b is None:
        return float('inf')
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(grid, start, goal, reservations=None, max_time=200):
    rows, cols = grid.shape
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), 0, start, [start]))
    visited = set()

    while open_set:
        f, t, current, path = heapq.heappop(open_set)
        if (current, t) in visited:
            continue
        visited.add((current, t))

        if current == goal:
            return path

        neighbors = [(0,1), (1,0), (0,-1), (-1,0)]
        for dx, dy in neighbors:
            nr, nc = current[0] + dx, current[1] + dy
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr, nc] == 0:
                next_cell = (nr, nc)
                next_time = t + 1

                # Reservation checking
                if reservations and reservations.get((nr, nc), -1) >= next_time:
                    continue

                heapq.heappush(open_set, (
                    next_time + heuristic(next_cell, goal),
                    next_time,
                    next_cell,
                    path + [next_cell]
                ))

        # Option to wait in place
        if reservations and reservations.get(current, -1) < t + 1:
            heapq.heappush(open_set, (
                t + 1 + heuristic(current, goal),
                t + 1,
                current,
                path + [current]
            ))

        if t > max_time:
            break

    print(f"A* failed: {start} → {goal}")
    return []

def build_reservations(paths, skip_index=None, buffer_time=5, buffer_radius=10, grid_shape=None):
    """
    Builds a time-space reservation table from other robots' paths.
    
    Args:
        paths: List of paths, each a list of (row, col) tuples.
        skip_index: Robot index to exclude from reservation (the current one planning).
        buffer_time: How many timesteps before and after to reserve.
        buffer_radius: How many cells in each direction to reserve around each occupied cell.
        grid_shape: Tuple (rows, cols) for bounds checking. Optional.
    
    Returns:
        Dict mapping (row, col) to latest reserved timestep.
    """
    reservations = {}
    rows, cols = grid_shape if grid_shape else (float('inf'), float('inf'))

    for i, path in enumerate(paths):
        if i == skip_index or not path:
            continue

        for t, (r, c) in enumerate(path):
            for dt in range(-buffer_time, buffer_time + 1):
                time_step = max(0, t + dt)  # clamp to t >= 0

                for dx in range(-buffer_radius, buffer_radius + 1):
                    for dy in range(-buffer_radius, buffer_radius + 1):
                        nr, nc = r + dy, c + dx

                        if 0 <= nr < rows and 0 <= nc < cols:
                            key = (nr, nc)
                            reservations[key] = max(reservations.get(key, -1), time_step)

    return reservations
