import numpy as np
import matplotlib.pyplot as plt
import heapq
from scipy.ndimage import binary_dilation

def load_map(filename="slam_map.npy", inflate_radius=2):
    grid = np.load(filename)
    if inflate_radius > 0:
        structure = np.ones((inflate_radius * 2 + 1, inflate_radius * 2 + 1))
        grid = binary_dilation(grid, structure=structure).astype(np.uint8)
    return grid

# === Define landmarks by name and cell coordinate ===
landmarks = {
    "charging": (50, 50),
    "goal_A": (450, 450),
    "goal_B": (320, 100)
}

# === Landmark-aware heuristic ===
def landmark_heuristic(a, b, landmark_list):
    a = np.array(a)
    b = np.array(b)
    min_cost = np.linalg.norm(a - b)  # fallback direct
    for l in landmark_list:
        l = np.array(l)
        cost = np.linalg.norm(a - l) + np.linalg.norm(l - b)
        min_cost = min(min_cost, cost)
    return min_cost

def a_star(grid, start, goal, landmarks_dict):
    h, w = grid.shape
    open_set = []
    heapq.heappush(open_set, (0 + landmark_heuristic(start, goal, list(landmarks_dict.values())), 0, start))
    came_from = {}
    cost_so_far = {start: 0}
    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    while open_set:
        _, cost, current = heapq.heappop(open_set)

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        for dx, dy in neighbors:
            nx, ny = current[0] + dx, current[1] + dy
            if 0 <= nx < h and 0 <= ny < w and grid[nx, ny] == 0:
                next_node = (nx, ny)
                new_cost = cost + 1
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + landmark_heuristic(next_node, goal, list(landmarks_dict.values()))
                    heapq.heappush(open_set, (priority, new_cost, next_node))
                    came_from[next_node] = current
    return None

if __name__ == "__main__":
  # === Plan between two named landmarks ===
  start_name = "charging"
  goal_name = "goal_A"

  slam_grid = load_map("slam_map.npy", inflate_radius=2)
  start = landmarks[start_name]
  goal = landmarks[goal_name]

  path = a_star(slam_grid, start, goal, landmarks)

  # === Visualization ===
  plt.imshow(slam_grid, cmap='gray', origin='lower')
  if path:
      px, py = zip(*path)
      plt.plot(py, px, color='cyan', linewidth=2, label="Planned Path")
  plt.scatter([start[1]], [start[0]], c='green', s=40, label=f"Start: {start_name}")
  plt.scatter([goal[1]], [goal[0]], c='red', s=40, label=f"Goal: {goal_name}")
  plt.title("Landmark-Aware A* Path Planning")
  plt.legend()
  plt.show()
