import json
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter

def load_log(file_path="log.json"):
    with open(file_path, "r") as f:
        return json.load(f)

def scan_to_pointcloud(scan, resolution_deg):
    angles = np.radians(np.arange(0, 360, resolution_deg))
    scan = np.array(scan)
    valid = ~np.isnan(scan)
    angles = angles[valid]
    scan = scan[valid]
    xs = np.cos(angles) * scan
    ys = np.sin(angles) * scan
    points = np.vstack([xs, ys]).T
    return points

def pose_to_transform(x, y, theta):
    T = np.identity(3)
    T[0:2, 0] = [np.cos(theta), -np.sin(theta)]
    T[0:2, 1] = [np.sin(theta),  np.cos(theta)]
    T[0:2, 2] = [x, y]
    return T

def build_occupancy_grid(log, resolution=0.02, grid_size=10.0):
    resolution_deg = 360 // len(log[0]["scan"])
    points_world = []

    for entry in log:
        scan = entry["scan"]
        pose = entry["pose"]
        points = scan_to_pointcloud(scan, resolution_deg)
        T = pose_to_transform(*pose)
        points_hom = np.hstack([points, np.ones((points.shape[0], 1))])
        transformed = (T @ points_hom.T).T[:, :2]
        points_world.append(transformed)

    all_points = np.vstack(points_world)

    # Shift all points so min is at (0, 0)
    min_coords = np.min(all_points, axis=0)
    shifted_points = all_points - min_coords
    grid_dim = int(grid_size / resolution)
    grid = np.zeros((grid_dim, grid_dim), dtype=np.float32)

    for x, y in shifted_points:
        gx = int(np.clip(x / resolution, 0, grid_dim - 1))
        gy = int(np.clip(y / resolution, 0, grid_dim - 1))
        grid[gy, gx] = 1.0

    # Apply Gaussian blur to smooth sparse occupancy hits
    blurred = gaussian_filter(grid, sigma=1.0)
    binary_grid = (blurred > 0.2).astype(np.uint8)

    return binary_grid, shifted_points, resolution

# Main
log = load_log()
grid, shifted_points, resolution = build_occupancy_grid(log, resolution=0.02, grid_size=10.0)
np.save("occupancy_grid.npy", grid)

# Visualization: grid + properly shifted point overlay
plt.imshow(grid, cmap="gray", origin="lower")
plt.scatter(shifted_points[:, 0] / resolution, shifted_points[:, 1] / resolution, s=0.1, c='red')
plt.title("Smoothed Occupancy Grid with Point Overlay (0.02m resolution)")
plt.xlabel("X cells")
plt.ylabel("Y cells")
plt.show()
