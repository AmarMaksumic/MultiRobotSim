import json
import numpy as np
import matplotlib.pyplot as plt

def load_log(file_path="log.json"):
    with open(file_path, "r") as f:
        return json.load(f)

def scan_to_points(scan, resolution_deg):
    """Convert polar scan to Cartesian coordinates in robot frame."""
    angles = np.radians(np.arange(0, 360, resolution_deg))
    xs = np.cos(angles) * scan
    ys = np.sin(angles) * scan
    return np.vstack([xs, ys]).T  # Shape (N, 2)

def transform_points(points, pose):
    """Apply 2D rigid body transform to points from robot to world frame."""
    x, y, theta = pose
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta),  np.cos(theta)]])
    transformed = (R @ points.T).T + np.array([x, y])
    return transformed

def visualize_map(points):
    """Plot the accumulated point cloud."""
    xs, ys = points[:, 0], points[:, 1]
    plt.figure(figsize=(8, 8))
    plt.scatter(xs, ys, s=1, color='black')
    plt.title("Reconstructed 2D Map from LiDAR Scans")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.axis("equal")
    plt.grid(True)
    plt.show()

def main():
    log = load_log("log.json")
    all_points = []

    resolution = 360 // len(log[0]["scan"])  # infer from scan size

    for frame in log:
        pose = frame["pose"]  # [x, y, theta]
        scan = np.array(frame["scan"])
        local_points = scan_to_points(scan, resolution)
        world_points = transform_points(local_points, pose)
        all_points.append(world_points)

    all_points = np.vstack(all_points)
    visualize_map(all_points)

if __name__ == "__main__":
    main()
