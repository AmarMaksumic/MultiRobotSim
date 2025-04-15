import json
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
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

def transform_pointcloud(points, pose):
    x, y, theta = pose
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    return (R @ points.T).T + np.array([x, y])

def icp_pose_estimate(source_points, target_pcd, init_pose):
    source = o3d.geometry.PointCloud()
    source_points_3d = np.hstack([source_points, np.zeros((source_points.shape[0], 1))])
    source.points = o3d.utility.Vector3dVector(source_points_3d)
    trans_init = pose_to_matrix(init_pose)
    result = o3d.pipelines.registration.registration_icp(
        source, target_pcd, max_correspondence_distance=0.3,
        init=trans_init,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    return matrix_to_pose(result.transformation), result

def pose_to_matrix(pose):
    x, y, theta = pose
    c, s = np.cos(theta), np.sin(theta)
    T = np.eye(4)
    T[:2, :2] = [[c, -s], [s, c]]
    T[0, 3] = x
    T[1, 3] = y
    return T

def matrix_to_pose(T):
    x, y = T[0, 3], T[1, 3]
    theta = np.arctan2(T[1, 0], T[0, 0])
    return [x, y, theta]

def build_icp_slam_map(log, resolution=0.02):
    resolution_deg = 360 // len(log[0]["scan"])
    global_points = []
    global_pcd = o3d.geometry.PointCloud()
    poses = []

    for i, entry in enumerate(log):
        scan = entry["scan"]
        init_pose = entry["pose"]
        scan_pts = scan_to_pointcloud(scan, resolution_deg)
        transformed_pts = transform_pointcloud(scan_pts, init_pose)
        transformed_pts_3d = np.hstack([transformed_pts, np.zeros((transformed_pts.shape[0], 1))])

        if i == 0:
            poses.append(init_pose)
            global_points.append(transformed_pts)
            global_pcd.points = o3d.utility.Vector3dVector(transformed_pts_3d)
            continue

        est_pose, _ = icp_pose_estimate(scan_pts, global_pcd, init_pose)
        refined_pts = transform_pointcloud(scan_pts, est_pose)
        refined_pts_3d = np.hstack([refined_pts, np.zeros((refined_pts.shape[0], 1))])
        poses.append(est_pose)
        global_points.append(refined_pts)

        current = o3d.geometry.PointCloud()
        current.points = o3d.utility.Vector3dVector(refined_pts_3d)
        global_pcd += current

    return np.vstack(global_points)

def rasterize_pointcloud(points, resolution=0.02, map_size_m=10.0):
    grid_dim = (int(map_size_m / resolution), int(map_size_m / resolution))
    grid = np.zeros(grid_dim[::-1], dtype=np.float32)

    for x, y in points:
        gx = int(x / resolution)
        gy = int(y / resolution)
        if 0 <= gx < grid_dim[0] and 0 <= gy < grid_dim[1]:
            grid[gy, gx] = 1.0

    blurred = gaussian_filter(grid, sigma=1.0)
    binary_grid = (blurred > 0.2).astype(np.uint8)
    return binary_grid, points

if __name__ == "__main__":
    # Run SLAM pipeline
    resolution = 0.02
    log = load_log()
    pointcloud = build_icp_slam_map(log, resolution=resolution)
    grid, world_points = rasterize_pointcloud(pointcloud, resolution=resolution, map_size_m=10.0)
    np.save("slam_map.npy", grid)

    # Visualization
    plt.imshow(grid, cmap="gray", origin="lower")
    px = np.clip((world_points[:, 0] / resolution).astype(int), 0, grid.shape[1] - 1)
    py = np.clip((world_points[:, 1] / resolution).astype(int), 0, grid.shape[0] - 1)
    plt.scatter(px, py, s=0.1, c='red')
    plt.title("SLAM Map (ICP Aligned) in Real-World Coordinates")
    plt.xlabel("X cells")
    plt.ylabel("Y cells")
    plt.show()

