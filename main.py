import numpy as np
import matplotlib.pyplot as plt
from environment import Environment, Object
from lidar import LIDAR, plot_scan_polar
from world import make_environment

def plot_lidar_heading(ax, lidar, scale=0.5, color='blue'):
    """Draw an arrow showing the LIDAR's heading direction."""
    x, y = lidar.translation
    theta_rad = np.radians(lidar.rotation)

    dx = np.cos(theta_rad) * scale
    dy = np.sin(theta_rad) * scale

    ax.arrow(x * lidar.environment.resolution,
             y * lidar.environment.resolution,
             dx * lidar.environment.resolution,
             dy * lidar.environment.resolution,
             head_width=5, head_length=5,
             fc=color, ec=color, linewidth=2)

def main():
    # Create a 10x10 meter environment with 100 grid cells per meter (1cm resolution)
    env = make_environment(grid_size=10, resolution=100)

    # Create and configure LIDAR
    lidar = LIDAR(name="Lidar1", resolution=5, range=10.0, environment=env)
    lidar.set_location(5, 5)
    lidar.set_rotation(45)  # Facing west

    # Perform scan
    scan_data = lidar.scan()
    hit_points = lidar.get_last_hit_points()

    # Create subplots: one 2D map, one polar scan
    fig = plt.figure(figsize=(12, 6))
    ax_env = fig.add_subplot(1, 2, 1)
    ax_polar = fig.add_subplot(1, 2, 2, projection='polar')

    # Plot environment
    env.display_environment(ax=ax_env, show=False, title="Environment Grid")

    # Overlay LiDAR hit points
    for (x, y) in hit_points:
        gx = int(x * env.resolution)
        gy = int(y * env.resolution)
        if 0 <= gx < env.grid.shape[0] and 0 <= gy < env.grid.shape[1]:
            ax_env.plot(gx, gy, 'rx')

    # Draw LIDAR heading
    plot_lidar_heading(ax_env, lidar)

    # Polar LiDAR scan
    plot_scan_polar(scan_data, lidar.resolution, title=f"{lidar.name} Polar Scan", ax=ax_polar)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
