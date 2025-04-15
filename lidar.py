import numpy as np
import matplotlib.pyplot as plt

class LIDAR:
    def __init__(self, name, resolution, range, environment, noise_std=0.1):
        self.name = name
        self.resolution = resolution  # degrees per scan step
        self.range = range
        self.translation = (0, 0)     # (x, y) position in meters
        self.rotation = 0             # rotation in degrees (0 = facing up)
        self.environment = environment
        self.noise_std = noise_std

    def get_lidar_info(self):
        return f"LIDAR {self.name} is located at {self.translation} with a rotation of {self.rotation} degrees."

    def set_location(self, x, y):
        if self.environment.is_valid_position(
            type('FakeObj', (), {'position': (x, y), 'type': 'circle', 'size': 0.0})
        ):
            self.translation = (x, y)
        else:
            raise ValueError(f"Invalid location {x, y} for LIDAR {self.name}")

    def set_rotation(self, angle):
        self.rotation = angle % 360  # Normalize angle to [0, 360)

    def scan(self):
        distances = []
        self.hit_points = []  # (x, y) for each ray
        for angle in range(0, 360, self.resolution):
            distance, hit = self.calculate_distance(angle)
            noisy_distance = distance + np.random.normal(0, self.noise_std * (distance / self.range))
            noisy_distance = np.clip(noisy_distance, 0, self.range)
            distances.append(noisy_distance)
            self.hit_points.append(hit)
        return distances

    def calculate_distance(self, angle):
        """Returns (distance, (x_hit, y_hit))"""
        theta = np.radians(angle + self.rotation)
        x0, y0 = self.translation
        min_distance = self.range
        x_hit, y_hit = x0 + self.range * np.cos(theta), y0 + self.range * np.sin(theta)

        for r in np.linspace(0, self.range, num=200):
            x = x0 + r * np.cos(theta)
            y = y0 + r * np.sin(theta)

            # Check bounds
            if not self.environment.is_valid_position(
                type('FakeObj', (), {'position': (x, y), 'type': 'circle', 'size': 0.0})
            ):
                return r, (x, y)

            for obj in self.environment.get_objects() + self.environment.get_dynamic_objects():
                cx, cy = obj.position
                if obj.type == "circle":
                    radius = obj.size / 2
                    if np.linalg.norm((x - cx, y - cy)) <= radius:
                        return r, (x, y)
                elif obj.type == "rectangle":
                    width, height = obj.size
                    if (cx - width / 2 <= x <= cx + width / 2) and (cy - height / 2 <= y <= cy + height / 2):
                        return r, (x, y)

                # Check if the point is within the grid bounds
                gx, gy = self.environment.world_to_grid(x, y)
                if not (0 <= gx < self.environment.grid.shape[0] and 0 <= gy < self.environment.grid.shape[1]):
                    return r, (x, y)

        return np.nan, (x_hit, y_hit)

    def get_last_hit_points(self):
        return self.hit_points  # Returns [(x1, y1), (x2, y2), ...] from last scan


def plot_scan_polar(scan_data, resolution_deg, title="LiDAR Scan", ax=None):
    angles_deg = np.arange(0, 360, resolution_deg)
    angles_rad = np.radians(angles_deg)

    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='polar')

    ax.plot(angles_rad, scan_data, marker='o', linestyle='-', color='tab:blue')
    ax.set_title(title)
    ax.set_theta_zero_location("N")  # Zero degrees is "up"
    ax.set_theta_direction(1)       # Counterclockwise rotation
    ax.set_rlabel_position(135)      # Move radial labels out of way
    ax.grid(True)

    return ax

