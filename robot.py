import numpy as np
from lidar import LIDAR

class Robot:
    def __init__(self, name, x, y, theta, environment, lidar_resolution=15, lidar_range=10.0):
        self.name = name
        self.x = x
        self.y = y
        self.theta = theta  # in radians
        self.environment = environment

        # Create LIDAR and mount it on the robot
        self.lidar = LIDAR(
            name=f"{name}_lidar",
            resolution=lidar_resolution,
            range=lidar_range,
            environment=environment
        )
        self.lidar.set_location(x, y)
        self.lidar.set_rotation(np.degrees(self.theta))

        self.scan_data = []

    # use holonomic motion model to move the robot
    def move(self, dx, dy, dtheta):
        self.x += dx
        self.y += dy
        self.theta += dtheta

        # Update LIDAR position and rotation
        self.lidar.set_location(self.x, self.y)
        self.lidar.set_rotation(np.degrees(self.theta))

    def update_scan(self):
        self.scan_data = self.lidar.scan()
        return self.scan_data

    def get_pose(self):
        return (self.x, self.y, self.theta)

    def get_lidar_hits(self):
        return self.lidar.get_last_hit_points()
