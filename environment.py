import matplotlib.pyplot as plt
import numpy as np

class Environment:
  def __init__(self, grid_size, resolution):
    self.grid_size = grid_size
    self.resolution = resolution  # number of grid cells per meter
    self.grid = np.zeros((grid_size * resolution, grid_size * resolution))
    self.objects = []
    self.dynamic_objects = []

  def world_to_grid(self, x, y):
    gx = int(round(x * self.resolution))
    gy = int(round(y * self.resolution))
    return gx, gy

  def add_object(self, obj):
    if self.is_valid_position(obj):
      self.objects.append(obj)
      self.update_grid_for_object(obj, occupied=1)
    else:
      raise ValueError(f"Invalid position {obj.position} for object {obj.name}")

  def remove_object(self, obj):
    if obj in self.objects:
      self.objects.remove(obj)
      self.update_grid_for_object(obj, occupied=0)
    else:
      raise ValueError(f"Object {obj.name} not found")

  def is_valid_position(self, obj):
    cx, cy = obj.position
    if obj.type == "circle":
      radius = obj.size / 2
      for dx in np.linspace(-radius, radius, int(radius * self.resolution * 2)):
        for dy in np.linspace(-radius, radius, int(radius * self.resolution * 2)):
          if dx**2 + dy**2 <= radius**2:
            gx, gy = self.world_to_grid(cx + dx, cy + dy)
            if not (0 <= gx < self.grid.shape[0] and 0 <= gy < self.grid.shape[1]):
              return False
            if self.grid[gx, gy] != 0:
              return False
    elif obj.type == "rectangle":
      width, height = obj.size
      for dx in np.linspace(-width/2, width/2, int(width * self.resolution)):
        for dy in np.linspace(-height/2, height/2, int(height * self.resolution)):
          gx, gy = self.world_to_grid(cx + dx, cy + dy)
          if not (0 <= gx < self.grid.shape[0] and 0 <= gy < self.grid.shape[1]):
            return False
          if self.grid[gx, gy] != 0:
            return False
    return True

  def update_grid_for_object(self, obj, occupied):
    cx, cy = obj.position
    if obj.type == "circle":
      radius = obj.size / 2
      for dx in np.linspace(-radius, radius, int(radius * self.resolution * 2)):
        for dy in np.linspace(-radius, radius, int(radius * self.resolution * 2)):
          if dx**2 + dy**2 <= radius**2:
            gx, gy = self.world_to_grid(cx + dx, cy + dy)
            if 0 <= gx < self.grid.shape[0] and 0 <= gy < self.grid.shape[1]:
              self.grid[gx, gy] = occupied
    elif obj.type == "rectangle":
      width, height = obj.size
      for dx in np.linspace(-width/2, width/2, int(width * self.resolution)):
        for dy in np.linspace(-height/2, height/2, int(height * self.resolution)):
          gx, gy = self.world_to_grid(cx + dx, cy + dy)
          if 0 <= gx < self.grid.shape[0] and 0 <= gy < self.grid.shape[1]:
            self.grid[gx, gy] = occupied

  def display_environment(self, ax=None, title="Environment Grid", show=True):
    if ax is None:
      fig, ax = plt.subplots()
    ax.imshow(self.grid.T, cmap='gray', origin='lower', interpolation='nearest')
    ax.set_title(title)
    ax.set_xlabel("X-axis (meters)")
    ax.set_ylabel("Y-axis (meters)")
    ax.set_aspect('equal')
    if show:
      plt.show()
    return ax  # Return the axes in case caller wants to modify further

  def get_objects(self):
    return self.objects

  def get_dynamic_objects(self):
    return self.dynamic_objects

class Object:
  def __init__(self, name, position, type, size):
    self.name = name
    self.position = position  # (x, y)
    self.type = type  # 'circle' or 'rectangle'
    self.size = size  # float for circle, (width, height) tuple for rectangle

  def __str__(self):
    return f"Object(name={self.name}, position={self.position}, type={self.type})"

# Example usage:
if __name__ == "__main__":
  env = Environment(grid_size=10, resolution=100)

  obj1 = Object("Circle1", (2, 3), "circle", 1.0)
  obj2 = Object("Rect1", (5, 5), "rectangle", (2.0, 3.0))

  env.add_object(obj1)
  env.add_object(obj2)

  env.display_environment()
