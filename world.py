# world.py
from environment import Environment, Object

def make_environment(grid_size=10, resolution=100):
    env = Environment(grid_size=grid_size, resolution=resolution)
    env.add_object(Object("Circle", (3, 3), "circle", 1.0))
    env.add_object(Object("Rectangle", (6, 6), "rectangle", (2.0, 1.0)))
    return env
