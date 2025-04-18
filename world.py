# world.py
from environment import Environment, Object

def make_environment(grid_size=5, resolution=100):
    env = Environment(grid_size=grid_size, resolution=resolution)
    # env.add_object(Object("Circle", (3, 3), "circle", 1.0))
    env.add_object(Object("Rectangle", (1, 2), "rectangle", (1, 2)))
    env.add_object(Object("Rectangle", (4, 4), "rectangle", (1, 1)))
    env.add_object(Object("Circle", (2, 4), "circle", 0.5))
    env.add_object(Object("Circle", (4, 2), "circle", 0.75))
    env.add_object(Object("Circle", (1, 4), "circle", 0.5))
    env.add_object(Object("Circle", (2, 1), "circle", 0.5))
    env.add_object(Object("Circle", (4, 1), "circle", 0.5))
    env.add_object(Object("Rectangle", (3, 1), "rectangle", (1, 2)))
    return env
