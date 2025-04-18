from copy import deepcopy
from a_star_planner import a_star, build_reservations

def wcbs_plan(starts, goals, grid, planner, max_time=2000):
    num_agents = len(starts)
    all_paths = [[] for _ in range(num_agents)]

    for i in range(num_agents):
        reservations = build_reservations(all_paths, skip_index=i, buffer_time=1, buffer_radius=9, grid_shape=grid.shape)
        path = planner(grid, starts[i], goals[i], reservations=reservations, max_time=max_time)
        if not path:
            print(f"WCBS: Robot {i} failed to find path")
            return []
        all_paths[i] = path

    return all_paths
