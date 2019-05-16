import numpy as np
from setup_1 import *
import matplotlib.pyplot as plt

# Only for factory cases

def create_random_factory(obstacle_density, robot_density,factory_size):
    """Creates a square gridded factory with obstacles as blocks randomly placed"""

    fac = factory()

    # Factory bounds
    fac.bounds = np.array([0, factory_size, 0, factory_size])
    line_bounds = [
        [[fac.bounds[0], fac.bounds[2]], [fac.bounds[0], fac.bounds[3]], [fac.bounds[1], fac.bounds[3]],
         [fac.bounds[1], fac.bounds[2]], [fac.bounds[0], fac.bounds[2]]]]
    fac.walls = [line_bounds]

    # Generate random obstacles, robots and targets
    num_obs = int((factory_size**2)*obstacle_density)
    num_robots = int((factory_size ** 2) * robot_density)
    random_positions = np.array()
    random_positions = np.random.choice(factory_size,(num_obs+2*num_robots,2),replace=False)
    obstacle_positions = random_positions[:num_obs,:]
    robot_positions = random_positions[num_obs:num_obs + num_robots, :]
    target_positions = random_positions[num_obs + num_robots:, :]

    fac.machines = [machine(obs_pos) for obs_pos in obstacle_positions]
    fac.robots = [robot(rob_pos) for rob_pos in robot_positions]

    return fac

if __name__ == "__main__":
    fac = create_random_factory(0.2,0.2,10)
    fac.plot_floor()
    plt.show()