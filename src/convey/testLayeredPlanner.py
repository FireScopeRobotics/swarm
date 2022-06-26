#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

from layered_planner.tools import *
from layered_planner.rrt import *
from layered_planner.potential_fields import *
import matplotlib.pyplot as plt


class Params:
    def __init__(self):
        self.animate = 1 # show RRT construction, set 0 to reduce time of the RRT algorithm
        self.visualize = 1 # show constructed paths at the end of the RRT and path smoothing algorithms
        self.maxiters = 5000 # max number of samples to build the RRT
        self.goal_prob = 0.05 # with probability goal_prob, sample the goal
        self.minDistGoal = 0.25 # [m], min distance os samples from goal to add goal node to the RRT
        self.extension = 0.4 # [m], extension parameter: this controls how far the RRT extends in each step.
        self.world_bounds_x = [-2.5, 2.5] # [m], map size in X-direction
        self.world_bounds_y = [-2.5, 2.5] # [m], map size in Y-direction
        self.drone_vel = 4.0 # [m/s]
        self.ViconRate = 100 # [Hz]
        self.max_sp_dist = 0.3 * self.drone_vel # [m], maximum distance between current robot's pose and the sp from global planner
        self.influence_radius = 1.22 # potential fields radius, defining repulsive area size near the obstacle
        self.goal_tolerance = 0.05 # [m], maximum distance threshold to reach the goal
        self.num_robots = 3
        self.moving_obstacles = 0 # move small cubic obstacles or not



passage_width = 0.25
passage_location = 0.0
if __name__ == "__main__":
  # map generator:
  # in order: bottom left, bottom right, top right, top left
  obstacles = [
          # narrow passage
            np.array([[-2.5, -0.5], [-passage_location-passage_width/2., -0.5], [-passage_location-passage_width/2., 0.5], [-2.5, 0.5]]),
            np.array([[-passage_location+passage_width/2., -0.5], [2.5, -0.5], [2.5, 0.5], [-passage_location+passage_width/2., 0.5]]),
            np.array([[2, 2], [3, 2], [3, 3], [2,3]])
          ]
  plt.figure(figsize=(10,10))
  draw_map(obstacles, [-5,5], [-5,5])
  plt.draw()

  xy_start = np.array([1.4, 0.9])
  xy_goal =  np.array([1.5, -1.4])
  
  # rrt stuff: inside of calling the RRT functions, we can pass in a list of points as the RRT path 
  # in the future, we can use relative or global position
  # P = ShortenPath(P_long, obstacles, smoothiters=30)
  traj_global = waypts2setpts(P, params); P = np.vstack([P, xy_start])

  P = np.array([[ 1.5,        -1.4       ],
              [ 1.56253855, -1.53748091],
              [ 1.60423092,-1.62913484],
              [ 1.41396189, -1.66648865],
              [ 1.09274323, -1.57042937],
              [ 0.44432792, -1.63513007],
              [-0.08844011, -1.06158115],
              [ 0.05630422, -0.46986452],
              [ 0.11570076, -0.12651434],
              [-0.12021113,  0.40439351],
              [ 0.86717357,  1.46305047],
              [ 1.52035858,  1.29936696],
              [ 1.4,         0.9       ]])
