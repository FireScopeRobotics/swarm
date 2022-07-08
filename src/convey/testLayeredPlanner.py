#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy

from layered_planner.tools import *
from layered_planner.rrt import *
from layered_planner.potential_fields import *
import matplotlib.pyplot as plt

#data types
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

class Robot:
    def __init__(self):
        self.sp = [0, 0] #the point where the robot is
        self.sp_global = [0,0]
        self.route = np.array([self.sp])
        self.f = 0
        self.leader = False
        self.vel_array = []

# updates robot setpoitn route, velocity from potential grid and gradient planner
    def local_planner(self, obstacles, params):
        obstacles_grid = grid_map(obstacles)
        self.f = combined_potential(obstacles_grid, self.sp_global, params.influence_radius)
        self.sp, self.vel = gradient_planner_next(self.sp, self.f, params)
        self.vel_array.append(norm(self.vel))
        self.route = np.vstack( [self.route, self.sp] )

#global variables
start_pos = np.array([0,0])
robots = []
for i in range(3):
    robots.append(Robot())
robot1 = robots[0]; robot1.leader=True
robot1.route = np.array([start_pos]) #history of places the robot has been to
robot1.sp = start_pos  #initialize position of robots

mousePos = np.array([0,0], dtype = float)
mouse_move_id = 0
movement_flag = False

#function defs
def mouse_move(event):
  x, y = event.xdata, event.ydata
  global mousePos
  mousePos[0] = x
  mousePos[1] = y

def listen_mouse_movement(event):
  global mouse_move_id
  mouse_move_id = plt.connect("motion_notify_event", mouse_move)
  print("moving robot")
  global movement_flag
  movement_flag = True

def ignore_mouse_movement(event):
  global mouse_move_id
  global mousePos
  global movement_flag
  plt.disconnect(mouse_move_id)
  print("stopping robot")
  mousePos[0] = robot1.sp[0]
  mousePos[1] = robot1.sp[1]
  movement_flag = False


if __name__ == "__main__":
  # map generator:
  # in order: bottom left, bottom right, top right, top left
  obstacles = [
          # narrow passage
          np.array([[2, 2], [3, 2], [3, 3], [2,3]]),
          np.array([[-2, 1], [-1.5, 1], [-1.5, 1.5], [-2,1.5]]),
          np.array([[-1.4, -3], [-1.5, -3], [-1.5, 0], [-1.4,0]])
          ]
  plt.figure(figsize=(10,10))
  draw_map(obstacles, [-5,5], [-5,5])

  #get mouse location as coordinates:
  plt.connect("button_press_event", listen_mouse_movement)
  plt.connect("button_release_event", ignore_mouse_movement)

  params = Params()
  while True:
    goal = deepcopy(mousePos)
    # print(goal)
    goal_vec = goal - robot1.sp 
    dist_to_goal = norm(goal_vec)
    if dist_to_goal > 0.5:
      goal_dir = goal_vec/dist_to_goal*0.5
      goal = robot1.sp + goal_dir

    if movement_flag:  
      P = np.stack([robot1.sp, goal])
      pt = nextStepFromSingleWPT(P, params)
      robot1.sp_global = pt
      robot1.local_planner(obstacles, params)

        
    plt.cla()
    draw_map(obstacles, [-5,5], [-5,5])
    plt.plot(robot1.sp[0], robot1.sp[1], '^', color='green', markersize=10, zorder=15)
    if movement_flag:
      plt.plot(goal[0], goal[1], '.', color='yellow', markersize=10, zorder=15)
    plt.draw()
    plt.pause(0.02)

