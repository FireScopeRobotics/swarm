#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

from layered_planner.tools import *
from layered_planner.rrt import *
from layered_planner.potential_fields import *
import matplotlib.pyplot as plt



passage_width = 0.25
passage_location = 0.0
if __name__ == "__main__":
    # map generator:
    # in order: bottom left, bottom right, top left, top right
    obstacles = [
            # narrow passage
              np.array([[-2.5, -0.5], [-passage_location-passage_width/2., -0.5], [-passage_location-passage_width/2., 0.5], [-2.5, 0.5]]),
              np.array([[-passage_location+passage_width/2., -0.5], [2.5, -0.5], [2.5, 0.5], [-passage_location+passage_width/2., 0.5]]),
              np.array([[2, 2], [3, 2], [3, 3], [2,3]])
            ]
    plt.figure(figsize=(10,10))
    draw_map(obstacles, [-5,5], [-5,5])
    plt.draw()
    plt.pause(100)
