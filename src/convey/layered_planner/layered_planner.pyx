import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy
from layered_planner.tools import *
from layered_planner.rrt import *
from layered_planner.potential_fields import *
import matplotlib.pyplot as plt
import numpy as np
cimport numpy as np
from libcpp.vector cimport vector
from cython cimport floating

np.import_array()
ctypedef floating[:] float_array


cdef class Robot:
    
    def __init__(self):
        self.sp = [0, 0] #the point where the robot is
        self.sp_global = [0,0]
        self.route = np.array([self.sp])
        self.f = 0
        self.leader = False
        self.vel_array = []
    

    cdef public void SetSp_Global(self,float x, float y):
        self.sp_global = [x, y]
        
    cdef public void getSp_Global(self, vector[float]& vect):
        vect.clear()
        vect.push_back(self.sp_global[0])
        vect.push_back(self.sp_global[1])

    cdef public vector[float] getSp(self, vector[float]& vect):
        vect.clear()
        vect.push_back(self.sp[0])
        vect.push_back(self.sp[1])

    # updates robot setpoitn route, velocity from potential grid and gradient planner
    cdef public void local_planner(self, vector[vector[float_array]] obstacles,float influence_radius, float drone_velocity):
        obstacles_grid = grid_map(obstacles)
        self.f = combined_potential(obstacles_grid, self.sp_global, influence_radius)
        self.sp, self.vel = gradient_planner_next(self.sp, self.f, drone_velocity)
        self.vel_array.append(norm(self.vel))
        self.route = np.vstack( [self.route, self.sp] )

    def formation(num_robots, leader_des, v, l):
        """
        geometry of the swarm: following robots desired locations
        relatively to the leader
        """
        u = np.array([-v[1], v[0]])
        """ followers positions """
        des2 = leader_des - v*l*sqrt(3)/2 + u*l/2
        des3 = leader_des - v*l*sqrt(3)/2 - u*l/2
        des4 = leader_des - v*l*sqrt(3)
        des5 = leader_des - v*l*sqrt(3)   + u*l
        des6 = leader_des - v*l*sqrt(3)   - u*l
        des7 = leader_des - v*l*sqrt(3)*3/2 - u*l/2
        des8 = leader_des - v*l*sqrt(3)*3/2 + u*l/2
        des9 = leader_des - v*l*sqrt(3)*2
        if num_robots<=1: return []
        if num_robots==2: return [des4]
        if num_robots==3: return [des2, des3]
        if num_robots==4: return [des2, des3, des4]
        if num_robots==5: return [des2, des3, des4, des5]
        if num_robots==6: return [des2, des3, des4, des5, des6]
        if num_robots==7: return [des2, des3, des4, des5, des6, des7]
        if num_robots==8: return [des2, des3, des4, des5, des6, des7, des8]
        if num_robots==9: return [des2, des3, des4, des5, des6, des7, des8, des9]

        return [des2, des3, des4]
    cdef nextStepFromSingleWPT(P, float drone_velocity,int ViconRate):
        V = drone_velocity * 1.3 # [m/s], the setpoint should travel a bit faster than the robot
        freq = ViconRate # vicon rate is teh sample rate
        dt = 1./freq
        dx = V * dt # m/s * s = m
        vec = P[1]-P[0]
        dist = norm(vec)
        unit_vector = vec/dist
