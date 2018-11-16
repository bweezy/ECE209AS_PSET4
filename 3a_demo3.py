import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import obstacle
import cspace
# import rrt_modified as rrt
import rrt
import random
import copy

np.random.seed(1)

## Map Parameters ##
length = 750
width = 750

## Robot Parameters ##
r = 115/2.0

##### Workspace #####
# Create Obstacles and their Rectangular Plot Representation #
obstacles = []

obs = obstacle.Obstacle(150, 150, 400, 30)
obstacles.append(obs)

obs = obstacle.Obstacle(300, 300, 400, 30)
obstacles.append(obs)

obs = obstacle.Obstacle(500, 150, 400, 30)
obstacles.append(obs)

obs = obstacle.Obstacle(625, 340, 70, 10)
obstacles.append(obs)

obs = obstacle.Obstacle(450, 150, 30, 250)
obstacles.append(obs)

obs = obstacle.Obstacle(30, 150, 30, 300)
obstacles.append(obs)

obs = obstacle.Obstacle(500, 550, 30, 100)
obstacles.append(obs)

obs = obstacle.Obstacle(625, 400, 30, 75)
obstacles.append(obs)

obstacles_work = copy.deepcopy(obstacles)

##### Configuration Space #####
config_origin = (r,r)
config_length = length-2*r
config_width = width-2*r

# Translate Obstacles to Configuration Space and get their Rectangular Plot Representation #
obs_rects_after = []
for obs in obstacles:
	obs.translate(r)

obstacles_config = copy.deepcopy(obstacles)

initial_point = (600,70)
goal_region = [(70,250), 50, 50]

# Create Config Space Object #
config_space = cspace.Cspace(obstacles_work, obstacles_config, (r, width-r), (r, length-r), width, length, initial_point, goal_region)

# tree = rrt.RRT((100,100,np.pi/2), goal_region, config_space)
tree = rrt.RRT(initial_point, goal_region, config_space)
tree.explore(show_evolution=False)