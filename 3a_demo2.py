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
# Obstacles will be rectangles of length 15 and width 50
obstacle_length = 260
obstacle_width = 25

# Workspace Obstacle locations #
obstacle_locations = [(690, 50), (690, 440)]

# Create Obstacles and their Rectangular Plot Representation #
obstacles = []
for x, y in obstacle_locations:
	obs = obstacle.Obstacle(x, y, obstacle_length, obstacle_width)
	obstacles.append(obs)

obs = obstacle.Obstacle(300, 400, 100, 100)
obstacles.append(obs)

obs = obstacle.Obstacle(200, 150, 30, 500)
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
goal_region = [(670,350), 50, 50]

# Create Config Space Object #
config_space = cspace.Cspace(obstacles_work, obstacles_config, (r, width-r), (r, length-r), width, length, initial_point, goal_region)

# tree = rrt.RRT((100,100,np.pi/2), goal_region, config_space)
tree = rrt.RRT(initial_point, goal_region, config_space)
tree.explore(show_evolution=False)