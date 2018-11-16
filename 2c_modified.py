import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import obstacle
import cspace
import rrt_modified
import copy

np.random.seed(3)


## Map Parameters ##
length = 750
width = 750

## Robot Parameters ##
r = 115/2.0

##### Workspace #####
# Obstacles will be rectangles of length 15 and width 50
obstacle_length = 30
obstacle_width = 50

# Workspace Obstacle locations #
obstacle_locations = [(500, 375), (100, 300), (400, 200), (300, 600)]

# Create Obstacles and their Rectangular Plot Representation #
obstacles = []
for x, y in obstacle_locations:
    obs = obstacle.Obstacle(x, y, obstacle_length, obstacle_width)
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

initial_point = (200,200)
goal_region = [(600,600), 50, 50]

# Create Config Space Object #
config_space = cspace.Cspace(obstacles_work, obstacles_config, (r, width-r), (r, length-r), width, length, initial_point, goal_region)

tree = rrt_modified.RRT((200,200,np.pi/2), goal_region, config_space)
tree.explore(show_evolution=False)