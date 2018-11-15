import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import obstacle
import cspace
import rrt_modified

np.random.seed(4)


## Map Parameters ##
length = 750
width = 750

## Robot Parameters ##
r = 115/2.0

# Initialize Plot #
fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1, sharex=True, sharey=True)
plt.ylim(-10, length+10)
plt.xlim(-10, width+10)
ax1.set_title("Work Space")
ax2.set_title("Configuration Space")

##### Workspace #####

# Obstacles will be rectangles of length 15 and width 50
obstacle_length = 30
obstacle_width = 50

# Workspace Obstacle locations #
obstacle_locations = [(500, 375), (100, 300), (400, 200), (300, 600)]

# Create Obstacles and their Rectangular Plot Representation #
obstacles = []
obs_rects_before = []
for x, y in obstacle_locations:
	obs = obstacle.Obstacle(x, y, obstacle_length, obstacle_width)
	obstacles.append(obs)
	obs_rects_before.append(patches.Rectangle((obs.x, obs.y),obs.width, obs.length, linewidth=1, edgecolor='r', facecolor='r'))

# Plot Workspace Border #
rect1 = patches.Rectangle((0,0),width,length,linewidth=1,edgecolor='r',facecolor='none')
ax1.add_patch(rect1)

# Plot Workspace Obstacles#
for rect in obs_rects_before:
	ax1.add_patch(rect)


##### Configuration Space #####
config_origin = (r,r)
config_length = length-2*r
config_width = width-2*r

# Plot Configuration Space Border #
rect2_outer = patches.Rectangle((0,0),width,length,linewidth=1,edgecolor='r',facecolor='r')
rect2_inner = patches.Rectangle((r,r),int(width-2*r),int(length-2*r),linewidth=1,edgecolor='r',facecolor='w')
ax2.add_patch(rect2_outer)
ax2.add_patch(rect2_inner)

# Translate Obstacles to Configuration Space and get their Rectangular Plot Representation #
obs_rects_after = []
for obs in obstacles:
	obs.translate(r)
	obs_rects_after.append(patches.Rectangle((obs.x, obs.y), obs.width, obs.length, linewidth=1, edgecolor='r', facecolor='r'))

# Plot Obstacles #
for rect in obs_rects_after:
	ax2.add_patch(rect)

# Show Plot # 
#plt.show()


goal_region = [(600,600), 50, 50]
# Create Config Space Object #
config_space = cspace.Cspace(obstacles, (r, width-r), (r, length-r), width, length, goal_region)

#print obstacles[1].fill

#print config_space.detect_point_collision([(200,200)])


tree = rrt_modified.RRT((200,200,np.pi/2), goal_region, config_space, ax2)

tree.explore()