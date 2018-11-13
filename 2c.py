import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import obstacle

## Map Parameters ##
length = 500
width = 500

## Robot Parameters ##
r = 115/2.0

## Obstacle Parameters ##

# Obstacles will be rectangles of length 15 and width 50
obstacle_length = 15
obstacle_width = 50

# Locations
obstacle_locations = [(250, 300), (100, 150), (400, 200)]

obstacles = []
obs_rects = []

for x, y in obstacle_locations:
	obs = obstacle.Obstacle(x, y, obstacle_length, obstacle_width)

	obstacles.append(obs)
	obs_rects.append(patches.Rectangle((x,y), obstacle_width, obstacle_length, linewidth=1, edgecolor='r', facecolor='r'))

fig, axes = plt.subplots(nrows=2,ncols=1)
rect1 = patches.Rectangle((0,0),width,length,linewidth=1,edgecolor='r',facecolor='none')
rect2 = patches.Rectangle((r,r),int(width-2*r),int(length-2*r),linewidth=1,edgecolor='r',facecolor='none')


plt.ylim(-10, length+10)
plt.xlim(-10, width+10)
axes[0].add_patch(rect1)
axes[1].add_patch(rect2)
for rect in obs_rects:
	axes[1].add_patch(rect)

plt.show()
## Change to Configuration Space ##
for obs in obstacles:
	obs.translate(r)






