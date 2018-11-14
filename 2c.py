import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import obstacle

## Map Parameters ##
length = 750
width = 750

## Robot Parameters ##
r = 115/2.0

## Obstacle Parameters ##

# Obstacles will be rectangles of length 15 and width 50
obstacle_length = 30
obstacle_width = 50

# Locations
obstacle_locations = [(500, 375), (100, 300), (400, 200), (300, 600)]

obstacles_before = []
obs_rects_before = []

for x, y in obstacle_locations:
	obs = obstacle.Obstacle(x, y, obstacle_length, obstacle_width)

	obstacles_before.append(obs)
	obs_rects_before.append(patches.Rectangle((obs.x, obs.y),obs.width, obs.length, linewidth=1, edgecolor='r', facecolor='r'))

fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1, sharex=True, sharey=True)
rect1 = patches.Rectangle((0,0),width,length,linewidth=1,edgecolor='r',facecolor='none')
rect2_outer = patches.Rectangle((0,0),width,length,linewidth=1,edgecolor='r',facecolor='r')
rect2_inner = patches.Rectangle((r,r),int(width-2*r),int(length-2*r),linewidth=1,edgecolor='r',facecolor='w')


plt.ylim(-10, length+10)
plt.xlim(-10, width+10)
ax1.set_title("Work Space")
ax2.set_title("Configuration Space")
ax1.add_patch(rect1)
ax2.add_patch(rect2_outer)
ax2.add_patch(rect2_inner)
for rect in obs_rects_before:
	ax1.add_patch(rect)


## Change to Configuration Space ##
obs_rects_after = []
for obs in obstacles_before:
	obs.translate(r)
	obs_rects_after.append(patches.Rectangle((obs.x, obs.y) ,obs.width, obs.length, linewidth=1, edgecolor='r', facecolor='r'))

for rect in obs_rects_after:
	ax2.add_patch(rect)


plt.show()



