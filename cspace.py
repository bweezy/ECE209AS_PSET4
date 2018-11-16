import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

## Map Parameters ##
length = 750
width = 750

## Robot Parameters ##
r = 115/2.0


class Cspace:

	def __init__(self, obstacles_work, obstacles_config, x_boundaries, y_boundaries, orig_width, orig_length, init_point, goal):

		self.obstacles = obstacles_config
		self.obstacles_work = obstacles_work
		assert len(x_boundaries) == 2, "provide 2 x boundaries"
		assert len(y_boundaries) == 2, "provide 2 y boundaries"
		self.x_boundaries = x_boundaries
		self.y_boundaries = y_boundaries
		self.orig_width = orig_width
		self.orig_length = orig_length
		self.init_point = init_point
		self.goal = goal


	def detect_point_collision(self, trajectory):

		for x,y in trajectory:
			if x <= self.x_boundaries[0] or x >= self.x_boundaries[1] or y <= self.y_boundaries[0] or y >= self.y_boundaries[1]:
				return True

			for obs in self.obstacles:
				if (x >= obs.x and x <= (obs.x + obs.width)) and (y >= obs.y and y <= (obs.y + obs.length)):
					return True

		return False


	def plot(self):
		fig, ax = plt.subplots(1)
		# obs_rects = []
		# for obs in self.obstacles:
		# 	obs_rects.append(patches.Rectangle((obs.x, obs.y), obs.width, obs.length, linewidth=1, edgecolor='r', facecolor='r'))

		# rect2_outer = patches.Rectangle((0,0),self.orig_width,self.orig_length,linewidth=1,edgecolor='r',facecolor='r')
		# rect2_inner = patches.Rectangle((self.x_boundaries[0],self.y_boundaries[0]), self.x_boundaries[1] - self.x_boundaries[0],
		# 	self.y_boundaries[1] - self.y_boundaries[0],linewidth=1,edgecolor='r',facecolor='w')
		# ax.add_patch(rect2_outer)
		# ax.add_patch(rect2_inner)

		# for rect in obs_rects:
		# 	ax.add_patch(rect)

		# rect = patches.Rectangle(self.goal[0], self.goal[1], self.goal[2],linewidth=1,edgecolor='y',facecolor='y')
		# ax.add_patch(rect)


		# Plot Configuration Space Border #
		rect2_outer = patches.Rectangle((0,0),width,length,linewidth=1,edgecolor='r',facecolor='r')
		rect2_config_mask = patches.Rectangle((2,2),width-4,length-4,linewidth=0,edgecolor='pink', facecolor='pink')
		rect2_inner = patches.Rectangle((r,r),int(width-2*r),int(length-2*r),linewidth=1,edgecolor='w',facecolor='w')
		ax.add_patch(rect2_outer)
		ax.add_patch(rect2_config_mask)
		ax.add_patch(rect2_inner)

		# Translate Obstacles to Configuration Space and get their Rectangular Plot Representation #
		obs_rects_after = []
		obs_rects_before = []
		for obs in self.obstacles_work:
			obs_rects_before.append(patches.Rectangle((obs.x, obs.y),obs.width, obs.length, linewidth=0, edgecolor='r', facecolor='r'))
		
		for obs in self.obstacles:
			obs_rects_after.append(patches.Rectangle((obs.x, obs.y), obs.width, obs.length, linewidth=0, edgecolor='pink', facecolor='pink'))

		# Plot Obstacles #
		for rect in obs_rects_after:
			ax.add_patch(rect)

		# Plot Workspace Obstacles#
		for rect in obs_rects_before:
			ax.add_patch(rect)

		# Plot Initial Point #
		circ = patches.Circle(self.init_point, 10, edgecolor='g',facecolor='g')
		ax.add_patch(circ)

		# Plot Goal #
		rect = patches.Rectangle(self.goal[0], self.goal[1], self.goal[2],linewidth=1,edgecolor='y',facecolor='y')
		ax.add_patch(rect)

		# plt.ylim(-10, length+10)
		# plt.xlim(-10, width+10)

		return ax