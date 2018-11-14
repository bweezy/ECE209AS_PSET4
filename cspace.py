import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

class Cspace:

	def __init__(self, obstacles, x_boundaries, y_boundaries, orig_width, orig_length, goal):

		self.obstacles = obstacles
		assert len(x_boundaries) == 2, "provide 2 x boundaries"
		assert len(y_boundaries) == 2, "provide 2 y boundaries"
		self.x_boundaries = x_boundaries
		self.y_boundaries = y_boundaries
		self.orig_width = orig_width
		self.orig_length = orig_length
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
		obs_rects = []
		for obs in self.obstacles:
			obs_rects.append(patches.Rectangle((obs.x, obs.y), obs.width, obs.length, linewidth=1, edgecolor='r', facecolor='r'))

		rect2_outer = patches.Rectangle((0,0),self.orig_width,self.orig_length,linewidth=1,edgecolor='r',facecolor='r')
		rect2_inner = patches.Rectangle((self.x_boundaries[0],self.y_boundaries[0]), self.x_boundaries[1] - self.x_boundaries[0],
			self.y_boundaries[1] - self.y_boundaries[0],linewidth=1,edgecolor='r',facecolor='w')
		ax.add_patch(rect2_outer)
		ax.add_patch(rect2_inner)

		for rect in obs_rects:
			ax.add_patch(rect)

		rect = patches.Rectangle(self.goal[0], self.goal[1], self.goal[2],linewidth=1,edgecolor='y',facecolor='y')
		ax.add_patch(rect)

		return ax