import numpy as np

class Cspace:

	def __init__(self, obstacles, x_boundaries, y_boundaries):

		self.obstacles = obstacles
		assert len(x_boundaries) == 2, "provide 2 x boundaries"
		assert len(y_boundaries) == 2, "provide 2 y boundaries"
		self.x_boundaries = x_boundaries
		self.y_boundaries = y_boundaries


	def detect_point_collision(self, trajectory):

		for x,y in trajectory:
			if x <= self.x_boundaries[0] or x >= self.x_boundaries[1] or y <= self.y_boundaries[0] or y >= self.y_boundaries[1]:
				return True

			for obs in self.obstacles:
				if (x >= obs.x and x <= (obs.x + obs.width)) and (y >= obs.y and y <= (obs.y + obs.length)):
					return True

		return False