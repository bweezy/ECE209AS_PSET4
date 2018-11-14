

class Cspace:

	def __init__(self, obstacles, x_boundaries, y_boundaries):

		self.obstacles = obstacles
		assert len(x_boundaries) == 2, "provide 2 x boundaries"
		assert len(y_boundaries) == 2, "provide 2 y boundaries"
		self.x_boundaries = x_boundaries
		self.y_boundaries = y_boundaries