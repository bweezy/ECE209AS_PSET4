import numpy as np


class Obstacle:

	def __init__(self, x, y, length, width):
		self.update(x, y, length, width)

	def update(self, x, y, length, width):

		self.x = x
		self.y = y
		self.length = length
		self.width = width

	def translate(self, r):
		new_x = max(0, self.x - r)
		new_y = max(0, self.y - r)

		self.update(new_x, new_y, int(np.ceil(self.length + r)), int(np.ceil(self.width + r)))
