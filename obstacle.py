import numpy as np


class Obstacle:

	def __init__(self, x, y, length, width):
		self.update(x, y, length, width)

	def update(self, x, y, length, width):

		self.x = x
		self.y = y
		self.length = length
		self.width = width
		self.border = np.empty((2*(width + length),2))
		self.fill = np.empty((width*length,2))

		for i in np.arange(width):
			for j in np.arange(length):
				self.fill[i*length + j][0] = x + i
				self.fill[i*length + j][1] = y + j

		for i in np.arange(width):
			self.border[i][0] = x + i 
			self.border[i][1] = y
			self.border[i+1][0] = x + i
			self.border[i+1][1] = y + length

		for i in np.arange(length):
			self.border[width + i][0] = x
			self.border[width + i][1] = y + i
			self.border[width + i][0] = x + width
			self.border[width + i][1] = y + i

	def translate(self, r):
		new_x = max(0, self.x - r)
		new_y = max(0, self.y - r)

		self.update(new_x, new_y, int(np.ceil(self.length + r)), int(np.ceil(self.width + r)))




