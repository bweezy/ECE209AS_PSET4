import numpy as np
from distance import *
import matplotlib.pyplot as plt

class Node:

	def __init__(self, x, y):
		self.x = x
		self.y = y
		self.parent = None

	def getxy(self):
		return (self.x, self.y)


class RRT:


	def __init__(self, start, goal, config_space):

		self.start = Node(start[0], start[1])
		self.goal = Node(goal[0], goal[1])
		self.config_space = config_space
		self.nodes = []
		self.edges = {}
		self.nodes.append(self.start)

	def update(self):

		x = np.random.randint(self.config_space.x_boundaries[0], self.config_space.x_boundaries[1])
		y = np.random.randint(self.config_space.y_boundaries[0], self.config_space.y_boundaries[1])

		target_node = Node(x, y)
		c_node = closest_point(self.nodes, target_node)

		# Generate 1 second trajectory towards target 
		trajectory = get_trajectory(c_node, target_node)

		if not self.config_space.detect_point_collision(trajectory):
			new_node = Node(int(trajectory[-1][0]), int(trajectory[-1][1]))
			self.nodes.append(new_node)

			if c_node not in self.edges:
				self.edges[c_node] = []
			self.edges[c_node].append(new_node)

			self.edges[new_node] = [c_node]


	def explore(self):

		i = 0
		while self.goal not in self.nodes:
			print i
			self.update()
			self.visualize()
			i+=1

		print 'found goal'

	def visualize(self):
		xy = map(lambda node: node.getxy(), self.nodes)
		x = map(lambda point: point[0], xy)
		y = map(lambda point: point[1], xy)

		for key in self.edges:
			curr_x, curr_y = key.getxy()
			connections = self.edges[key]
			for connection in connections:
				goal_x, goal_y = connection.getxy()
				plt.arrow(curr_x, curr_y, goal_x-curr_x, goal_y-curr_y, head_width=0, head_length=0)
		plt.scatter(x,y)
		plt.show()



def get_trajectory(start, goal):

	distance_1sec = 50 #mm

	start_x, start_y = start.getxy()
	goal_x, goal_y = goal.getxy()
	# Find maximum reachable target point
	if distance(start.getxy(), goal.getxy()) > distance_1sec:
		rise = goal_y - start_y
		run = goal_x - start_x

		a = 50.0/np.sqrt(run**2 + rise**2)

		goal_x = a*run + start_x
		goal_y = a*rise + start_y
	
	if start_x > goal_x:
		x_vals = [goal_x, start_x]
		y_vals = [goal_y, start_y]
		x_eval = np.linspace(goal_x,start_x,num=5)
	else:
		x_vals = [start_x, goal_x]
		y_vals = [start_y, goal_y]
		x_eval = np.linspace(start_x, goal_x,num=5)

	

	trajectory_y = np.interp(x_eval, [start_x, goal_x], [start_y, goal_y])

	return zip(x_eval, trajectory_y)