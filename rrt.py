import numpy as np
from distance import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time

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
		self.goal = goal
		self.config_space = config_space
		self.nodes = []
		self.edges = {}
		self.nodes.append(self.start)
		# self.axis = axis

	def update(self):

		x = np.random.randint(self.config_space.x_boundaries[0], self.config_space.x_boundaries[1])
		y = np.random.randint(self.config_space.y_boundaries[0], self.config_space.y_boundaries[1])

		target_node = Node(x, y)

		NN_start_time = time.time()
		c_node = closest_point(self.nodes, target_node)
		NN_time = time.time() - NN_start_time

		# Generate 1 second trajectory towards target 
		drive_to_start_time = time.time()
		trajectory = get_trajectory(c_node, target_node)
		drive_to_time = time.time() - drive_to_start_time

		col_check_start_time = time.time()
		is_collided = self.config_space.detect_point_collision(trajectory)
		col_check_time = time.time() - col_check_start_time

		computational_time = (NN_time, drive_to_time, col_check_time)

		if not is_collided:
			x, y = int(trajectory[-1][0]), int(trajectory[-1][1])
			new_node = Node(x, y)
			self.nodes.append(new_node)

			if c_node not in self.edges:
				self.edges[c_node] = []
			self.edges[c_node].append(new_node)

			self.edges[new_node] = [c_node]

			for x,y in trajectory:
				if x >= self.goal[0][0] and x <= self.goal[0][0] + self.goal[1] and y >= self.goal[0][1] and y <= self.goal[0][1] + self.goal[2]:
					return True, computational_time

		return False, computational_time


	def explore(self, show_evolution=True):
		i = 0
		NN_total_time = 0
		drive_to_total_time = 0
		col_check_total_time = 0
		found = False
		while not found:
			# print i
			found, update_time = self.update()
			NN_total_time = NN_total_time + update_time[0]
			drive_to_total_time = drive_to_total_time + update_time[1]
			col_check_total_time = col_check_total_time + update_time[2]

			if i%100 == 0 and show_evolution:
				self.visualize()
			i+=1

		print 'found goal'
		print 'Total elapsed time - '
		print '                Nearest neighbor: ', NN_total_time
		print '                Drive-to : ', drive_to_total_time
		print '                Collision checking: ', col_check_total_time

		self.visualize()


	def visualize(self):
		xy = map(lambda node: node.getxy(), self.nodes)
		x = map(lambda point: point[0], xy)
		y = map(lambda point: point[1], xy)

		ax = self.config_space.plot()

		for key in self.edges:
			curr_x, curr_y = key.getxy()
			connections = self.edges[key]
			for connection in connections:
				conn_x, conn_y = connection.getxy()
				ax.plot([curr_x, conn_x], [curr_y, conn_y], color='b')
		
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
		trajectory_y = np.interp(x_eval, [goal_x, start_x], [goal_y, start_y])
		x_eval = x_eval[::-1]
		trajectory_y = trajectory_y[::-1]
	else:
		x_vals = [start_x, goal_x]
		y_vals = [start_y, goal_y]
		x_eval = np.linspace(start_x, goal_x,num=5)
		trajectory_y = np.interp(x_eval, [start_x, goal_x], [start_y, goal_y])
	
	return zip(x_eval, trajectory_y)