import numpy as np
from distance import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import lines
from get_trajectory_input import get_trajectory_input

## Map Parameters ##
length = 750
width = 750

class Node:

	def __init__(self, x, y, theta=None):
		self.x = x
		self.y = y
		self.theta = theta
		self.parent = None

	def get_x_y_theta(self):
		return (self.x, self.y, self.theta)

	def getxy(self):
		return (self.x, self.y)


class RRT:


	def __init__(self, start, goal, config_space, axis):

		self.start = Node(start[0], start[1], start[2])
		self.goal = goal
		self.config_space = config_space
		self.nodes = []
		self.edges = {}
		self.nodes.append(self.start)
		self.axis = axis

	def update(self):

		x = np.random.randint(self.config_space.x_boundaries[0], self.config_space.x_boundaries[1])
		y = np.random.randint(self.config_space.y_boundaries[0], self.config_space.y_boundaries[1])

		target_node = Node(x, y)
		c_node = closest_point(self.nodes, target_node)

		# Generate 1 second trajectory towards target 
		trajectory, trajectory_obj, final_angle = get_trajectory(c_node, target_node)

		if trajectory == []:
			return False, []

		if not self.config_space.detect_point_collision(trajectory):
			x, y = int(trajectory[-1][0]), int(trajectory[-1][1])
			new_node = Node(x, y, final_angle)
			self.nodes.append(new_node)

			# import pdb; pdb.set_trace()

			if c_node not in self.edges:
				self.edges[c_node] = []
			self.edges[c_node].append(new_node)

			self.edges[new_node] = [c_node]

			for x,y in trajectory:
				if x >= self.goal[0][0] and x <= self.goal[0][0] + self.goal[1] and y >= self.goal[0][1] and y <= self.goal[0][1] + self.goal[2]:
					return True, trajectory_obj

			return False, trajectory_obj

		else:
			return False, []



	def explore(self):
		ax = self.config_space.plot()

		i = 0
		found = False
		trajectory_obj_list = []
		while not found:
			# print i
			found, trajectory_obj = self.update()
			trajectory_obj_list.append(trajectory_obj)
			# if i%100 == 0:
				# import pdb; pdb.set_trace()
				# print trajectory_obj_list
				# self.visualize(trajectory_obj_list, ax)
				# trajectory_obj_list = []
			i+=1

		print 'found goal'
		self.visualize(trajectory_obj_list, ax)


		plt.ylim(-30, length+30)
		plt.xlim(-30, width+30)
		plt.show()


	def visualize(self, trajectory_obj_list, ax):
		xy = map(lambda node: node.getxy(), self.nodes)
		x = map(lambda point: point[0], xy)
		y = map(lambda point: point[1], xy)

		# ax = self.config_space.plot()


		# for key in self.edges:
		# 	curr_x, curr_y = key.getxy()
		# 	connections = self.edges[key]
		# 	for connection in connections:
		# 		conn_x, conn_y = connection.getxy()
		# 		ax.plot([curr_x, conn_x], [curr_y, conn_y], color='b')
		for trajectory_obj in trajectory_obj_list:
			if trajectory_obj != []:
				if type(trajectory_obj) is lines.Line2D:
					ax.add_line(trajectory_obj)
				else:
					ax.add_patch(trajectory_obj)
		
		# plt.draw()



def get_trajectory(start, goal):
	max_angular_velocity = 10.0/3.0*np.pi
	distance_1sec = 50 #mm

	start_x, start_y, start_theta = start.get_x_y_theta()
	goal_x, goal_y = goal.getxy()

	if distance(start.getxy(), goal.getxy()) > distance_1sec:
		rise = goal_y - start_y
		run = goal_x - start_x

		a = 50.0/np.sqrt(run**2 + rise**2)

		goal_x = a*run + start_x
		goal_y = a*rise + start_y

	trajectory_obj, omega_l, omega_r, final_angle = get_trajectory_input((start_x, start_y, start_theta),(goal_x, goal_y))

	# new_x, new_y, new_theta = robot_model(omega_l, omega_r, start_x, start_y, start_theta)
	if type(trajectory_obj) is lines.Line2D:
		x_eval = np.linspace(goal_x,start_x,num=10)
		trajectory_y = np.interp(x_eval, [goal_x, start_x], [goal_y, start_y])
		x_eval = x_eval[::-1]
		trajectory_y = trajectory_y[::-1]
	elif type(trajectory_obj) is patches.Arc:
		# print type(trajectory_obj)
		angles = np.linspace(trajectory_obj.theta1, trajectory_obj.theta2, num=10)
		radius = trajectory_obj.height/2.0

		# print radius
		center = trajectory_obj.center
		x_eval = radius*np.cos(angles*np.pi/180) + center[0]
		trajectory_y = radius*np.sin(angles*np.pi/180) + center[1]

		arc_length = abs(trajectory_obj.theta1-trajectory_obj.theta2)*radius*np.pi/180.0

		if distance_1sec * 3 < arc_length:
			print arc_length
			return [], [], []
	
	return zip(x_eval, trajectory_y), trajectory_obj, final_angle


def robot_model(omega_l, omega_r, x, y, theta, delta_t):
	r = 20.0 # wheel radius 20mm 
	b = 85.0 # wheel distance = 85mm
	new_x = x + float(b*(omega_l+omega_r))/(2*(omega_r-omega_l))*(np.sin(float(omega_r-omega_l)*delta_t/b + theta) - np.sin(theta))
	new_y = y - float(b*(omega_l+omega_r))/(2*(omega_r-omega_l))*(np.cos(float(omega_r-omega_l)*delta_t/b + theta) - np.cos(theta))
	new_theta = r*(omega_r-omega_l)*delta_t/b + theta

	return new_x, new_y, new_theta