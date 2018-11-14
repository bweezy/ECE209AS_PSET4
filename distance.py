import numpy as np

def distance(given, target):
	x1, y1 = given
	x2, y2 = target

	return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)


def closest_point(V, target):

	min_distance = float("Inf")
	closest_point = None
	for v in V:
		cur_distance = distance(v.getxy(), target.getxy())
		if cur_distance < min_distance:
			min_distance = cur_distance
			closest_point = v


	return closest_point




