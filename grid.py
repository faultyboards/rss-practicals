#!/usr/bin/env python
import sys
import numpy as np
import pdb
'''
This module is meant for generating the arena grid as well as calculating
the priors P(sensor|position) for each occupiable field in the grid.

It leaves the grid file as a pickle 
'''

class Grid():
	# Set the size of grid cells and the number of allowed orientations
	cell_size = 0.07
	angle_res = 12
	# Set resolution for sensor readings simulation
	simulation_res = 2
	# Set the number of rays used when simulating sensor readings 
	raytracing_res = 10

	# Specify arena data
	arena_size = np.array([3.2, 4.25])
	arena_size_idx = np.ceil(arena_size / cell_size)
	arena_size_idx = (int(arena_size_idx[0]), int(arena_size_idx[1]))
	# Write equations of walls - a set of constraints making a point (x,y)
	# lie OUTSIDE of available spate. 
	not_in_arena = ['x < 0 or x > 3.2 or y < 0 or y > 4.25', # external boundaries of the arena
					'x < 1.05 and y < 1.05', # the concave corner
					'y < x - 2.9', # small cut corner
					'y > x + 3.65', # large cut corner
					'x > 1 and y > 0.5 * x + 1.9 and y < -0.5 * x + 3.65', # the triangle island
					'x > 2.29 and x < 2.45 and y < 3.2 and y > 1.05']

	start_point = np.array([1.40, 0.41])
	satellite_pos = np.array([0.65, 0.35, 2.95])

	def __init__(self):
		self.in_arena = np.empty(Grid.arena_size_idx, dtype=np.uint)
		# Check if centre of each grid cell is in the arena
		it = np.nditer(self.in_arena, flags=['multi_index'], op_flags=['writeonly'])
		while not it.finished:
			it[0] = Grid.is_in_arena(it.multi_index[0], it.multi_index[1])
			it.iternext()

	@staticmethod
	def is_in_arena(x_idx, y_idx):
		'''
		Evaluates if the centre of a cell of the given index lies within the arena
		'''
		(x, y) = Grid.grid_to_xy((x_idx, y_idx))
		for excluding_condition in Grid.not_in_arena:
			if eval(excluding_condition):
				return False
		return True

	@staticmethod
	def grid_to_xy(grid_coord):
		'''
		Gives the (x,y) coordinates of the centre of a grid cell indexed by grid_coord.
		'''
		return (np.array(grid_coord)*Grid.cell_size)+np.array([Grid.cell_size/2, Grid.cell_size/2])

	@staticmethod
	def xy_to_grid(xy_coord):
		'''
		Returns the index of the grid cell the point (x,y) lies within.
		'''
		val = np.floor(xy_coord / Grid.cell_size)
		return (val[0], val[1])

	def __str__(self):
		'''defined
		Displays a human-readable representatnion of the arena
		'''
		retval = ''
		for x in reversed(range(self.in_arena.shape[0])):
			for y in reversed(range(self.in_arena.shape[1])):
				# if Grid.start_point.all(np.array(Grid.xy_to_grid(np.array((x,y))))):
				# 	retval += 'S '
				# else:
				# 	retval += '  ' if self.in_arena[x,y] else '# '
				retval += '  ' if self.in_arena[x,y] else '# '
			retval += '\r\n'
		return retval

# Executed when the module is called directly
if __name__ == "__main__":
	print(Grid.xy_to_grid(np.array([0.01,0.19])))
	print(Grid.grid_to_xy(Grid.xy_to_grid(np.array([0.1,0.19]))))
	g = Grid()
	print(g)
	print('Preparing a gird of priors!')