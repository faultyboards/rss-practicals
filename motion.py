import time
import numpy as np

from sensors import Sensors
from motors import Motors

class Motion():
	'''
	This class is a high-level motion controller dealing with odometry, navigation, etc.
	'''
	def __init__(IO, sensors):
		# Multiplier dealing with how the state of the battery affects distance-travelled
		# estimates
		self.avg_speed = 1
		self.avg_speed_last_update = None
		self.IO = IO
		self.sensors = Sensors(self.IO)
		self.motors = Motors(self.IO)

		self.hall_trig_dist = 1
		self.hall_timer = None
		self.hall_count = 0
		self.hall_reading_prev

		self.angle_time_multiplier = 1

	def _hall_handler():
		'''
		Called each time step to check what's up with teh hall sensor.
		Deals with updating the battery_state estimation which then in turn is used to estimate
		distance travelled.
		'''
		self.sensors.update_readings(type='digital')
		hall_reading = self.sensors.get_hall()
		if not hall_reading and self.hall_reading_prev:
			self.hall_count += 1
			if self.hall_count > 1:
				self.avg_speed = self.hall_trig_dist / (time.time() - self.hall_timer)
			else:
				
			self.hall_timer = time.time()

			self.hall_reading_prev = hall_reading

	def _hall_handler_reset():
		self.hall_trig_dist = 1
		self.hall_timer = None
		self.hall_count = 0
		self.sensors.update_readings(type='digital')
		self.hall_reading_prev = self.sensors.get_hall()

	def move(amount, amount_type='distance'):
		'''
		Moves the robot by the specified amount. By default the amount is distance in meteres.
		Set amount_type to 'time' for the amount to be time of motion in seconds, or 'callback'
		for amount to be a reference to a callback function to be evaulated periodically (True
		result stops the motion).
		Negative values move the robot backwards.
		Returns estimated distance travelled in meters.
		'''
		motion_complete = False
		start_time = time.time()
		amount_travelled = 0
		self.hall_count = 0
		hall_reading_prev = False
		hall_timer = None

		if amount_type = 'radians':
			max_time = amount * self.angle_time_multiplier * self.avg_speed
		elif amount_type = 'degrees':
			max_time = amount * self.angle_time_multiplier * self.avg_speed * np.pi / 180
		elif 'time':
			max_time = time

		self.motors.full_on('right' if amount > 0 else 'left')

		while time.time() - start_time < max_time:

			amount_travelled = (time.time() - start_time) / (self.angle_time_multiplier * self.avg_speed)

		return amount_travelled


	def turn(amount, amount_type='radians'):
		'''
		Truns the robot by the specified amount. By default the amount is an angle in radians.
		Set amount_type to 'degs' for imput in degrees, 'time' for the amount to be time of
		rotation in seconds, or 'callback' for amount to be a reference to a callback function
		to be evaulated periodically (True result stops the motion). - CALLBACK NOT WORKING YET
		Positive values turn the robot clockwise and negative turn it anticlockwise.
		Returns estimated angle turned in radians.
		'''
		motion_complete = False
		start_time = time.time()
		amount_travelled = 0
		if amount_type = 'radians':
			max_time = amount * self.angle_time_multiplier * self.battery_state
		elif amount_type = 'degrees':
			max_time = amount * self.angle_time_multiplier * self.battery_state * np.pi / 180
		elif 'time':
			max_time = time

		self.motors.full_on('right' if amount > 0 else 'left')

		while time.time() - start_time < max_time:
			amount_travelled = (time.time() - start_time) / (self.angle_time_multiplier * self.battery_state)

		return amount_travelled