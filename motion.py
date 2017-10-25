import time
import numpy as np

from sensors import Sensors
from motors import Motors

class Motion():
	'''
	This class is a high-level motion controller dealing with odometry, navigation, etc.
	'''
	def __init__(self, IO):
		# Multiplier dealing with how the state of the battery affects distance-travelled
		# estimates
		self.avg_speed = 1
		self.IO = IO
		self.sensors = Sensors(self.IO)
		self.motors = Motors(self.IO)

		self.hall_trig_dist = 0.125
		self.hall_timer = None
		self.hall_count = 0
		self.last_hall_reading_time = None
		self.hall_reading_prev = False

		self.angle_time_multiplier = 1 # Needs to be callibrated

		self.i = 0

	def _hall_handler(self):
		'''
		Called each time step to check what's up with teh hall sensor.
		Deals with updating the battery_state estimation which then in turn is used to estimate
		distance travelled.
		'''
		self.sensors.update_readings(type='digital')
		hall_reading = self.sensors.get_hall()
		time_now = time.time()
		if not hall_reading and self.hall_reading_prev:
			self.hall_count += 1
			self.i += 1
			if self.hall_count > 1:
				new_avg_speed = self.hall_trig_dist / (time_now - self.hall_timer)
				if (np.abs(new_avg_speed-self.avg_speed) > 0.1 * self.avg_speed):
					raise Warning("Average speed estimate changed by more than 10%! (Hall sensor problem?)")
				self.avg_speed = new_avg_speed
			self.last_hall_reading_time = time_now
				
			self.hall_timer = time_now

		self.hall_reading_prev = hall_reading

	def _hall_handler_reset(self):
		self.hall_trig_dist = 1
		self.hall_timer = None
		self.hall_count = 0
		self.sensors.update_readings(type='digital')
		self.hall_reading_prev = self.sensors.get_hall()

	def move(self, amount, amount_type='distance'):
		'''
		Moves the robot by the specified amount. By default the amount is distance in meteres.
		Set amount_type to 'time' for the amount to be time of motion in seconds, or 'callback'
		for amount to be a reference to a callback function to be evaulated periodically (True
		result stops the motion).
		Negative values move the robot backwards.
		Returns estimated distance travelled in meters.
		'''
		self._hall_handler_reset()
		start_time = time.time()
		amount_travelled = 0
		travel_pre_hall = 0
		travel_since_hall = 0

		if amount_type == 'distance':
			max_time = np.abs(amount) / self.avg_speed
		elif 'time':
			max_time = time.time()

		self.motors.full_on('forward' if amount > 0 else 'backwards')

		condition = True
		while condition:
			self._hall_handler()
			time_now = time_now

			if self.hall_count == 0:
				travel_pre_hall += (time_now - start_time) * self.avg_speed
			else:
				travel_since_hall = (time_now - self.last_hall_reading_time) * self.avg_speed 

			amount_travelled = travel_pre_hall + (self.hall_trig_dist * self.hall_count) + travel_since_hall
			
			if amount_type == 'distance':
				condition = amount_travelled <= amount
			else:
				condition = amount >= (time_now - start_time)

		self.motors.stop()
		return amount_travelled


	def turn(self, amount, amount_type='radians'):
		'''
		Truns the robot by the specified amount. By default the amount is an angle in radians.
		Set amount_type to 'degs' for imput in degrees, 'time' for the amount to be time of
		rotation in seconds, or 'callback' for amount to be a reference to a callback function
		to be evaulated periodically (True result stops the motion). - CALLBACK NOT WORKING YET
		Positive values turn the robot clockwise and negative turn it anticlockwise.
		Returns estimated angle turned in radians.
		'''
		start_time = time.time()
		amount_travelled = 0
		print('{}\t{}\t{}'.format(np.abs(amount), self.angle_time_multiplier, self.avg_speed))
		if amount_type == 'radians':
			max_time = np.abs(amount) * self.angle_time_multiplier / self.avg_speed
		elif amount_type == 'degrees':
			max_time = np.abs(amount) * self.angle_time_multiplier / self.avg_speed * np.pi / 180
		elif 'time':
			max_time = time

		self.motors.full_on('right' if amount > 0 else 'left')

		while time.time() - start_time < max_time:
			print('{}\t{}'.format(time.time() - start_time, max_time))
			amount_travelled = (time.time() - start_time) * self.avg_speed / self.angle_time_multiplier

		self.motors.stop()
		return amount_travelled