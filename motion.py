import time
import numpy as np

from sensors import Sensors
from motors import Motors

SATELLITE_POS = [-0.69, 0, 2.95]
ANTENNA_POS = [-0.04, -0.09, 0.25]
START_POS = [0., 0., 0.]


def find_sat_angls(robot_pos, robot_ornt, sat_pos):
	p = np.array(robot_pos) + np.array(ANTENNA_POSITION)
	s = np.array(sat_pos)
	r = s - p
	r_f = np.concatenate([r[:2], np.zeros(1)])

	ant_angl = (np.pi / 2 - np.arccos(np.dot(r,
				np.array([0, 0, 1])) / np.linalg.norm(r))) % (2 * np.pi)
	bot_turn = ((1 if r_f[1] < 0 else -1) * np.arccos(np.dot(r_f, np.array([1, 0, 0])) / np.linalg.norm(r_f)) +
					np.pi / 2 -
					robot_ornt) % (2 * np.pi)

	if bot_turn < -np.pi / 2:
		bot_turn += np.pi
	elif bot_turn > np.pi / 2:
		bot_turn -= np.pi
	else:
		ant_angl = np.pi - ant_angl

	return bot_turn, ant_angl


class Motion():
	'''
	This class is a high-level motion controller dealing with odometry, navigation, etc.
	'''

	def __init__(self, IO):
		# Multiplier dealing with how the state of the battery affects distance-travelled
		# estimates

		self.avg_speed = 0.0825  # initial value assumes average / full battery
		self.IO = IO
		self.sensors = Sensors(self.IO)
		self.motors = Motors(self.IO)
		self.motors.enable_servo()

		self.hall_trig_dist = 0.135675
		self.hall_timer = None
		self.hall_count = 0
		self.last_hall_reading_time = None
		self.hall_reading_prev = False

		self.angle_time_multiplier = 0.18465

		self.wall_distance_control_mul = 1

		self.servo_engaged = False

	def _hall_handler(self):
		'''
		Called each time step to check what's up with teh hall sensor.
		Deals with updating the battery_state estimation which then in turn is used to estimate
		distance travelled.
		'''
		self.sensors.update_readings(type='digital')
		hall_reading = self.sensors.get_hall()
		time_now = time.time()
		# print(hall_reading)
		if not hall_reading and self.hall_reading_prev:
			self.hall_count += 1
			if self.hall_count > 1:
				new_avg_speed = self.hall_trig_dist / (time_now - self.hall_timer)
				print('new_avg_speed: {}'.format(new_avg_speed))
				if (np.abs(new_avg_speed - self.avg_speed) > 0.2 * self.avg_speed):
					print("Warning! Average speed estimate changed by more than 20%! (Hall sensor problem?)")
				self.avg_speed = new_avg_speed
			self.last_hall_reading_time = time_now

			self.hall_timer = time_now
			print('Hall fall - {}'.format(self.hall_count))

		self.hall_reading_prev = hall_reading

	def _hall_handler_reset(self):
		self.hall_timer = None
		self.hall_count = 0
		self.sensors.update_readings(type='digital')
		self.hall_reading_prev = self.sensors.get_hall()

	def move(self, amount, amount_type='distance', wall_following=None):
		'''
		Moves the robot by the specified amount. By default the amount is distance in meteres.
		Set amount_type to 'time' for the amount to be time of motion in seconds, or 'callback'
		for amount to be a reference to a callback function to be evaulated periodically (True
		result stops the motion).

		If wall_following is set to a number the robot will actively try to keep a given distance
		from a wall on its left (negative values of 'wall_following') or right (positive values for
		'wall_following'). 'wall_following' should be in meters.

		Negative values move the robot backwards.
		Returns estimated distance travelled in meters.
		'''
		self._hall_handler_reset()
		start_time = time.time()
		amount_travelled = 0
		travel_time_pre_hall = 0
		travel_time_since_hall = 0

		self.motors.full_on('forward' if amount > 0 else 'backward')

		condition = True
		while condition:
			self._hall_handler()
			time_now = time.time()

			if self.hall_count == 0:
				travel_time_pre_hall = (time_now - start_time)
			else:
				travel_time_since_hall = (time_now - self.last_hall_reading_time)

			amount_travelled = travel_time_pre_hall * self.avg_speed + \
							   (self.hall_trig_dist * ((self.hall_count - 1) if self.hall_count > 0 else 0) ) + \
							   travel_time_since_hall * self.avg_speed

			if amount_type == 'distance':
				condition = amount_travelled <= np.abs(amount)
			elif amount_type == 'callback':
				condition, reason = amount(self.sensors, amount_travelled)
			else:
				condition = np.abs(amount) >= (time_now - start_time)

			if wall_following is not None:
				wall_dist = self.sensors.get_ir['right' if wall_following >=0 else 'left']
				error_percent = np.abs(wall_following - wall_dist) / wall_following
				print(error_percent)

				self.motors.apply_direction_skew(error_percent)

		self.motors.stop()
		if amount_type == 'callback':
			return amount_travelled, reason
		else:
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
		if amount_type == 'radians':
			max_time = np.abs(amount) * self.angle_time_multiplier / self.avg_speed
		elif amount_type == 'degrees':
			max_time = np.abs(amount) * self.angle_time_multiplier / \
							  self.avg_speed * np.pi / 180
		elif 'time':
			max_time = time

		self.motors.full_on('right' if amount > 0 else 'left')

		condition = True
		while condition:
			amount_travelled = (time.time() - start_time) * \
								self.avg_speed / self.angle_time_multiplier

			if amount_type == 'callback':
				condition, reason = amount(self.sensors, amount_travelled)
			else:
				condition = time.time() - start_time < max_time

		self.motors.stop()
		if amount_type == 'callback':
			return amount_travelled, reason
		else:
			return amount_travelled

	def move_along_wall(self, amount, wall_distance, amount_type='distance'):
		''' 
		Like self.move() but tries to keep a constant distance from a wall on its left (negative
		values of 'amount') or right (positive values for 'amount').
		'''




	def set_antenna(self, angle, degree_type='radians'):
		if not self.servo_engaged:
			self.motors.enable_servo()
			self.servo_engaged = True
			
		if degree_type == 'radians':
			angle *= 180 / np.pi
		elif degree_type == 'degrees':
			pass

		self.motors.set_servo(angle)
