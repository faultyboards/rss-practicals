import time

from sensors import Sensors
from motors import Motors

class Motion():
	'''
	This class is a high-level motion controller dealing with odometry, navigation, etc.
	'''
	def __init__(IO, sensors):
		# Multiplier dealing with how the state of the battery affects distance-travelled
		# estimates
		self.battery_state = 1
		self.battery_state_last_update = None
		self.IO = IO
		self.sensors = Sensors(self.IO)

	def _hall_triggered():
		'''
		Called each time a hall sensor is triggered while moving forward.
		Deals with updating the battery_state estimation which then in turn is used to estimate
		distance travelled.
		'''

	def move(amount, amount_type='distance'):
		'''
		Moves the robot by the specified amount. By default the amount is distance in meteres.
		Set amount_type to 'time' for the amount to be time of motion in seconds, or 'callback'
		for amount to be a reference to a callback function to be evaulated periodically (True
		result stops the motion).
		Negative values move the robot backwards.
		Returns estimated distance travelled in meters.
		'''

	def turn(amount, amount_type='radians'):
		'''
		Truns the robot by the specified amount. By default the amount is an angle in radians.
		Set amount_type to 'degs' for imput in degrees, 'time' for the amount to be time of
		rotation in seconds, or 'callback' for amount to be a reference to a callback function
		to be evaulated periodically (True result stops the motion).
		Positive values turn the robot clockwise and negative turn it anticlockwise.
		Returns estimated angle turned in radians.
		'''
