import time

class Motors:
	def __init__(self, IO):
		self.IO = IO
		self.initial_state = (0, 0)
		self.current_state = self.initial_state

		# Setup constants
		self.motor_pwr_multiplier = (1, 1)

	def apply_mult(self, motor_setting):
		return (self.motor_pwr_multiplier[0]*motor_setting[0], \
			self.motor_pwr_multiplier[1]*motor_setting[1])

	def full_forward(self, how_long=None):
		fullon = self.apply_mult((100, 100))
		self.IO.setMotors(fullon)
		old_state = self.current_state
		self.current_state = fullon
		if how_long is not None:
			time.sleep(how_long)
			self.IO.setMotors(old_state)
			self.current_state = old_state

