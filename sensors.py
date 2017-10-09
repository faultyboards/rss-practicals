class Sensors:
	def __init__(self, IO):
		self.IO = IO

		#  define ports for sensors
		self.port = {}
		# digital sensors
		self.port['whisker'] = {'left': 0, 'right': 1}
		self.port['switch'] = [2, 3, 4, 5]
		self.port['hall'] = [6]

		# analogue sensors
		self.port['sonar'] = 0
		self.port['light'] = [1, 2, 3, 4]
		self.port['ir'] = [5, 6]

		# define thresholds / multipliers for analogue sensors
		self.whskr_on_state = {'left': 1, 'right': 1}
		self.light_threshold = {'black': 0, 'reflective': 250}	# TODO provide actual values
		self.ir_dist_mltpl = 128 # TODO provide actual values
		self.sonar_dist_mltpl = 128 # TODO provide actual values

		# for storing most recent sensor readings
		self.analogue_readings = self.IO.getSensors()
		self.digital_readings = self.IO.getInputs()

	def update_readings(self, type='both'):
		if type == 'analogue' or type == 'both':
			self.analogue_readings = self.IO.getSensors()
		if type == 'digital' or type == 'both':
			self.digital_readings = self.IO.getInputs()

	def get_whisker(self, side='both'):
		if side == 'both':
			return self.digital_readings[self.port['whisker']['left']] or self.digital_readings[self.port['whisker']['right']]
		else:
			return self.digital_readings[self.port['whisker'][side]]

	def get_switch(self, switch_no='either'):
		if switch_no == 'either':
			return sum([self.digital_readings[x] for x in self.port['switch']])
		else:
			return self.digital_readings[self.port['switch'][switch_no]]

	def get_sonar(self, raw=False):
		if not raw:
			return self.analogue_readings[self.port['sonar']]*self.sonar_dist_mltpl
		else:
			return self.analogue_readings[self.port['sonar']]

	def get_ir(self, sensor_no, raw=False):
		if not raw:
			return self.analogue_readings[self.port['ir'][sensor_no]]*self.ir_dist_mltpl
		else:
			return self.analogue_readings[self.port['ir'][sensor_no]]

	def get_light(self, sensor_no, kind=False):
		if kind:
			return self.analogue_readings[self.port['light'][sensor_no]] < self.light_threshold[kind]
		else:
			return self.analogue_readings[self.port['light'][sensor_no]]


