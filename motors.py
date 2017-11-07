import time


class Motors:
    def __init__(self, IO):
        self.IO = IO
        self.state = {'DC': {}, 'servo': {}}
        self.state['DC']['initial'] = (0, 0)
        self.state['servo']['initial'] = 0
        self.state['DC']['current'] = self.state['DC']['initial']
        self.state['servo']['current'] = self.state['servo']['initial']
        self.state['servo']['engaged'] = False
        self.state['DC']['direction_skew'] = 0

        # Setup constants
        self.motor_pwr_multiplier = (0.9, -0.9)
        self.direction_skew_multiplier = (0.1, -0.1)

        self.servo_seconds_per_degree = 0.02
        self.last_servo_change = None

        self.motions = {'forward': (100, 100), 'backward': (-100, -100),
                        'right': (-100, 100), 'left': (100, -100)}

    def apply_direction_skew(self, dir_skew):
        '''
        Skews the direction of motion in faviour of turning more right (positive skew) or left
        (negative skew).
        '''
        if dir_skew > 1:
            dir_skew = 1
        elif dir_skew < -1:
            dir_skew = -1
        self.state['DC']['direction_skew'] = dir_skew
        

    def apply_mult(self, motor_setting):
        return (self.motor_pwr_multiplier[0] * motor_setting[0] + \
                self.state['DC']['direction_skew'] * self.direction_skew_multiplier[0], \
                self.motor_pwr_multiplier[1] * motor_setting[1] + \
                self.state['DC']['direction_skew'] * self.direction_skew_multiplier[1])

    def full_on(self, motion='forward', how_long=None):
        fullon = self.apply_mult(self.motions[motion])
        old_state = self.state['DC']['current']
        self.state['DC']['current'] = fullon
        if old_state != self.state['DC']['current']:
            self.IO.setMotors(fullon[0], fullon[1])
            if how_long is not None:
                time.sleep(how_long)
                self.state['DC']['current'] = old_state
                self.IO.setMotors(old_state[0], old_state[1])

    def stop(self):
        self.IO.setMotors(1, 1)
        self.apply_direction_skew(0)
        time.sleep(1)
        self.state['DC']['current'] = (0, 0)

    def enable_servo(self):
        self.IO.servoEngage()
        self.state['servo']['engaged'] = True
        time.sleep(0.5)
        self.set_servo(self.state['servo']['initial'])
        time.sleep(2)
        print('Servo Engaged!')

    def set_servo(self, angle):
        if self.state['servo']['engaged']:
            self.IO.servoSet(angle)
            # block for a time proportional to the angle we need to turn
            time.sleep(abs(angle - self.state['servo']['current']) * self.servo_seconds_per_degree)
            self.state['servo']['current'] = angle
        else:
            raise Exception('Servo was not engaged!')
