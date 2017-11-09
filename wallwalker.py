def mtn_front_bumper_stop_cb_generator(travel_allowed,
                                       left_feature_thresh=None):
    def mtn_front_bumper_stop_cb_(sensors, amount_travelled):
        '''
        The callback used in stopping motion.
        '''
        if sensors.get_whisker():
            return False, 'whisker'
        elif amount_travelled >= travel_allowed:
            return False, 'distance'
        elif (left_feature_thresh is not None and
              sensors.get_ir('left') < left_feature_thresh):
            return False, 'ir'
        else:
            return True, None


def mtn_simple_bumper(sensors, amount_travelled):
    '''
    Callback which stops only when the front bumper is bumped.
    '''
    if sensors.get_whisker():
        return False, None
    else:
        return True, None


class Wallwalker():
    '''
    The class dealing with walking around the 1D reduction of the arena.
    '''

    def __init__(self, sensors, motion):
        self.sensors = sensors
        self.motion = motion

        self.grid_res = 0.1
        self.seg_f_n = 'segment_data.pickle'
        self.current_segment = 0

        self.segments_no = 5
        self.seg_transition_due = False
        self.distance_along = 0

        self.motion_callback = None

    def step(self, step_length=None):
        '''
        Progress the robot along the pre-defined wall-following path by a given
        distance (or until the current segment finishes). If no step length is
        defined the robot shall progress by a step length defined for that
        segment.
        '''
        if not self.seg_transition_due:
            seg_fun = eval('self._step_{}'.format(self.current_segment))
            seg_fun(step_length)
        else:
            seg_trans_fun = eval(
                'self._transition_{}'.format(self.current_segment))
            seg_trans_fun(step_length)

    def _step_0(self, step_length=None):
        '''
        Step action for segment 0.
        '''
        segment_length = 1
        segment_step_size = 0.2
        distance_from_wall = 0.5

        left_wall_dist_along = 0.89
        left_wall_dist_thresh = 1

        mtn_cb = mtn_front_bumper_stop_cb_generator(segment_step_size,
                                                    left_wall_dist_thresh)

        distance_travelled, reason = self.motion.move(mtn_cb,
                                                      'callback',
                                                      distance_from_wall)

        self.distance_along += distance_travelled

        if reason == 'whisker' or self.distance_along >= segment_length:
            self.seg_transition_due = True
        elif reason == 'ir':
            self.distance_along = left_wall_dist_along

    def _step_1(self, step_length=None):
        '''
        Step action for segment 1.
        '''
        pass

    def _step_2(self, step_length=None):
        '''
        Step action for segment 2.
        '''
        pass

    def _step_3(self, step_length=None):
        '''
        Step action for segment 3.
        '''
        pass

    def _step_4(self, step_length=None):
        '''
        Step action for segment 4.
        '''
        pass

    def _transition_0(self, step_length=None):
        '''
        Transition action for segment 0 to the next one.
        '''
        backward_move = - (0.75 / 2 - 0.13)

        # Hit the wall
        if not self.sensors.get_whisker():
            self.motion.move(mtn_simple_bumper, 'callback')

        # Back off
        self.motion(backward_move)

        self.turn(-90, 'degrees')

    def _transition_1(self, step_length=None):
        '''
        Transition action for segment 1 to the next one.
        '''
        pass

    def _transition_2(self, step_length=None):
        '''
        Transition action for segment 2 to the next one.
        '''
        pass

    def _transition_3(self, step_length=None):
        '''
        Transition action for segment 3 to the next one.
        '''
        pass

    def _transition_4(self, step_length=None):
        '''
        Transition action for segment 4 to the next one.
        '''
        pass

    def get_xy_th_estimate(self):
        '''
        Returns the estimate of the position of the robot in terms of (x,y)
        coordinates and the orientation theta (in radians where the x axis to
        the right is zero angle)
        '''
        if self.current_segment == 0:
            x = 0.35 + self.distance_along
            y = 0.41
            th = 0
        elif self.current_segment == 1:
            x = 1.43
            y = 0.35 + self.distance_along
            th = 0
        elif self.current_segment == 2:
            x = 41
            y = 0.35 + self.distance_along
            th = 0
        elif self.current_segment == 3:
            x = 41
            y = 0.35 + self.distance_along
            th = 0
        elif self.current_segment == 4:
            x = 41
            y = 0.35 + self.distance_along
            th = 0

        return (x, y, th)


if __name__ == "__main__":
    # TODO Segment prior initialization code
    pass
