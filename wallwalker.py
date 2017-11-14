import numpy as np


def mtn_front_bumper_stop_cb_generator(travel_allowed,
                                       left_feature_thresh=None,
                                       threshold_dir='lower'):
    def mtn_front_bumper_stop_cb(sensors, amount_travelled):
        '''
        The callback used in stopping motion.
        '''
        epsilon = 0.05
        if sensors.get_whisker():
            return False, 'whisker'
        elif amount_travelled >= travel_allowed:
            return False, 'distance'
        elif (left_feature_thresh is not None and
              threshold_dir == 'lower' and
              sensors.get_ir('left') < left_feature_thresh - epsilon):
            return False, 'ir_lower'
        elif (left_feature_thresh is not None and
              threshold_dir == 'higher' and
              sensors.get_ir('left') >= left_feature_thresh + epsilon):
            return False, 'ir_higher'
        else:
            return True, None
    return mtn_front_bumper_stop_cb


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

        self.misc_state = None

        self.motion_callback = None

        self.robot_half_lenght = 0.14  # TODO
        self.robot_half_width = 0.13  # TODO

        self._segment_info = {0: (2.15 - 0.2 - 0.3 / 2 - 0.65 / 2,
                                  0.2,
                                  1.05 / 2 - self.robot_half_width,
                                  2.15 - 0.75 - 0.16 - 0.2 - 0.3 / 2,
                                  2.15 - 0.75 - 0.2 - 0.3 / 2,
                                  1.,
                                  0,
                                  0.75 / 2 - self.robot_half_lenght),
                              1: (4.25 - 1.05 / 2 - 1.05 / 2,
                                  0.2,
                                  0.75 / 2 - self.robot_half_width,
                                  1.05 / 2,
                                  1.05 / 2 + 2.15,
                                  1.,
                                  0,
                                  1.05 / 2 - self.robot_half_lenght),
                              2: (3.20 - 0.7 - 0.75 / 2,
                                  0.2,
                                  1.05 / 2 - self.robot_half_width,
                                  2.25 - 0.75 / np.sqrt(3),
                                  2.25,
                                  2.,
                                  1.,
                                  0.05),
                              3: (2.6 - 0.4,
                                  0.2,
                                  1. / 2 - self.robot_half_width,
                                  1.05 / 2,
                                  1.05 / 2 + 0.75,
                                  1.,
                                  1.,
                                  0.3),
                              3: (1.5,
                                  0.2,
                                  None,
                                  None,
                                  None,
                                  None,
                                  None,
                                  None)}

    def step(self, step_length=None):
        '''
        Progress the robot along the pre-defined wall-following path by a given
        distance (or until the current segment finishes). If no step length is
        defined the robot shall progress by a step length defined for that
        segment.
        '''
        print('Seg: {}\tdistance along: {}\ttransition due: {}'.format(
            self.current_segment,
            self.distance_along,
            self.seg_transition_due))
        if self.seg_transition_due:
            retval = self.generic_transition()
        else:
            retval = self.generic_step_with_left_hook()
        return retval

    def unstep(self, step_length=None):
        '''
        Motions back by a step amount appropirate for a given segment.
        '''
        _, segment_step_size, _, _, _, _, _, _ = self._segment_info[
            self.current_segment]
        distance_undone = self.motion.move(-segment_step_size)
        self.distance_along -= distance_undone

    def generic_step_with_left_hook(self):
        if self.current_segment >= len(self._segment_info):
            print('no more steps')
            return False

        segment_length, \
            segment_step_size, \
            distance_from_wall, \
            left_obst_start_along, \
            left_obst_finish_along, \
            left_obst_dist_thresh, \
            left_obst_along_thresh, \
            transtion_backoff = self._segment_info[
                self.current_segment]

        # In this step misc_state means if we are already in the corridor
        if self.misc_state is None:
            self.misc_state = False
        print(segment_step_size)
        if self.distance_along < left_obst_along_thresh:
            # Ignore left ir until we exit the corridor
            mtn_cb = mtn_front_bumper_stop_cb_generator(segment_step_size)
        elif self.misc_state:
            mtn_cb = mtn_front_bumper_stop_cb_generator(segment_step_size,
                                                        left_obst_dist_thresh,
                                                        threshold_dir='lower')
        else:
            mtn_cb = mtn_front_bumper_stop_cb_generator(segment_step_size,
                                                        left_obst_dist_thresh,
                                                        threshold_dir='higher')

        distance_travelled, reason = self.motion.move(mtn_cb,
                                                      'callback',
                                                      distance_from_wall)

        self.distance_along += distance_travelled

        if reason == 'whisker' or self.distance_along >= segment_length:
            self.seg_transition_due = True
        elif reason == 'ir_lower':
            self.distance_along = left_obst_start_along
            self.misc_state = True
        elif reason == 'ir_higher':
            self.distance_along = left_obst_finish_along
            self.misc_state = False

        return distance_travelled

    def generic_transition(self):
        _, _, _, _, _, _, _, transtion_backoff = self._segment_info[
            self.current_segment]
        if transtion_backoff is not None:
            self._transition_bump_n_turn(transtion_backoff)
            return True
        else:
            return False

    def _transition_bump_n_turn(self, backward_move, next_segment=None):
        backward_move = -backward_move

        # Hit the wall
        if not self.sensors.get_whisker():
            self.motion.move(mtn_simple_bumper, 'callback')

        # Back off
        self.motion(backward_move)

        self.turn(-90, 'degrees')

        if next_segment is None:
            self.current_segment += 1
        else:
            self.current_segment = next_segment
        self.distance_along = 0

    def get_xy_th_estimate(self):
        '''
        Returns the estimate of the position of the robot in terms of (x,y)
        coordinates and the orientation theta (in radians where the x axis to
        the right is zero angle)
        '''
        if self.current_segment == 0:
            x = self.distance_along
            y = 0.41
            th = 0
        elif self.current_segment == 1:
            x = 2.15 - 0.2 - 0.3 / 2 - 0.75 / 2
            y = 1.05 / 2 + self.distance_along
            th = -np.pi / 2
        elif self.current_segment == 2:
            x = 2.15 - 0.2 - 0.3 / 2 - 0.75 / 2 - self.distance_along
            y = 4.25 - 0.2 - 0.42 / 2 - 1.05 / 2
            th = np.pi
        elif self.current_segment == 3:
            x = -(0.3 / 2 + 0.2 + 0.5 + 1. / 2)
            y = 4.25 - 0.2 - 0.42 / 2 - 1.05 / 2 - self.distance_along
            th = np.pi / 2
        elif self.current_segment == 4:
            x = self.distance_along - (0.3 / 2 + 0.2 + 0.5 + 1. / 2)
            y = self._segment_info[3][0] + 1.05 - 0.2 - 0.42 / 2
            th = 0
        return np.array([x, y, th])

    def get_step_size(self):
        '''
        Returns the current target wall distance.
        '''
        return self._segment_info[self.current_segment][1]

    def get_targ_wall_dist(self):
        '''
        Returns the current target wall distance.
        '''
        return self._segment_info[self.current_segment][2]

if __name__ == "__main__":
    # TODO Segment prior initialization code
    pass
