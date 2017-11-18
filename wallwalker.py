import numpy as np


def mtn_front_bumper_stop_cb_generator(travel_allowed,
                                       poi_detection_disabled=True):
    def mtn_front_bumper_stop_cb(sensors, amount_travelled):
        '''
        The callback used in stopping motion.
        '''
        epsilon = 0.05
        sensors.update_readings(type='both')
        poi_sensors = sensors.get_poi_sensors()

        if poi_sensors:
            for sensor in poi_sensors:
                print('{}:{}'.format(sensor, sensors.get_light(sensor, raw=True)))

        if sensors.get_whisker():
            return False, 'whisker'
        elif not poi_detection_disabled and poi_sensors:
            if 'front' in poi_sensors:
                reason = 'poi_front'
                if 'left' in poi_sensors:
                    reason += '_left'
                if 'right' in poi_sensors:
                    reason += '_right'
            elif 'left' in poi_sensors:
                reason = 'poi_left'
            elif 'right' in poi_sensors:
                reason = 'poi_right'
            return False, reason
        elif amount_travelled >= travel_allowed:
            return False, 'distance'
        else:
            return True, None
    return mtn_front_bumper_stop_cb


def mtn_simple_bumper(sensors, amount_travelled):
    '''
    Callback which stops only when the front bumper is bumped.
    '''
    sensors.update_readings(type='digital')
    if sensors.get_whisker():
        return False, None
    else:
        return True, None


class Wallwalker():
    '''
    The class dealing with walking around the 1D reduction of the arena.
    '''

    def __init__(self, parent):
        self.sensors = parent.sensors
        self.motion = parent.motion
        self.parent = parent
        self.grid_res = 0.1
        self.seg_f_n = 'segment_data.pickle'
        self.current_segment = 0

        self.segments_no = 5
        self.seg_transition_due = False
        self.distance_along = 0

        self.misc_state = None

        self.motion_callback = None

        self.robot_half_lenght = 0.14  # TODO
        self.robot_half_width = 0.12  # TODO

        self.corr_width_1 = 0.98
        self.corr_width_2 = 0.8
        self.corr_width_3 = 1.07
        self.corr_width_4 = 1.03
        self.corr_width_1 = 0.9

        self.poi_detected = False

        self._segment_info = {0: (2.15 - 0.2 - 0.3 / 2 - self.corr_width_2 / 2,
                                  0.35,
                                  self.corr_width_1 / 2 - self.robot_half_width,
                                  2.15 - 0.75 - 0.16 - 0.2 - 0.3 / 2,
                                  2.15 - 0.75 - 0.2 - 0.3 / 2,
                                  1.,
                                  0,
                                  0.75 / 2 - self.robot_half_lenght),
                              1: (4.25 - self.corr_width_1 / 2 - self.corr_width_3 / 2,
                                  0.35,
                                  self.corr_width_2 / 2 - self.robot_half_width,
                                  1.05 / 2,
                                  1.05 / 2 + 2.15,
                                  1.,
                                  0,
                                  1.05 / 2 - self.robot_half_lenght),
                              2: (3.20 - 0.7 - self.corr_width_2 / 2,
                                  0.35,
                                  self.corr_width_3 / 2 - self.robot_half_width + 0.15,
                                  2.25 - 0.75 / np.sqrt(3),
                                  2.25,
                                  2.,
                                  1.,
                                  0.05),
                              3: (2.6 - 0.4,
                                  0.35,
                                  self.corr_width_4 / 2 - self.robot_half_width + 0.1,
                                  1.05 / 2,
                                  1.05 / 2 + 0.75,
                                  1.,
                                  1.,
                                  0.3),
                              4: (1.5,
                                  0.35,
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
        print('Seg: {}\tdistance along: {}/{}\ttransition due: {}'.format(
            self.current_segment,
            self.distance_along,
            self.get_segment_len(),
            self.seg_transition_due))
        # print('light: f:{}\tr:{}\tl:{}\tr:{}'.format(self.sensors.get_light('front', raw=True),
        #                                              self.sensors.get_light('rear', raw=True),
        #                                              self.sensors.get_light('left', raw=True),
        #                                              self.sensors.get_light('right', raw=True)))
        # print('dist {} angl {}'.format(self.parent.wall_dist(), self.parent.wall_angle() * 180 / np.pi))
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

        segment_step_size = np.min([segment_step_size, segment_length - self.distance_along])

        mtn_cb = mtn_front_bumper_stop_cb_generator(segment_step_size, poi_detection_disabled=self.poi_detected)

        distance_travelled, reason = self.motion.move(mtn_cb,
                                                      'callback',
                                                      distance_from_wall)

        if reason[:3] == 'poi':
            self.poi_detected = True
        else:
            self.poi_detected = False

        self.distance_along += distance_travelled
        print('Stopping because of {} after having traversed {}'.format(reason, distance_travelled))

        if reason == 'whisker' or self.distance_along >= segment_length:
            self.seg_transition_due = True

        return distance_travelled, reason

    def generic_transition(self):
        _, _, _, _, _, _, _, transtion_backoff = self._segment_info[
            self.current_segment]
        if transtion_backoff is not None:
            self._transition_bump_n_turn(transtion_backoff)
            # print('qwe')
            self.seg_transition_due = False
            return True, None
        else:
            return False, None

    def _transition_bump_n_turn(self, backward_move, next_segment=None):
        backward_move = -backward_move

        # Hit the wall
        if not self.sensors.get_whisker():
            self.motion.move(mtn_simple_bumper, 'callback')

        # Back off
        self.motion.move(backward_move)
        print('trunin')
        qew = self.motion.turn(-90, 'degrees')
        print('trunned {}'.format(qew))

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

    def get_segment_len(self):
        '''
        Returns the current segment's length.
        '''
        return self._segment_info[self.current_segment][0]

if __name__ == "__main__":
    # TODO Segment prior initialization code
    pass
