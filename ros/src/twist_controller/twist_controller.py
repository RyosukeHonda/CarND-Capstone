
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):

    def __init__(self, steering_pid):

        self.steering_pid = steering_pid

    def control(self, proposed_linear_velocity, current_velocity, cross_track_error, sample_time):

        throttle = 1.0 if 0.9 * proposed_linear_velocity > current_velocity.linear.x else 0.0
        brake = 5.0 if 1.1 * proposed_linear_velocity < current_velocity.linear.x else 0.0

        steer = self.steering_pid.step(cross_track_error, sample_time)

        # Return throttle, brake, steer
        return throttle, brake, steer
