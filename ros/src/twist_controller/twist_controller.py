
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):

    def __init__(self, throttle_pid, steering_pid):

        self.throttle_pid = throttle_pid
        self.steering_pid = steering_pid

    def control(self, linear_velocity_error, cross_track_error, sample_time):

        throttle = self.throttle_pid.step(linear_velocity_error, sample_time)
        brake = 0

        if throttle < 0:

            brake = throttle
            throttle = 0

        steer = self.steering_pid.step(cross_track_error, sample_time)

        # Return throttle, brake, steer
        return throttle, brake, steer
