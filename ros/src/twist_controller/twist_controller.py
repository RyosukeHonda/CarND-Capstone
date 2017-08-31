import lowpass


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):

    def __init__(self, throttle_pid, steering_pid):

        self.throttle_pid = throttle_pid
        self.steering_pid = steering_pid

        self.throttle_filter = lowpass.SmoothingFilter(window_weight=0.8)
        self.brake_filter = lowpass.SmoothingFilter(window_weight=0.2)
        self.steering_filter = lowpass.SmoothingFilter(window_weight=0.5)

    def control(self, linear_velocity_error, cross_track_error, sample_time):

        throttle = self.throttle_pid.step(linear_velocity_error, sample_time)
        throttle = self.throttle_filter.get_smoothed_value(throttle)

        brake = 0

        if throttle < 0:

            brake = 200
            brake = self.brake_filter.get_smoothed_value(brake)
            throttle = 0

        steering = self.steering_pid.step(cross_track_error, sample_time)

        return throttle, brake, steering
