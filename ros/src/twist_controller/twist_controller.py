import lowpass


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):

    def __init__(self, throttle_pid, brake_pid, steering_pid):

        self.throttle_pid = throttle_pid
        self.brake_pid = brake_pid
        self.steering_pid = steering_pid

        self.throttle_filter = lowpass.SmoothingFilter(window_weight=0.8)
        self.brake_filter = lowpass.SmoothingFilter(window_weight=0.0)
        self.steering_filter = lowpass.SmoothingFilter(window_weight=0.5)

    def control(self, linear_velocity_error, cross_track_error, sample_time):

        throttle = self.throttle_pid.step(linear_velocity_error, sample_time)
        throttle = self.throttle_filter.get_smoothed_value(throttle)

        brake = 0

        if throttle > 0:

            # Update brake value in filter with 0
            self.brake_filter.get_smoothed_value(0)

        else:

            throttle = 0

            brake = self.brake_pid.step(-linear_velocity_error, sample_time)
            brake = self.brake_filter.get_smoothed_value(brake)

        steering = self.steering_pid.step(cross_track_error, sample_time)

        return throttle, brake, steering
