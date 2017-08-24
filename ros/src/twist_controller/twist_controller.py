import lowpass


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):

    def __init__(self, throttle_pid, steering_pid):

        self.throttle_pid = throttle_pid
        self.steering_pid = steering_pid

        self.throttle_filter = lowpass.LowPassFilter(tau=10, ts=1)
        self.brake_filter = lowpass.LowPassFilter(tau=10, ts=1)
        self.steering_filter = lowpass.LowPassFilter(tau=10, ts=1)

    def control(self, linear_velocity_error, cross_track_error, sample_time):

        throttle = self.throttle_pid.step(linear_velocity_error, sample_time)
        brake = 0

        if throttle < 0:

            brake = throttle
            throttle = 0

        steering = self.steering_pid.step(cross_track_error, sample_time)

        # Return throttle, brake, steer
        # return self.throttle_filter.filt(throttle), self.brake_filter.filt(brake), self.steering_filter.filt(steer)
        return throttle, brake, steering
