
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        pass

    def control(self, proposed_linear_velocity, proposed_angular_velocity, current_velocity):

        throttle = 1.0 if 0.9 * proposed_linear_velocity > current_velocity.linear.x else 0.0
        brake = 5.0 if 1.1 * proposed_linear_velocity < current_velocity.linear.x else 0.0

        # Return throttle, brake, steer
        return throttle, brake, 0
