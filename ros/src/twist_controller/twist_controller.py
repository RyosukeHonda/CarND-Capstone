
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        pass

    def control(self, proposed_linear_velocity, proposed_angular_velocity):

        # Return throttle, brake, steer
        return 1.0, 0., 0.
