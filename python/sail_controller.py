import sailbot_constants
import math


class SailController:
    def __init__(self):
        pass

    @staticmethod
    # modified version of sail controller described on page 31 of the paper here:
    # https://core.ac.uk/download/pdf/79618904.pdf. Note the apparent wind angle convention
    # difference where 0 degrees for us is wind blowing from back of boat to front
    def get_sail_angle(apparent_wind_angle_rad):

        # bound angle to be between -pi and pi based on this post: https://stackoverflow.com/a/2321125
        bounded_angle = math.atan2(
            math.sin(apparent_wind_angle_rad), math.cos(apparent_wind_angle_rad))
        return (
            sailbot_constants.SAIL_CONTROLLER_MAX_SAIL_ANGLE
            / (-math.pi)
            * abs(bounded_angle)
            + sailbot_constants.SAIL_CONTROLLER_MAX_SAIL_ANGLE
        )
