import sailbot_constants
import math


class JibController:
    def __init__(self):
        pass

    @staticmethod
    # modified version of sail controller described on page 31 of the paper here:
    # https://core.ac.uk/download/pdf/79618904.pdf. Note the apparent wind angle convention
    # difference where 0 degrees for us is wind blowing from back of boat to front
    def get_jib_angle(apparent_wind_angle_rad):

        # bound angle to be between -pi and pi based on this post: https://stackoverflow.com/a/2321125
        bounded_angle = math.atan2(
            math.sin(apparent_wind_angle_rad), math.cos(apparent_wind_angle_rad))
        return (
            sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE
            / (-math.pi)
            * abs(bounded_angle)
            + sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE
        )

    @staticmethod
    def get_winch_position(jibAngle):
        """
        Converts a jib angle in radians to a winch position.

        ** Description of Winch Position **

        Arguments
        ---------
        float : jibAngle
            The jib angle in radians

        Returns
        -------
        int
            The winch position corresponding to the jib angle
        """

        return int(jibAngle * (360 / (math.pi / 2)))
