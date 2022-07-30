import sailbot_constants
import math


class JibController:
    def __init__(self):
        pass

    @staticmethod
    # modified version of sail controller described on page 31 of the paper here:
    # https://core.ac.uk/download/pdf/79618904.pdf. Note the apparent wind angle convention
    # difference where 0 degrees for us is wind blowing from back of boat to front
    def get_jib_angle(apparent_wind_angle_rad, X1=0, X2=math.pi):

        # bound angle to be between -pi and pi based on this post: https://stackoverflow.com/a/2321125
        bounded_angle = math.atan2(
            math.sin(apparent_wind_angle_rad), math.cos(apparent_wind_angle_rad))

        # Generate Slope Value based on X1, X2, and the Max Sail angle
        k_s = (-sailbot_constants.SAIL_CONTROLLER_MAX_SAIL_ANGLE
               ) / (X2 - X1)

        # Generate Linear function from slope and intercept
        jib_angle = (k_s * abs(bounded_angle)
                     ) - (k_s * X2)

        # Clamp Linear Function
        min = 0
        max = sailbot_constants.SAIL_CONTROLLER_MAX_SAIL_ANGLE

        if(jib_angle > max):
            return max
        if(jib_angle < min):
            return min
        else:
            return jib_angle

    @staticmethod
    def get_winch_position(sailAngle, quantParam=360):
        """
        Converts a sail angle in radians to a winch position.

        ** Description of Winch Position **

        Arguments
        ---------
        float : sailAngle
            The sail angle in radians

        int : quantParam
            The quantization parameter for the winch position.

        Returns
        -------
        int
            The quantized winch position corresponding to the sail angle.
        """
        exact_position = int(sailAngle * (360 / (math.pi / 2)))
        step = 360 // quantParam
        quantized_position = int(round(exact_position / float(step))) * step
        return quantized_position
