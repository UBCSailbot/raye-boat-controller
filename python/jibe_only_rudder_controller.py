import math


class JibeOnlyRudderController:
    # Constructor
    def __init__(self):
        pass

    @staticmethod
    # Private Helper method to return the sign of a given number
    def __getSign(inNumber):
        return (inNumber / abs(inNumber))

    @staticmethod
    # Takes as Parameters the current_heading and desired_heading in absolute coordinates (I.E. relative to north)
    # And takes the apparent_wind_angle as the absolute angle of the wind vector.
    # Returns an integer value representing the direction the boat should turn to jibe
    # where 1 is counterclockwise and -1 is clockwise
    def get_jibe_controller_direction(
            current_heading,
            desired_heading,
            apparent_wind_angle):

        # Bind all values between 0 and 2Pi for ease of calculation
        apparent_wind_angle = (apparent_wind_angle) % (2 * math.pi)
        current_heading %= (2 * math.pi)
        desired_heading %= (2 * math.pi)

        # Find heading relative to wind, where 0 is upwind
        wind_relative_current_heading = (
            current_heading - apparent_wind_angle) % (2 * math.pi)
        wind_relative_desired_heading = (
            desired_heading - apparent_wind_angle) % (2 * math.pi)

        # find error term
        error = -wind_relative_desired_heading + wind_relative_current_heading

        # get jibe direction, 1 is counterclockwise, -1 is clockwise
        return JibeOnlyRudderController.__getSign(error)

    @staticmethod
    # Takes as Parameters desired_heading and current_heading as absolute coordinates  (I.E. relative to north)
    # and the jibe_direction as a value of either -1 or 1, where 1 is to turn
    # the boat counter clockwise and vice versa for -1
    def get_jibe_controller_error(
            current_heading,
            desired_heading,
            jibe_direction):

        current_heading %= (2 * math.pi)
        desired_heading %= (2 * math.pi)
        # get the error in one direction, subtract from 2 PI if it is the other
        # way
        error_one_way = desired_heading - current_heading

        if jibe_direction > 0:
            return max((2 * math.pi - abs(error_one_way)) * -
                       JibeOnlyRudderController.__getSign(error_one_way), error_one_way)
        else:
            return min((2 * math.pi - abs(error_one_way)) * -
                       JibeOnlyRudderController.__getSign(error_one_way), error_one_way)
                       