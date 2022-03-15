from typing_extensions import Self
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

        #Generate Slope Value based on X1, X2, and the Max Sail angle
        k_s =  (-sailbot_constants.SAIL_CONTROLLER_MAX_SAIL_ANGLE ) / (sailbot_constants.X2_SAIL - sailbot_constants.X1_SAIL )

        #Generate Linear function from slope and intercept
        sail_angle = (k_s * abs(bounded_angle)) - (k_s * sailbot_constants.X2_SAIL)  

        #Clamp Linear Function
        min = 0
        max = sailbot_constants.SAIL_CONTROLLER_MAX_SAIL_ANGLE

        if(sail_angle > max):
            return max
        if(sail_angle < min):
            return min
        else: return sail_angle
