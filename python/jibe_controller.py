import sailbot_constants
import math
#TODO add tests 
class JibeController

    def __init__(self):
        pass:

    @staticmethod

    def get_jibe_controller_direction(current_heading, desired_heading, apparent_wind_angle):
        #Bind all values between 0 and 2Pi for ease of calculation
        apparent_wind_angle = (apparent_wind_angle + pi) % 2 * pi
        current_heading %= 2 * pi 
        desired_heading %= 2 * pi

        #Find heading relative to wind, where 0 is upwind
        wind_relative_current_heading  = (current_heading - apparent_wind_angle) % 2 * pi
        wind_relative_desired_heading  = (current_heading - desired_wind_angle) % 2 * pi

        #find error term
        error = wind_relative_desired_heading - wind_relative_current_heading

        #get jibe direction, 1 is counterclockwise, -1 is clockwise
        return sign(error)

    @staticmethod

    def get_jibe_controller_error(current_heading, desired_heading, jibe_direction)
        
        current_heading %= 2 * pi 
        desired_heading %= 2 * pi
        #get the error in one direction, subtract from 2 PI if it is the other way
        error_one_way = desired_heading - current_heading

        if jibe_direction > 0:
            return max((2 * pi  - abs(error_one_way)) * -sign(error_one_way), error_one_way)
        else:
            return min((2 * pi  - abs(error_one_way)) * -sign(error_one_way), error_one_way)