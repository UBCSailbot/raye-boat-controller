import sailbot_constants
import math

class JibeOnlySailController:
    def __init__(self):
        pass

    @staticmethod

    def __getSign(inNumber):
        return (inNumber / abs(inNumber))

    def get_jibe_controller_direction(current_heading, desired_heading, apparent_wind_angle):

        #Bind all values between 0 and 2Pi for ease of calculation
        apparent_wind_angle = (apparent_wind_angle + math.pi) % (2 * math.pi)
        current_heading %= (2 * math.pi) 
        desired_heading %= (2 * math.pi)

        #Find heading relative to wind, where 0 is upwind
        wind_relative_current_heading  = (current_heading - apparent_wind_angle) % 2 * math.pi
        wind_relative_desired_heading  = (desired_heading - apparent_wind_angle) % 2 * math.pi

        #find error term
        error = -wind_relative_desired_heading + wind_relative_current_heading

        #get jibe direction, 1 is counterclockwise, -1 is clockwise
        return JibeOnlySailController.__getSign(error)

    @staticmethod

    def get_jibe_controller_error(current_heading, desired_heading, jibe_direction):
        
        current_heading %= 2 * math.pi 
        desired_heading %= 2 * math.pi
        #get the error in one direction, subtract from 2 PI if it is the other way
        error_one_way = desired_heading - current_heading

        if jibe_direction > 0:
            return max((2 * math.pi  - abs(error_one_way)) * -JibeOnlySailController.__getSign(error_one_way), error_one_way)
        else:
            return min((2 * math.pi  - abs(error_one_way)) * -JibeOnlySailController.__getSign(error_one_way), error_one_way)
