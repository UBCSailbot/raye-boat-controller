from heading_controller import HeadingController
import sailbot_constants
import math


class RudderController(HeadingController):
    def __init__(self):
        pass

    @staticmethod
    # How feed back gain is calculated is taken from equation 5.2 on page 25 of this paper:
    # https://core.ac.uk/download/pdf/79618904.pdf
    def get_feed_back_gain(heading_error):
        if heading_error > 2*math.pi or heading_error < -2*math.pi:
            raise ValueError("heading_error must be between -2pi and 2pi")
        while(heading_error >= 2*math.pi):
            heading_error -= 2*math.pi
        while(heading_error <= -2*math.pi):
            heading_error += 2*math.pi

        return sailbot_constants.KP / (1 + sailbot_constants.CP * abs(heading_error))

    # implementation taken from: https://stackoverflow.com/a/2007279
    @staticmethod
    def get_heading_error_tackable(setPoint, measure):
        return math.atan2(math.sin(setPoint - measure), math.cos(setPoint - measure))
