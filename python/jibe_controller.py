# from heading_controller import HeadingController
# import sailbot_constants
# import math


class JibeController():
    def __init__(self):
        pass

    @staticmethod
    # How feed back gain is calculated is taken from equation 5.2 on page 25 of this paper:
    # https://core.ac.uk/download/pdf/79618904.pdf
    def get_feed_back_gain(heading_error):
        return 5

    # implementation taken from: https://stackoverflow.com/a/2007279
    @staticmethod
    def get_heading_error_tackable(setPoint, measure):
        return 0.2
