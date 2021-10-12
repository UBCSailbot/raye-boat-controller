class HeadingController:
    def __init__(self):
        pass

    @staticmethod
    def get_feed_back_gain(heading_error):
        raise NotImplementedError()

    # implementation taken from: https://stackoverflow.com/a/2007279
    @staticmethod
    def get_heading_error_tackable(setPoint, measure):
        raise NotImplementedError()

    @staticmethod
    def get_feed_back_gain(setPoint, measure):
        raise NotImplementedError()
