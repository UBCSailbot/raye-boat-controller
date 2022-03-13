import sailbot_constants

class ControllerOutputRefiner:
    def __init__(self):
        pass

    @staticmethod
    def saturate(inputSignal, upperBound, lowerBound):
        """
        This function saturates an input signal between an upper bound and a
        lower bound. If the input signal is greater than the upper bound, then
        the upper bound is returned. If the input signal is smaller than the
        lower bound, then the lower bound is returned. Otherwise, the input
        signal is return unmodified.

        Paramters
        ---------
        inputSignal : float
            A float representing the input signal.

        upperBound : float
            A float representing the upper bound for the saturated signal.

        lowerBound : float
            A float representing the lower bound for the saturated signal.

        Returns
        -------
        float
            The saturated signal bounded between the upper and lower bound.
        """

        # Check for bad inputs
        assert(upperBound >= lowerBound)

        if (lowerBound <= inputSignal <= upperBound):
            return inputSignal
        elif (inputSignal > upperBound):
            return upperBound
        else:
            return lowerBound


    @staticmethod
    def lowPowerAngle(inputSignal, currAngle):
        # check if angle changed more than MIN_ANGLE_FOR_SWITCH, if not, keep the same
        return (abs(currAngle - inputSignal) 
        >= sailbot_constants.MIN_ANGLE_FOR_SWITCH)

