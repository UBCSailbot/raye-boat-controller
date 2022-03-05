from control_modes import ControlModes
from controller_selector import ControllerSelector
from tack_controller import TackController
from jibe_only_rudder_controller import JibeOnlyRudderController

import sailbot_constants
import math

# TODO: Add class methods to get error and feedback gain


class HeadingController:

    # The controller selector
    __ctrl_selector = None

    # A class reference to the current controller
    __controlModeID = 0

    def __init__(self, boat_speed, current_time, initialControlMode=ControlModes.UNKNOWN.value):
        """
        Initializes a HeadingController object with a specified control mode.

        Arguments
        ---------
        int : initialControlMode (optional)
            The ID of the initial control mode that the rudder controller. This parameter must
            adhere to the existing values in the ControlModes class found in control_modes.py.

        float : boat_speed
            The current boat speed in knots

        int : current_time
            The current time in seconds (unix timestamp)

        Throws
        ------
        ValueError
            A ValueError exception is thrown when the initialControlMode is incorrectly specified.
            The control mode ID must adhere to the existing values in the ControlModes class found
            in control_modes.py or the initialization will fail.

        """
        self.__ctrl_selector = ControllerSelector(
            init_boat_speed=boat_speed,
            unix_timestamp=current_time,
            initialControlMode=initialControlMode
        )

        self.__controlModeID = self.__ctrl_selector.getControlModeID()

    def getControlModeID(self):
        """
        Returns
        -------
        int
            Returns an integer representing the control mode ID
        """
        return self.__controlModeID

    def switchControlMode(self, heading_error, boat_speed, current_time):
        """
        Switches the rudder controller depending on the sensor readings. The
        controller selector is invoked and contains most of the logic for
        switching the controller.

        Arguments
        ---------
        float : heading_error
            The current heading error from the setpoint. Should be in radians.

        Modifies
        --------
        - The control mode is modified upon calling this method

        Returns
        -------
        bool
            Returns True if the switch was successful, and False if no switch
            occurred.

        """

        if(self.__ctrl_selector.switchControlMode(heading_error, boat_speed, current_time)):
            self.__controlModeID = self.__ctrl_selector.getControlModeID()
            return True

        return False

    def get_heading_error(self, current_heading, desired_heading, apparent_wind_angle):
        """
        Calculates the heading error. The heading error varies depending on the current control
        mode of the boat.

        Arguments
        ---------
        float : current_heading
            The current direction of the boat in radians.

        float : desired_heading
            The desired direction of the boat in radians.

        float : apparent_wind_angle
            The apparent wind angle in radians.

        Returns
        -------
        float
            Returns the heading error depending on the current control mode in radians.

        """

        if(self.__controlModeID == ControlModes.JIBE_ONLY.value):
            jibe_direction = JibeOnlyRudderController.get_jibe_controller_direction(
                current_heading=current_heading,
                desired_heading=desired_heading,
                apparent_wind_angle=apparent_wind_angle
            )
            error = JibeOnlyRudderController.get_jibe_controller_error(
                current_heading=current_heading,
                desired_heading=desired_heading,
                jibe_direction=jibe_direction
            )
            return error

        # Either UNKNOWN or TACKABLE
        else:
            error = TackController.get_heading_error_tackable(
                setPoint=desired_heading,
                measure=current_heading
            )
            return error

    def get_feed_back_gain(self, heading_error):
        """
        Calculates the feedback gain depending on the current heading error.

        How feedback gain is calculated is take from equation 5.2 on page 25 of this paper:
        https://core.ac.uk/download/pdf/79618904.pdf

        Arguments
        ---------
        float : heading_error
            The current heading error from the setpoint. Should be in radians.

        Returns
        -------
        float
            Returns the feedback gain.

        """

        if (abs(heading_error) > 2 * math.pi):
            raise ValueError("heading_error must be between -2pi and 2pi")

        # Bound the heading error between 0 and 2pi
        if (abs(heading_error) >= 2 * math.pi):
            heading_error %= 2 * math.pi

        # Normalize heading error between -1 and 1 (relevant as controller is nonlinear)
        # heading_error = (heading_error / math.pi) - 1

        return sailbot_constants.KP / (1 + sailbot_constants.CP * abs(heading_error))
