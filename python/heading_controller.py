from control_modes import ControlModes
from controller_selector import ControllerSelector
from sailbot_msg.msg import Sensors

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
