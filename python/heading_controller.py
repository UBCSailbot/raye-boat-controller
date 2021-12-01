from control_modes import ControlModes
from controller_selector import ControllerSelector
from sailbot_msg.msg import Sensors

# TODO: Add class methods to get error and feedback gain


class HeadingController:

    # The controller selector
    __ctrl_selector = None

    # A class reference to the current controller
    __controller = None

    def __init__(self, initialControlMode=ControlModes.UNKNOWN.value):
        """
        Initializes a HeadingController object with a specified control mode.

        Arguments
        ---------
        int : initialControlMode (optional)
            The ID of the initial control mode that the rudder controller. This parameter must
            adhere to the existing values in the ControlModes class found in control_modes.py.

        Throws
        ------
        ValueError
            A ValueError exception is thrown when the initialControlMode is incorrectly specified.
            The control mode ID must adhere to the existing values in the ControlModes class found
            in control_modes.py or the initialization will fail.

        """
        boat_speed = Sensors.gps_ais_groundspeed_knots
        current_time = Sensors.gps_ais_timestamp_utc

        self.__ctrl_selector = ControllerSelector(
            init_boat_speed=boat_speed,
            unix_timestamp=current_time,
            initialControlMode=initialControlMode
        )

        self.__controller = self.__ctrl_selector.getControlMode()

    def getController(self):
        """
        Returns
        -------
        Returns a class reference to the current controller used by the rudder.

        """
        return self.__controller

    def switchControlMode(self, heading_error):
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

        boat_speed = Sensors.gps_ais_groundspeed_knots
        current_time = int(Sensors.gps_ais_timestamp_utc)

        if(self.__ctrl_selector.switchControlMode(heading_error, boat_speed, current_time)):
            self.__controller = self.__ctrl_selector.getControlMode()
            return True

        return False
