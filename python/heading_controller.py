from control_modes import ControlModes
from controller_selector import ControllerSelector

# TODO: Add class methods to get error and feedback gain


class HeadingController:

    # The controller selector
    ctrl_selector = None

    # A class reference to the current controller
    controller = None

    def __init__(self, sensors, initialControlMode=ControlModes.TACKABLE):
        """
        Initializes a HeadingController object with a specified control mode.

        Arguments
        ---------
        sensors
            An object representing the sensors that contain the sensor readings. Refer to
            the following page for the possible values that can be obtained from these sensors:
            https://github.com/UBCSailbot/sailbot-msg/blob/master/msg/Sensors.msg

        int : initialControlMode
            The ID of the initial control mode that the rudder controller. This parameter must
            adhere to the existing values in the ControlModes class found in control_modes.py.

        Throws
        ------
        ValueError
            A ValueError exception is thrown when the initialControlMode is incorrectly specified.
            The control mode ID must adhere to the existing values in the ControlModes class found
            in control_modes.py or the initialization will fail.

        """
        boat_speed = sensors.gps_can_groundspeed_knots
        current_time = sensors.gps_ais_timestamp_utc

        self.ctrl_selector = ControllerSelector(
            init_boat_speed=boat_speed,
            unix_timestamp=current_time,
            initialControlMode=initialControlMode
        )

        self.controller = self.ctrl_selector.getControlMode()
        return

    def getController(self):
        """
        Returns
        -------
        Returns a class reference to the current controller used by the rudder.

        """
        return self.controller

    def switchControlMode(self, sensors):
        """
        Switches the rudder controller depending on the sensor readings. The
        controller selector is invoked and contains most of the logic for
        switching the controller.

        Arguments
        ---------
        sensors
            An object representing the sensors that contain the sensor readings. Refer to
            the following page for the possible values that can be obtained from these sensors:
            https://github.com/UBCSailbot/sailbot-msg/blob/master/msg/Sensors.msg

        Returns
        -------
        Returns a class reference to the current control mode used by the rudder.

        """
        self.controller = self.ctrl_selector.switchControlMode(sensors=sensors)
        return self.controller
