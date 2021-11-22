from jibe_controller import JibeController
from tack_controller import TackController
from control_modes import ControlModes
import sailbot_constants
# import math
from sailbot_msg.msg import Sensors

# TODO: Determine appropriate constants for controller switching conditions


class ControllerSelector:

    controllerMappings = {
        ControlModes.JIBE_ONLY: JibeController,
        ControlModes.TACKABLE: TackController
    }

    # A class reference to the current rudder controller
    __controlMode = TackController

    # An ID indicating the current rudder controller
    __controlModeID = ControlModes.UNKNOWN

    # The time of the latest controller switch
    __lastSwitchTime = 0

    def __init__(self, initialControlMode=ControlModes.UNKNOWN):
        """
        Initializes a ControllerSelector object.

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
        init_boat_speed = Sensors.gps_ais_groundspeed_knots
        unix_timestamp = Sensors.gps_ais_timestamp_utc

        # Check if the input is valid
        if (not self.isValidModeID(initialControlMode)):
            raise ValueError("An invalid control mode was passed as an argument")

        # Initialize the control mode
        if(initialControlMode == ControlModes.UNKNOWN):
            self.__switchFromUnknown(init_boat_speed)
        else:
            self.__assignMode(initialControlMode)

        self.__lastSwitchTime = int(unix_timestamp)

    def switchControlMode(self, heading_error):
        """
        Switches the current rudder controller depending on the current sensor readings
        and the current control mode. A switch only occurs when the switching interval
        has timed out (see the sailbot constants). Upon a successful switch, the time
        since the last switch should be updated to the current time and the control mode
        should be updated according to the current conditions. If the switch interval has
        not timed out, then this method has no effects.

        Arguments
        ---------
        float : heading_error
            The current heading error from the setpoint. Should be in radians.

        Returns
        -------
        bool
            Returns true if a switch successfully occurred, and false if no switch
            occurred.

        """
        boat_speed = Sensors.gps_can_groundspeed_knots
        current_time = int(Sensors.gps_ais_timestamp_utc)

        # Only switch if the switch interval has timed out
        if(current_time - self.__lastSwitchTime >= sailbot_constants.SWITCH_INTERVAL):

            # Currently tacking
            if (self.__controlModeID == ControlModes.TACKABLE):
                self.__switchFromTacking(boat_speed, current_time, self.__lastSwitchTime, heading_error)

            # Currently jibing
            elif (self.__controlModeID == ControlModes.JIBE_ONLY):
                self.__switchFromJibing(current_time, self.__lastSwitchTime, heading_error)

            # If in UNKNOWN mode, we must resolve to a new control mode
            if (self.__controlModeID == ControlModes.UNKNOWN):
                self.__switchFromUnknown(boat_speed, current_time)

            self.__lastSwitchTime = current_time
            return True

        return False

    def getControlMode(self):
        """
        Returns
        -------
        Returns a class reference to the current control mode used by the rudder.

        """
        return self.__controlMode

    def getControlModeID(self):
        """
        Returns
        -------
        int
            Returns an integer representing the control mode ID
        """
        return self.__controlModeID

    def __switchFromUnknown(self, boat_speed):
        """
        Switches the controller mode from the UNKNOWN controller to another controller.

        Switching Conditions
        --------------------
        JIBE_ONLY:
            - The boat speed is less than or equal to a speed threshold.

        TACKABLE:
            - Default control mode. If the switch condition is not met for any other
              controller, then assign the control mode to TACKABLE.

        Arguments
        ---------
        float : boat_speed
            The current boat speed in knots.

        """
        if(boat_speed <= sailbot_constants.SPEED_THRSHOLD_FOR_JIBING_KNOTS):
            self.__assignMode(ControlModes.JIBE_ONLY)

        else:
            self.__assignMode(ControlModes.TACKABLE)

        return

    def __switchFromTacking(self, boat_speed, current_time, latest_switch_time, heading_error):
        """
        The controller switches from TACKABLE mode to UNKNOWN mode if it meets its
        exit condition. The boat must be in TACKABLE mode when this method is invoked
        or there may be unexpected results.

        Exit Condition
        --------------
        - The boat speed is less than or equal to the speed threshold, OR
        - The boat has been in TACKABLE mode for longer than the specified timeout, OR
        - The boat has reached the specified setpoint

        Arguments
        ---------
        float : boat_speed
            The current boat speed in knots.

        int : current_time
            The current time (unix timestamp)

        int : latest_switch_time
            The time of the latest switch of the controller mode

        float : heading_error
            The current heading error in radians

        """
        tooSlow = boat_speed <= sailbot_constants.SPEED_THRSHOLD_FOR_JIBING_KNOTS
        timeout = current_time - latest_switch_time >= sailbot_constants.MAX_TIME_FOR_TACKING
        reached_heading = abs(heading_error) <= sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # TODO: Panic mode. If we are not at the desired heading by the end of the timeout then
        #       we need to do something else.
        if(tooSlow or timeout or reached_heading):
            self.__assignMode(ControlModes.UNKNOWN)
        return

    def __switchFromJibing(self, current_time, latest_switch_time, heading_error):
        """
        The controller switches from JIBE_ONLY mode to UNKNOWN mode if it meets its
        exit condition. The boat must be in JIBE_ONLY mode when this method is invoked
        or there may be unexpected results.

        Exit Condition
        --------------
        - The boat has been in JIBE_ONLY mode for longer than the specified timeout, OR
        - The boat has reached the specified setpoint

        Arguments
        ---------
        int : current_time
            The current time (unix timestamp)

        int : latest_switch_time
            The time of the latest switch of the controller mode

        float : heading_error
            The current heading error in radians

        """
        timeout = current_time - latest_switch_time >= sailbot_constants.MAX_TIME_FOR_JIBING
        reached_heading = abs(heading_error) <= sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        if(timeout or reached_heading):
            self.__assignMode(ControlModes.UNKNOWN)
        return

    def __assignMode(self, control_mode_id):
        """
        Changes the control mode according to the control mode ID. Refer to
        control_modes.py for the control mode IDs and how they are encoded.
        If an invalid control mode ID is passed, then the default control mode
        is tacking, but the control mode ID is assigned to UNKNOWN. Note that an
        invalid control mode ID is not ideal and should be avoided.

        Arguments
        ---------
        control_mode_id : int
            The ID of the new control mode. Refer to control_mode.py for the possible
            control mode IDs and what control modes they map to.

        """

        # Control mode is valid
        if(self.isValidModeID(control_mode_id)):
            self.__controlModeID = control_mode_id
            self.__controlMode = self.controllerMappings(self.__controlModeID, TackController)

        # Invalid control mode passed. Assigning a default value
        else:
            self.__controlModeID = ControlModes.UNKNOWN
            self.__controlMode = TackController

        return

    @staticmethod
    def isValidModeID(modeID):
        """
        Checks if a mode ID is valid.

        Arguments
        ---------
        int : modeID
            The mode ID to be checked

        Returns
        -------
        bool
            Returns True if the mode ID is valid, and false otherwise.

        """
        return (ControlModes.JIBE_ONLY <= modeID <= ControlModes.UNKNOWN) and (isinstance(modeID, int))
