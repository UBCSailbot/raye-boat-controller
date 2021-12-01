from io import RawIOBase
from jibe_only_rudder_controller import JibeOnlyRudderController
from tack_controller import TackController
from control_modes import ControlModes
import sailbot_constants

# TODO: Determine appropriate constants for controller switching conditions


class ControllerSelector:

    controllerMappings = {
        ControlModes.JIBE_ONLY.value: JibeOnlyRudderController,
        ControlModes.TACKABLE.value: TackController
    }

    # A class reference to the current rudder controller
    __controlMode = TackController

    # An ID indicating the current rudder controller
    __controlModeID = ControlModes.UNKNOWN.value

    # The time of the latest controller switch
    __lastSwitchTime = 0

    def __init__(self, init_boat_speed, unix_timestamp, initialControlMode=ControlModes.UNKNOWN.value):
        """
        Initializes a ControllerSelector object.

        Arguments
        ---------
        float : init_boat_speed
            The current boat speed in knots

        int : unix_timestamp
            The current time in seconds (unix timestamp)

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

        # Check if the input is valid
        if (not self.isValidModeID(initialControlMode)):
            raise ValueError("An invalid control mode was passed as an argument")

        # Initialize the control mode
        if(initialControlMode == ControlModes.UNKNOWN.value):
            self.__switchFromUnknown(init_boat_speed)
        else:
            self.__assignMode(initialControlMode)

        self.__lastSwitchTime = int(unix_timestamp)

    def switchControlMode(self, heading_error, boat_speed, current_time):
        """
        Switches the current rudder controller depending on the current sensor readings
        and the current control mode. A switch only occurs when the switching interval
        has timed out (see the sailbot constants). Upon a successful switch, the control mode
        should be updated according to the current conditions. If the switch interval has
        not timed out, then this method has no effects.

        The time since the last switch is only updated when the switch interval has timed out
        and a switch is *attempted*. It is not required that a switch be successful.

        Arguments
        ---------
        float : heading_error
            The current heading error from the setpoint. Should be in radians.

        float : boat_speed
            The current boat speed in knots

        int : current_time
            The current time in seconds (unix timestamp)

        Returns
        -------
        bool
            Returns true if a switch successfully occurred, and false if no switch
            occurred (i.e. we did not enter UNKNOWN at any point).
        """

        # Only switch if the switch interval has timed out
        if(current_time - self.__lastSwitchTime >= sailbot_constants.SWITCH_INTERVAL):

            # Currently tacking
            if (self.__controlModeID == ControlModes.TACKABLE.value):
                self.__switchFromTacking(boat_speed, current_time, self.__lastSwitchTime, heading_error)

            # Currently jibing
            elif (self.__controlModeID == ControlModes.JIBE_ONLY.value):
                self.__switchFromJibing(current_time, self.__lastSwitchTime, heading_error)

            # Update the latest switch time since a switch was attempted
            self.__lastSwitchTime = current_time

            # If in UNKNOWN mode, we must resolve to a new control mode
            if (self.__controlModeID == ControlModes.UNKNOWN.value):
                self.__switchFromUnknown(boat_speed)
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
        if(boat_speed <= sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS):
            self.__assignMode(ControlModes.JIBE_ONLY.value)

        else:
            self.__assignMode(ControlModes.TACKABLE.value)

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
        tooSlow = boat_speed <= sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS
        timeout = current_time - latest_switch_time >= sailbot_constants.MAX_TIME_FOR_TACKING
        reached_heading = abs(heading_error) <= sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # TODO: Panic mode. If we are not at the desired heading by the end of the timeout then
        #       we need to do something else.
        if(tooSlow or timeout or reached_heading):
            self.__assignMode(ControlModes.UNKNOWN.value)
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
            self.__assignMode(ControlModes.UNKNOWN.value)
        return

    def __assignMode(self, control_mode_id):
        """
        Changes the control mode according to the control mode ID. Refer to
        control_modes.py for the control mode IDs and how they are encoded.
        
        Arguments
        ---------
        control_mode_id : int
            The ID of the new control mode. Refer to control_mode.py for the possible
            control mode IDs and what control modes they map to.

        Raises
        ------
        ValueError
            Raises a value error when an invalid control mode is passed

        """

        # Control mode is valid
        if(self.isValidModeID(control_mode_id)):
            self.__controlModeID = control_mode_id
            self.__controlMode = self.controllerMappings.get(self.__controlModeID, TackController)

        # Invalid control mode passed. Raise ValueError
        else:
           raise ValueError("An invalid control mode was attempted to be set") 

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
        return (ControlModes.JIBE_ONLY.value <= modeID <= ControlModes.UNKNOWN.value) and (isinstance(modeID, int))
