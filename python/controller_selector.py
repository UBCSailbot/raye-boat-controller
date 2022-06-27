from control_modes import ControlModes
import sailbot_constants

# TODO: Determine appropriate constants for controller switching conditions


class ControllerSelector:

    # An ID indicating the current rudder controller
    __controlModeID = ControlModes.UNKNOWN.value

    def __init__(self, init_boat_speed, initialControlMode=ControlModes.UNKNOWN.value):
        """
        Initializes a ControllerSelector object.

        Arguments
        ---------
        float : init_boat_speed
            The current boat speed in knots

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

        # Default to not be in low power upon initialization
        low_power = False

        # Check if the input is valid
        if (not self.isValidModeID(initialControlMode)):
            raise ValueError("An invalid control mode was passed as an argument")

        # Initialize the control mode
        if(initialControlMode == ControlModes.UNKNOWN.value):
            self.__switchFromUnknown(init_boat_speed, low_power)
        else:
            self.__assignMode(initialControlMode)

    def switchControlMode(self, heading_error, boat_speed, low_power):
        """
        Switches the current rudder controller depending on the current sensor readings
        and the current control mode. Upon a successful switch, the control mode
        should be updated according to the current conditions.

        Arguments
        ---------
        float : heading_error
            The current heading error from the setpoint. Should be in radians.

        float : boat_speed
            The current boat speed in knots

        bool : low_power
            A boolean flag that determines if the boat should go into low power mode.

        Modifies
        --------
        - Modifies the current control mode ID

        Returns
        -------
        bool
            Returns true if a switch successfully occurred, and false if no switch
            occurred (i.e. we did not enter UNKNOWN at any point).
        """

        # Currently tacking
        if (self.__controlModeID == ControlModes.TACKABLE.value):
            self.__switchFromTacking(boat_speed, heading_error, low_power)

        # Currently jibing
        elif (self.__controlModeID == ControlModes.JIBE_ONLY.value):
            self.__switchFromJibing(heading_error, low_power)

        # Currently in low power
        elif (self.__controlModeID == ControlModes.LOW_POWER.value):
            self.__switchFromLowPower(low_power)

        # If in UNKNOWN mode, we must resolve to a new control mode
        if (self.__controlModeID == ControlModes.UNKNOWN.value):
            self.__switchFromUnknown(boat_speed, low_power)
            return True
        else:
            return False

    def getControlModeID(self):
        """
        Returns
        -------
        int
            Returns an integer representing the control mode ID
        """
        return self.__controlModeID

    def __switchFromUnknown(self, boat_speed, low_power):
        """
        Switches the controller mode from the UNKNOWN controller to another controller.

        Switching Conditions
        --------------------
        LOW_POWER:
            - The low_power flag is set to True

        JIBE_ONLY:
            - The boat speed is less than or equal to a speed threshold.

        TACKABLE:
            - Default control mode. If the switch condition is not met for any other
              controller, then assign the control mode to TACKABLE.

        Arguments
        ---------
        float : boat_speed
            The current boat speed in knots.

        bool : low_power
            A boolean flag that determines if the boat is in low power mode.

        """
        if(low_power):
            self.__assignMode(ControlModes.LOW_POWER.value)

        elif(boat_speed <= sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS):
            self.__assignMode(ControlModes.JIBE_ONLY.value)

        else:
            self.__assignMode(ControlModes.TACKABLE.value)

        return

    def __switchFromTacking(self, boat_speed, heading_error, low_power):
        """
        The controller switches from TACKABLE mode to UNKNOWN mode if it meets its
        exit condition. The boat must be in TACKABLE mode when this method is invoked
        or there may be unexpected results.

        Exit Condition
        --------------
        - The boat speed is less than or equal to the speed threshold, OR
        - The boat has reached the specified setpoint
        - The boat is in low power mode

        Arguments
        ---------
        float : boat_speed
            The current boat speed in knots.

        float : heading_error
            The current heading error in radians

        bool : low_power
            A boolean flag that determines if the boat is in low power mode

        """
        tooSlow = boat_speed <= sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS
        reached_heading = abs(heading_error) <= sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # TODO: Panic mode. If we are not at the desired heading by the end of the timeout then
        #       we need to do something else.
        if(tooSlow or reached_heading or low_power):
            self.__assignMode(ControlModes.UNKNOWN.value)
        return

    def __switchFromJibing(self, heading_error, low_power):
        """
        The controller switches from JIBE_ONLY mode to UNKNOWN mode if it meets its
        exit condition. The boat must be in JIBE_ONLY mode when this method is invoked
        or there may be unexpected results.

        Exit Condition
        --------------
        - The boat has reached the specified setpoint
        - The boat is in low power mode

        Arguments
        ---------
        float : heading_error
            The current heading error in radians

        bool : low_power
            A boolean flag that determines if the boat is in low power mode

        """
        reached_heading = abs(heading_error) <= sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        if(reached_heading or low_power):
            self.__assignMode(ControlModes.UNKNOWN.value)
        return

    def __switchFromLowPower(self, low_power):
        """
        The controller switches from LOW_POWER mode to UNKNOWN mode if it meets its
        exit condition. The boat must be in LOW_POWER mode when this method is invoked
        or there may be unexpected results.

        Exit Condition
        --------------
        - The boat is not in low power mode

        Arguments
        ---------
        bool : low_power
            A boolean flag that determines if the boat is in low power mode

        """

        if(not low_power):
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
