from control_modes import ControlModes
from controller_selector import ControllerSelector
from tack_controller import TackController
from jibe_only_rudder_controller import JibeOnlyRudderController
from controller_output_refiner import ControllerOutputRefiner
from sail_controller import SailController
from jib_controller import JibController

import sailbot_constants
import math


class HeadingController:

    # The controller selector
    __ctrl_selector = None

    # A class reference to the current controller
    __controlModeID = 0

    # Current Sail Angle in Radians
    __currSailAngleRad = 0

    # Current Jib Angle in Radians
    __currJibAngleRad = 0

    # Boolean for disabling low power mode
    __disableLowPower = None

    # Boolean to tell if the heading controller is in a fixed state or not
    __controlModeIsFixed = None

    # The winch quantization parameter
    __winchQuantParam = None

    def __init__(
            self,
            boat_speed=0,
            initialControlMode=ControlModes.UNKNOWN.value,
            disableLowPower=False,
            fixedControlMode=None,
            winchQuantParam=360):
        """
        Initializes a HeadingController object with a specified control mode.

        Arguments
        ---------
        float : boat_speed
            The current boat speed in knots

        int : initialControlMode (optional)
            The ID of the initial control mode that the rudder controller. This parameter must
            adhere to the existing values in the ControlModes class found in control_modes.py.

        bool : disableLowPower
            If True, low power mode is disabled and the boat will never enter low power mode.

        int, None : fixedControlMode
            If it is an integer, then the controller will fix on a specific control state. Consult
            control_modes.py for the correct integers to specify (do not specify UNKNOWN). If set to
            None, then the control mode is not fixed.

        int : winchQuantParam
            The quantization parameter for the winch position. If the value is 4 for example, the winch
            position is quantized into 4 sections.

        Throws
        ------
        ValueError
            A ValueError exception is thrown when the initialControlMode is incorrectly specified.
            The control mode ID must adhere to the existing values in the ControlModes class found
            in control_modes.py or the initialization will fail.

        """
        self.__ctrl_selector = ControllerSelector(
            init_boat_speed=boat_speed,
            initialControlMode=initialControlMode
        )

        self.__disableLowPower = disableLowPower
        self.__winchQuantParam = winchQuantParam

        if (fixedControlMode is not None) and \
           (ControllerSelector.isValidModeID(fixedControlMode)) and \
           (fixedControlMode != ControlModes.UNKNOWN.value):

            self.__controlModeID = fixedControlMode
            self.__controlModeIsFixed = True

        else:

            self.__controlModeID = self.__ctrl_selector.getControlModeID()
            self.__controlModeIsFixed = False

    def getControlModeID(self):
        """
        Returns
        -------
        int
            Returns an integer representing the control mode ID
        """
        return self.__controlModeID

    def switchControlMode(self, heading_error, boat_speed, low_battery_level=False, low_wind=False):
        """
        Switches the rudder controller depending on the sensor readings. The
        controller selector is invoked and contains most of the logic for
        switching the controller.

        Arguments
        ---------
        float : heading_error
            The current heading error from the setpoint. Should be in radians.

        float : boat_speed
            The speed of the boat in knots

        bool : low_battery_level
            A boolean flag that determines if the lowest battery level is below a thershold.

        bool : low_wind
            A boolean flag from the sensors that says if the boat is sailing in low winds.

        Modifies
        --------
        - The control mode is modified upon calling this method

        Returns
        -------
        bool
            Returns True if the switch was successful, and False if no switch
            occurred.

        """

        if (not self.__controlModeIsFixed):
            low_power = (low_battery_level or low_wind) and (not self.__disableLowPower)
            if(self.__ctrl_selector.switchControlMode(heading_error, boat_speed, low_power)):
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

        # TACKABLE
        elif (self.__controlModeID == ControlModes.TACKABLE.value):
            error = TackController.get_heading_error_tackable(
                setPoint=desired_heading,
                measure=current_heading
            )
            return error

        # LOW POWER or UNKNOWN
        else:
            if (ControllerOutputRefiner.lowPowerAngle(
                inputSignal=desired_heading,
                currAngle=current_heading
            )):
                error = TackController.get_heading_error_tackable(
                    setPoint=desired_heading,
                    measure=current_heading
                )
                return error
            else:
                return 0

    def get_feed_back_gain(self, heading_error, apparent_wind_angle_rad):
        """
        Calculates the feedback gain depending on the current heading error.

        How feedback gain is calculated is take from equation 5.2 on page 25 of this paper:
        https://core.ac.uk/download/pdf/79618904.pdf

        Arguments
        ---------
        float : heading_error
            The current heading error from the setpoint. Should be in radians.

        float : apparent_wind_angle_rad
            The current wind angle in radians based on the wind angle relative to the boat

        Returns
        -------
        float
            Returns the feedback gain.

        """

        # bound angle to be between -pi and pi based on this post: https://stackoverflow.com/a/2321125
        bounded_angle = ((apparent_wind_angle_rad + math.pi) % 2 * math.pi) - math.pi

        # Bound the heading error between -pi and pi
        if (abs(heading_error) >= math.pi):
            heading_error = ((heading_error + math.pi) % 2 * math.pi) - math.pi

        # return feedback to produce max rudder angle if boat is in the nogo zone
        if (abs(bounded_angle) > (sailbot_constants.NO_GO_ZONE_WIND_ANGLE)):
            return sailbot_constants.MAX_ABS_RUDDER_ANGLE_RAD / (abs(heading_error) + 0.01)

        # return normal feedback value
        else:
            return sailbot_constants.KP / (1 + sailbot_constants.CP * abs(heading_error))

    def get_sail_winch_position(self, apparentWindAngleRad, X1, X2):
        """
        Calculates the winch position of the sail according to the apparent wind angle.

        Arguments
        ---------
        float : apparentWindAngleRad
            The wind angle in radians

        float : X1
            Parameter for sail saturation function

        float : X2
            Parameter for sail saturation function

        Returns
        -------
        int
            The winch position of the sail
        """
        sailAngle = SailController.get_sail_angle(apparentWindAngleRad, X1, X2)
        smallChange = not ControllerOutputRefiner.lowPowerAngle(
            inputSignal=sailAngle,
            currAngle=self.__currSailAngleRad
        )

        if (self.__controlModeID == ControlModes.LOW_POWER.value) and (smallChange):
            winchPosition = SailController.get_winch_position(self.__currSailAngleRad, self.__winchQuantParam)
        else:
            self.__currSailAngleRad = sailAngle
            winchPosition = SailController.get_winch_position(sailAngle, self.__winchQuantParam)
        return winchPosition

    def get_jib_winch_position(self, apparentWindAngleRad, X1, X2):
        """
        Calculates the winch position of the jib according to the apparent wind angle.

        Arguments
        ---------
        float : apparentWindAngleRad
            The wind angle in radians

        float : X1
            Parameter for jib saturation function

        float : X2
            Parameter for jib saturation function

        Returns
        -------
        int
            The winch position of the jib
        """

        jibAngle = JibController.get_jib_angle(apparentWindAngleRad, X1, X2)
        smallChange = not ControllerOutputRefiner.lowPowerAngle(
            inputSignal=jibAngle,
            currAngle=self.__currJibAngleRad
        )

        if (self.__controlModeID == ControlModes.LOW_POWER.value) and (smallChange):
            winchPosition = JibController.get_winch_position(self.__currJibAngleRad, self.__winchQuantParam)
        else:
            self.__currJibAngleRad = jibAngle
            winchPosition = JibController.get_winch_position(jibAngle, self.__winchQuantParam)
        return winchPosition

    @property
    def lowPowerDisabled(self):
        """
        Returns
        -------
        bool
            Returns a boolean that specifies if the heading controller has low power
            mode disabled.
        """
        return self.__disableLowPower

    @property
    def controlModeIsFixed(self):
        """
        Returns
        -------
        bool
            Returns a boolean that specifies if the control state of the heading controller
            is fixed or not.
        """
        return self.__controlModeIsFixed
