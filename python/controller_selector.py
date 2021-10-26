from jibe_controller import JibeController
from rudder_controller import RudderController
from control_modes import ControlModes
import sailbot_constants
import math
from sailbot_msg.msg import actuation_angle, heading, Sensors

#TODO: Determine appropriate constants for controller switching conditions

class ControllerSelector:

     self.controllerMappings = {
        ControlModes.JIBE_ONLY : JibeController.
        ControlModes.TACKABLE : RudderController
    }

    # A class reference to the current rudder controller
    self.__controlMode = None

    # An ID indicating the current rudder controller
    self.__controlModeID = None

    # The time of the latest controller switch
    self.__lastSwitchTime = None

    def __init__(self, init_boat_speed, unix_timestamp, initialControlMode=ControlModes.TACKABLE):
        """
        Initializes a ControllerSelector object.

        Arguments
        ---------
        float : init_boat_speed
            The initial speed of the boat in knots. Can be found from the sensor readings 
            from sailbot-msg.

        string, int : unix_timestamp
            The unix timestamp. May be passed as a string or an integer. Can be found from
            the sensor readings from sailbot-msg.

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
    
        if (not self.isValidModeID(initialControlMode)):
           raise ValueError("An invalid control mode was passed as an argument to the heading controller")
        
        if(initialControlMode == ControlModes.UNKNOWN):
            self.__switchFromUnknown(init_boat_speed, int(unix_timestamp))

        else:
            self.__controlMode = self.controllerMappings.get(initialControlMode, RudderController)

        self.__controlModeID = ControlModes(self.__controlMode)
        self.__lastSwitchTime = int(unix_timestamp)

    def switchControlMode(self, sensors):
        """
        Switches the current rudder controller depending on the current sensor readings
        and the current control mode.

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
        boat_speed = sensors.gps_can_groundspeed_knots
        current_time = int(sensors.gps_ais_timestamp_utc)

        # Currently tacking
        if (self.__controlModeID == ControlModes.TACKABLE):
            self.__switchFromTacking(boat_speed, current_time)

        # Currently jibing
        else if (self.__controlModeID == ControlModes.JIBE_ONLY):
            self.__switchFromJibing(current_time)

        # Switch from jibe controller may result in an unknown control mode
        # Mode may also be currently unknown by default
        if (self.__controlModeID == ControlModes.UNKNOWN)
            self.__switchFromUnknown(boat_speed, current_time)

        return self.__controlMode

    def getController(self):
        """
        Returns
        -------
        Returns a class reference to the current control mode used by the rudder.

        """
        return self.__controlMode

    def __switchFromUnknown(self, boat_speed, current_time):
        """
        Switches the controller mode from the UNKNOWN controller to another controller.
        The boat switches to the TACKABLE controller if the boat speed is above the
        jibing speed threshold, and switches to the JIBE_ONLY controller otherwise.

        Arguments
        ---------
        float : boat_speed
            The current boat speed in knots.

        int : current_time
            The current unix timestamp.

        """
        if(boat_speed > SPEED_THRSHOLD_FOR_JIBING_KNOTS):
            self.__controlModeID = ControlModes.TACKABLE

        else:
            self.__controlModeID = ControlModes.JIBE_ONLY
            
        self.__controlMode = self.controllerMappings.get(__controlModeID, RudderController)
        self.__lastSwitchTime = current_time
        return

    def __switchFromTacking(self, boat_speed, current_time):
        """
        Switches the controller mode from the TACKABLE controller to another controller.
        The controller switches to the JIBE_ONLY controller if the boat speed is below
        jibing speed threshold, and remains the same otherwise.

        Arguments
        ---------
        float : boat_speed
            The current boat speed in knots.

        int : current_time
            The current unix timestamp.

        """
        if(boat_speed <= SPEED_THRSHOLD_FOR_JIBING_KNOTS):
            self.__controlModeID = ControlModes.JIBE_ONLY
            self.__controlMode = self.controllerMappings.get(__controlModeID, RudderController)

        self.__lastSwitchTime = current_time
        return

    def __switchFromJibing(self, current_time):
        """
        Switches the controller mode from the JIBE_ONLY controller to another controller.
        If the boat has reached the current way point, or the boat has been jibing beyond
        a specified time limit, then the controller switches to the UNKNOWN controller.
        Otherwise, the control mode remains the same.

        Arguments
        ---------
        float : boat_speed
            The current boat speed in knots.

        int : current_time
            The current unix timestamp.

        """
        #TODO: How to check if the boat reaches waypoint
        if(current_time - self.__lastSwitchTime >= MAX_TIME_FOR_JIBING):
            self.__controlModeID = ControlModes.UNKNOWN
            self.__controlMode = self.controllerMappings(self.__controlModeID, RudderController)
            self.__lastSwitchTime = current_time
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
        return (ControlModes.JIBE_ONLY <= modeID <= ControlModes.UNKNOWN) and
               (isinstance(modeID, int))
