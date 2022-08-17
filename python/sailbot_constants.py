import math

# Adjust the nonlinear feedback Function
#
KP = 0.7
CP = 0.34

# 0 < X1 and X2 < PI
# X1 < X2
X1_SAIL = math.pi / 2
X2_SAIL = math.pi

X1_JIB = 3 * math.pi / 4
X2_JIB = math.pi

MAX_WINCH_POSITION = 360
MIN_WINCH_POSITION = 0

# Unit conversions
KMPH_TO_KNOTS = 0.5399568035
DEGREES_TO_RADIANS = math.pi / 180

# Radians
SAIL_CONTROLLER_MAX_SAIL_ANGLE = math.pi / 2
JIB_CONTROLLER_MAX_SAIL_ANGLE = math.pi / 2  * 3 / 4
MIN_HEADING_ERROR_FOR_SWITCH = 0.1
MAX_ABS_RUDDER_ANGLE_RAD = math.pi / 6
MAX_ABS_SAIL_ANGLE_RAD = math.pi / 2
MAX_ABS_JIB_ANGLE_RAD = math.pi / 2 * 3 / 4
MIN_ANGLE_FOR_SWITCH = 5 * math.pi / 180  # min angle for low power

# Knots
SPEED_THRESHOLD_FOR_JIBING_KNOTS = 0.5

# Seconds
RUDDER_UPDATE_PERIOD = 2
WINCH_UPDATE_PERIOD = 30
PUBLISHING_PERIOD = 2.5

# Volts
MIN_VOLTAGE_THRESHOLD = 17
