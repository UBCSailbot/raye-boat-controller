from enum import Enum


class ControlModes(Enum):
    JIBE_ONLY = 1
    TACKABLE = 2
    LOW_POWER = 3
    UNKNOWN = 4


# Dictionary for decoding integers to control mode name
CONTROL_MODES = {
    1: "JIBE",
    2: "TACKABLE",
    3: "LOW POWER",
    4: "UNKNOWN"
}
