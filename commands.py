# This file contains the command definitions for a motor controller.
# These commands are used to control the motor, read feedback, and configure settings.
# The commands are organized into categories for better readability.


class Command:
    # Motion Control Commands
    SET_ACCELERATION = "AC"
    NEXT_ACCELERATION = "AX"
    SET_DECELERATION = "DC"
    NEXT_DECELERATION = "DX"
    GO_TO_SPEED = "G"
    EMERGENCY_STOP = "EX"
    RELEASE_ESTOP = "MG"
    STOP_ALL = "MS"
    QUICK_STOP = "QST"
    RESET_DRIVE = "RST"
    SET_SPEED = "S"
    NEXT_SPEED = "SX"

    # Position Control Commands
    GO_TO_POSITION = "P"
    SET_PID_P = "PG"
    GO_TO_RELATIVE = "PR"
    NEXT_RELATIVE = "PRX"
    NEXT_POSITION = "PX"

    # Counter/Encoder Commands
    SET_ENCODER = "C"
    SET_INTERNAL_COUNTER = "CB"
    SET_SSI_COUNTER = "CSS"
    HOME_COUNTER = "H"

    # Digital Output Commands
    RESET_DIGITAL_OUT = "D0"
    SET_DIGITAL_OUT = "D1"
    SET_ALL_DIGITAL = "DS"

    # PID Control Commands
    SET_CURRENT_I = "CIG"
    SET_CURRENT_P = "CPG"
    SET_PID_D = "DG"
    SET_PID_I = "IG"

    # Specialized Motor Control
    GO_TO_TORQUE = "GIQ"
    GO_TO_FLUX = "GID"

    # Configuration Commands
    SAVE_CONFIG = "EES"
    SET_BOOLEAN = "B"
    SET_BRAKE = "BRK"
    MOTOR_SETUP = "MSS"
    SET_VARIABLE = "VAR"


class Query:
    # Motor Feedback Queries
    GET_AMPS = "A"
    GET_AC_POWER = "ACP"
    GET_ANALOG = "AI"
    GET_ANALOG_CONV = "AIC"
    GET_ANGLE = "ANG"
    GET_SIN_COS = "ASI"

    # System Status Queries
    GET_BOOLEAN = "B"
    GET_BATTERY_AMPS = "BA"
    GET_SENSOR_REL = "BCR"
    GET_BRAKE = "BRK"

    # Counter/Encoder Queries
    GET_COUNTER = "C"
    GET_ANALOG_CMD = "CIA"
    GET_PULSE_CMD = "CIP"
    GET_COUNT_REL = "CR"

    # Digital I/O Queries
    GET_DIN = "D"
    GET_DIN_SINGLE = "DI"
    GET_DOUT = "DO"

    # Control Loop Queries
    GET_ERROR = "E"
    GET_FEEDBACK = "F"
    GET_FAULTS = "FF"
    GET_STATUS = "FM"
    GET_FLAGS = "FS"

    # Motor Control Queries
    GET_COMMAND = "M"
    GET_POWER = "P"
    GET_SPEED = "S"
    GET_SPEED_REL = "SR"
    GET_TEMP = "T"
    GET_VOLTS = "V"
    GET_VAR = "VAR"


class Config:
    # Motor Configuration
    ACCELERATION = "AC"
    AMPS_LIMIT = "ALIM"
    AMP_TRIGGER = "ATRIG"
    BATTERY_LIMITS = "BLFB"

    # Control Loops Configuration
    PID = "CPID"
    OPERATING_MODE = "MMOD"
    POSITION_COUNT = "MVEL"

    # Encoder Configuration
    ENCODER_PPR = "EPPR"
    ENCODER_USE = "EMOD"

    # Digital Input Configuration
    DIN_ACTION = "DINA"
    DIN_CONFIG = "DINC"

    # Digital Output Configuration
    DOUT_ACTION = "DOA"
    DOUT_CONFIG = "DOC"

    # Analog Input Configuration
    AIN_ACTION = "AIC"
    AIN_MIN_MAX = "AMIN"

    # Communication Configuration
    RS232_CONFIG = "RSBR"
    CAN_CONFIG = "CAN"
    TCP_CONFIG = "TCPCONF"

    # System Configuration
    SCRIPT_CONFIG = "BRUN"
    DEVICE_ID = "DID"
    NVM_SAVE_AUTO = "EES"
    FACTORY_DEFAULTS = "FLD"
