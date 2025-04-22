# This file contains the command definitions for a motor controller.
# These commands are used to control the motor, read feedback, and configure settings.
# The commands are organized into categories for better readability.

# cc = Channel *usually*, nn = value, mm = value2, ee = frame element

# Note: Uncommon commands are commented out, feel free to uncomment.


class Command:
    # Motion Control Commands
    SET_ACCELERATION = "AC cc nn"
    NEXT_ACCELERATION = "AX cc nn"
    SET_DECELERATION = "DC cc nn"
    NEXT_DECELERATION = "DX cc nn"
    GO_TO = "G cc nn"
    SET_SPEED = "S cc nn"
    NEXT_SPEED = "SX cc nn"

    # Stop commands
    EMERGENCY_STOP = "EX"
    RELEASE_ESTOP = "MG"
    STOP_ALL = "MS cc"
    QUICK_STOP_CHANNEL = "QST cc"
    RESET_DRIVE = "RST"

    # Position Control Commands
    GO_TO_POSITION = "P cc nn"
    GO_TO_RELATIVE = "PR cc nn"
    NEXT_ABSOLUTE_POSITION = "PX cc nn"
    NEXT_RELATIVE_POSITION = "PRX cc nn"

    # Counter/Encoder Commands
    SET_ENCODER_COUNTER = "C cc nn"
    SET_INTERNAL_COUNTER = "CB cc nn"
    # SET_SSI_COUNTER = "CSS cc nn"
    LOAD_HOME_COUNTER = "H cc"

    # Digital Output Commands
    RESET_DIGITAL_OUT_BIT = "D0 nn"
    SET_DIGITAL_OUT_BIT = "D1 nn"
    SET_ALL_DIGITAL_BITS = "DS nn"

    # PID Control Commands
    # SET_CURRENT_GAIN_I = "CIG cc nn"
    # SET_CURRENT_GAIN_P = "CPG cc nn"
    # SET_PID_GAIN_D = "DG cc nn"
    # SET_PID_GAIN_I = "IG cc nn"
    # SET_PID_GAIN_P = "PG cc nn"

    # Specialized Motor Control
    # GO_TO_TORQUE = "GIQ cc nn"
    # GO_TO_FLUX = "GID cc nn"

    # Configuration Commands
    SAVE_CONFIG_EEPROM = "EES"
    SET_BOOLEAN = "B nn mm"
    SET_BRAKE = "BRK cc nn"
    # MOTOR_SENSOR_SETUP = "MSS cc"
    SET_VARIABLE = "VAR nn mm"
    # SELF_TEST = "STT"

    # CAN Commands
    # CAN_GO_TO = "CG cc nn"
    # CAN_END = "CS ee nn"

    # Other Commands
    # RAW_DIRECT_SEND = "CU ee nn"
    # RUN_MICROBASIC = "R nn"

    # TODO DS402 not yet implemented


class Query:
    # Motor Feedback Queries
    # GET_AMPS = "A cc"
    # GET_AC_POWER = "ACP cc"
    # GET_ROTOR_ANGLE = "ANG cc"
    # GET_SIN_COS = "ASI cc"
    # GET_PEAK_AMPS = "DCA cc"
    # GET_PHASE_AMPS = "PHA cc"

    # System Status Queries
    GET_BOOLEAN = "B nn"
    # GET_BATTERY_AMPS = "BA cc"
    # GET_SENSOR_RELATIVE = "BCR cc nn" # nn only in RoboCAN
    GET_BRAKE = "BRK cc"
    # GET_LOCK_STATUS = "LK"
    GET_SENSOR_ERRORS = "SEC cc"
    SELF_TEST_RESULT = "STT"
    # GET_TIME = "TM"

    # Counter/Encoder Queries
    GET_ENCODER_COUNTER = "C cc"
    GET_INTERNAL_COUNTER = "CB cc"
    GET_PULSE_CMD = "CIP cc"
    GET_ENCODER_COUNT_RELATIVE = "CR cc"
    GET_ENCODER_SPEED_PERC = "SR cc"
    # GET_SSI_COUNTER_RELATIVE = "CSR cc"
    # GET_SSI_COUNTER_ABSOLUTE = "CSS cc"
    # GET_SSI_SPEED_RPM = "SS cc"
    # GET_SSI_SPEED_PERC = "SSR cc"
    # GET_HALL_STATE = "HS cc"
    # GET_SENSOR_ANGLE = "SNA cc"

    # Digital I/O Queries
    GET_DIN = "D"
    GET_DIN_SINGLE = "DI cc"
    GET_DOUT = "DO"
    GET_PULSE_INPUT = "PI cc"
    GET_PULSE_INPUT_CONVERTED = "PI cc"

    # Analog I/O Queries
    GET_ANALOG = "AI cc"
    GET_ANALOG_INPUT_CONVERTED = "AIC cc"
    GET_ANALOG_COMMAND_CONVERTED = "CIA cc"

    # Control Loop Queries
    GET_ERROR = "E cc"
    GET_FEEDBACK = "F cc"
    GET_FAULTS = "FF"
    GET_MOTOR_FLAGS = "FM cc"
    GET_STATUS_FLAGS = "FS"
    # GET_FOC_ANGLE_ADJUST = "FC cc"
    # GET_FOC_AMPS = "MA nn"
    GET_DESTINATION_REACHED = "DR cc"
    GET_POSITION_REL_TRACKING = "TR cc"

    # Motor Control Queries
    # GET_COMMAND = "M cc"
    # GET_POWER = "P cc"
    GET_SPEED_RPM = "S cc"
    GET_SPEED_REL = "SR"
    GET_SENSOR_SPEED = "BS cc"
    GET_SENSOR_SPEED_PERCENTAGE = "BSR cc"
    # GET_INSTANT_VOLTAGE = "SNS cc"
    GET_TEMP = "T cc"
    # GET_VOLTS = "V ee"
    GET_VAR = "VAR ee"

    # CAN Queries
    # CAN_FRAME = "CAN ee"
    # CAN_ERROR_COUNTER = "CEC cc"
    # CAN_RAW_FRAME_COUNT = "CF"
    # CAN_HEARTBEAT = "CHS cc"
    # ROBOCAN_ALIVE_NODES = "CL nn"
    # ROBOCAN_NODE_ALIVE = "ICL cc"

    # PID Control Queries
    # GET_CURRENT_GAIN_I = "CIG cc"
    # GET_CURRENT_GAIN_P = "CPG cc"
    # GET_CURRENT_GAIN_D = "CDG cc"
    # GET_PID_GAIN_D = "DG cc"
    # GET_PID_GAIN_I = "IG cc"
    # GET_PID_GAIN_P = "PG cc"

    # Other Queries
    # GET_RAW_FRAME_COUNT = "CD"
    # GET_RAW_DIRECT_FRAME = "DDT ee"
    # GET_RAW_DIRECT_FRAME_STRING = "SDT"
    # GET_INTERNAL_SERIAL_COMMAND = "CIS cc"
    # GET_FIRMWARE_ID = "FID"
    # CHECK_SCRIPT_CHECKSUM = "SCC"
    # GET_CONTROLLER_TYPE = "TRN"
    # GET_MCU_ID = "UID"

    # TODO DS402 not yet implemented, Magsensor, Flow sensor


class Config:

    # Control Loops Configuration
    OPERATING_MODE = "MMOD cc nn"

    # TODO pretty muhc all configs are not implemented
