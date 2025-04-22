from roboteq_manager import RoboteqDriver
from commands import Command, Query, Config
from typing import List
import json_parser
import time

# ANSI color codes for terminal output
RESET = "\033[0m"
GREY = "\033[90m"
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"


def start_user_program(devices: List[RoboteqDriver]):
    """
    Process a list of Roboteq devices

    Args:
        devices: List of RoboteqDriver instances

    Returns:
        Null
    """

    # Let the user know the program has started
    print(f"{GREY}*****************************************************{RESET}")
    print(f"    User program started with {len(devices)} devices.")
    print(f"{GREY}*****************************************************{RESET}")
    # Just a variable to hold the list index for the devices.
    Driver1 = device_by_name("Driver1", devices)

    # -----------------------------------------
    #               Example program
    # -----------------------------------------
    devices[Driver1].send_raw("!RST")  # First command doesnt respond, just do this
    devices[Driver1].send_raw("?FID")  # Get the firmware ID

    # Start with some linear configuration including homing
    devices[Driver1].linear_configure(axis=1, mm_per_rev=25, min_mm=0, max_mm=5000)
    devices[Driver1].linear_home_axis(axis=1, dio=0, direction="rev", speed=100)

    # Setup acceleration and deceleration
    devices[Driver1].send(Command.SET_ACCELERATION, cc=1, nn=1000)
    devices[Driver1].send(Command.SET_DECELERATION, cc=1, nn=1000)

    # Run through your program loop.
    # E.g. Move to 500mm using linear motion
    devices[Driver1].linear_move_absolute_mm(
        axis=1, position_mm=500, speed=500, wait=True
    )

    # E.g. Move to 500mm using linear motion
    devices[Driver1].linear_move_absolute_mm(
        axis=1, position_mm=1000, speed=500, wait=True
    )

    # TODO: Example of how to use the query function
    # TODO: Example of how to use DIO
    # TODO: Example of how to query the device

    devices[Driver1].send(Command.STOP_ALL, 1)

    return None


# ----------------------------------------------------------------------------------------------------
#           Helper functions
# ----------------------------------------------------------------------------------------------------
def device_by_name(name, devices):
    """
    Find the index of a device in the devices list by its name

    Args:
        name (str): Name of the device to find
        devices (list): List of RoboteqDriver instances

    Returns:
        int: Index of the device in the list, or -1 if not found
    """
    if not devices or not isinstance(name, str):
        return -1

    for i, device in enumerate(devices):
        # Check if device has a name property that matches
        if hasattr(device, "name") and device.name == name:
            return i

    # Device not found
    print(f"Device with name '{name}' not found")
    return -1


# ----------------------------------------------------------------------------------------------------
#           Main function if you want to run this file directly, instead of from the connection automation
# ----------------------------------------------------------------------------------------------------


def main():
    """
    Main function to run the user program

    Returns:
        None
    """

    # Initialize your devices here.
    devices = json_parser.load_roboteq_devices_from_json("config.json")

    # In this example im loading them from a JSON file, but you can manually set them up like this

    # devices = [
    #    RoboteqDriver(
    #        connect_using="serial",
    #        com_port="/dev/ttyUSB0",
    #        num_axis=2,
    #    ),
    #    RoboteqDriver(
    #        connect_using="tcp",
    #        host="192.168.1.10",
    #        tcp_port=9571,
    #        num_axis=1,
    #    ),
    # ]

    # Start the user program. Ive put it in a separate function to allow it to be called externally.
    start_user_program(devices)


if __name__ == "__main__":
    main()
