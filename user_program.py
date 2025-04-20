from roboteq_manager import RoboteqDriver
from commands import Command, Query, Config
from typing import List


def start_user_program(devices: List[RoboteqDriver]):
    """
    Process a list of Roboteq devices

    Args:
        devices: List of RoboteqDriver instances

    Returns:
        Null
    """

    # Let the user know the program has started
    print("********************************")
    print(f"User program started with {len(devices)} devices.")
    print("********************************")

    # -----------------------------------------
    #               Your program
    # -----------------------------------------
    # Start with some linear configuration including homing
    devices[device_by_name("Driver1")].linear_configure(
        axis=1, mm_per_rev=50, min_mm=0, max_mm=5000
    )
    devices[device_by_name("Driver1")].linear_home_axis(
        axis=1, dio=0, direction="rev", speed=100, deceleration=3000
    )
    devices[device_by_name("Driver1")].linear_configure(
        axis=2, mm_per_rev=50, min_mm=0, max_mm=3000
    )
    devices[device_by_name("Driver1")].linear_home_axis(
        axis=2, dio=0, direction="rev", speed=100, deceleration=3000
    )
    devices[device_by_name("Driver2")].linear_configure(
        axis=1, mm_per_rev=50, min_mm=0, max_mm=500
    )
    devices[device_by_name("Driver2")].linear_home_axis(
        axis=1, dio=0, direction="rev", speed=100, deceleration=3000
    )
    # Setup done, set speed to operating speed
    devices[device_by_name("Driver1")].send(Command.SET_SPEED, cc=1, nn=1000)
    devices[device_by_name("Driver1")].send(Command.SET_SPEED, cc=2, nn=1000)
    devices[device_by_name("Driver2")].send(Command.SET_SPEED, cc=1, nn=100)
    # Mave the first moves
    devices[device_by_name("Driver1")].linear_move_absolute_mm(
        axis=1, position_mm=1000, wait=False
    )
    devices[device_by_name("Driver1")].linear_move_absolute_mm(
        axis=2, position_mm=1000, wait=True
    )

    devices[device_by_name("Driver2")].linear_move_absolute_mm(
        axis=1, position_mm=100, wait=True
    )
    # Do you own thing, take sample etc
    # ?????????
    # Make next move
    devices[device_by_name("Driver2")].linear_move_absolute_mm(
        axis=1, position_mm="home", wait=True
    )

    # Stop all devices
    devices[device_by_name("Driver1")].send(Command.STOP_ALL)
    devices[device_by_name("Driver2")].send(Command.STOP_ALL)

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
#           If you want to run this file directly, instead of using the connection automation
# ----------------------------------------------------------------------------------------------------


def main():
    """
    Main function to run the user program

    Returns:
        None
    """
    # Initialize your devices here
    devices = [
        RoboteqDriver(
            connection_type="serial",
            com_port="/dev/ttyUSB0",
            num_axis=2,
        ),
        RoboteqDriver(
            connection_type="tcp",
            host="192.168.1.10",
            tcp_port=9571,
            num_axis=1,
        ),
    ]

    # Start the user program
    start_user_program(devices)


if __name__ == "__main__":
    main()
