from roboteq_manager import RoboteqDriver
from commands import Command, Query, Config
from typing import List
import json


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

    devices[device_by_name("Driver1")].run_command(
        Command.SET_SPEED, axis=1, speed=1000
    )
    devices[device_by_name("Driver1")].run_command(
        Command.SET_SPEED, axis=2, speed=1000
    )
    devices[device_by_name("Driver2")].run_command(Command.SET_SPEED, axis=1, speed=100)

    devices[device_by_name("Driver1")].linear_move_absolute_mm(
        axis=1, position_mm=1000, wait=False
    )
    devices[device_by_name("Driver1")].linear_move_absolute_mm(
        axis=2, position_mm=1000, wait=True
    )

    devices[device_by_name("Driver2")].linear_move_absolute_mm(
        axis=1, position_mm=100, wait=True
    )
    devices[device_by_name("Driver2")].linear_move_absolute_mm(
        axis=1, position_mm="home", wait=True
    )

    devices[device_by_name("Driver1")].run_command(Command.STOP_ALL)
    devices[device_by_name("Driver2")].run_command(Command.STOP_ALL)

    return None


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
