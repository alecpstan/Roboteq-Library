from roboteq_manager import RoboteqDriver
from commands import Command, Query, Config


def start_user_program(devices):
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

    return None
