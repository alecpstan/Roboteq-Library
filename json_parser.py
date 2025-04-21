import json
import os

# Import the module, not the class
import roboteq_manager


def load_roboteq_devices_from_json(json_filename):
    """
    Load a list of RoboteqDriver objects from a JSON file.

    Args:
        json_filename (str): The filename of the JSON file to load.

    Returns:
        list: A list of RoboteqDriver objects.
    """
    # Get the directory of the current script
    current_dir = os.path.dirname(os.path.abspath(__file__))

    # Full path to the JSON file
    json_path = os.path.join(current_dir, json_filename)

    # Check if the file exists
    if not os.path.exists(json_path):
        raise FileNotFoundError(f"JSON file not found: {json_path}")

    # Load JSON file
    with open(json_path, "r") as file:
        devices_data = json.load(file)

    # Create RoboteqDriver objects from the loaded data
    devices = []
    for device_data in devices_data:
        # Use our custom function to create a RoboteqDriver from dict
        device = _roboteq_driver_from_dict(device_data)
        devices.append(device)

    return devices


def _roboteq_driver_to_dict(device):
    """
    Convert a RoboteqDriver object to a dictionary for JSON serialization.

    Args:
        device: A RoboteqDriver object

    Returns:
        dict: A serializable dictionary with device properties
    """
    # Start with a clean dictionary
    result = {}

    # Get connection type, either from config or directly
    connection_type = getattr(device, "connect_using", None)
    if not connection_type and hasattr(device, "config"):
        connection_type = device.config.get("type")

    # Add basic properties
    result["type"] = connection_type
    result["name"] = getattr(device, "name", "Unnamed")
    result["num_axes"] = getattr(device, "num_axis", 1)
    result["timeout"] = getattr(device, "timeout", 1.0)

    # Add connection-specific details
    if connection_type == roboteq_manager.RoboteqDriver.CONN_SERIAL:
        # For serial connections, we just need the port
        result["port"] = getattr(device, "com_port", None)
        if result["port"] is None and hasattr(device, "config"):
            result["port"] = device.config.get("port")
    elif connection_type == roboteq_manager.RoboteqDriver.CONN_TCP:
        # For TCP connections, we need host and port
        result["host"] = getattr(device, "host", None)
        result["port"] = getattr(device, "tcp_port", None)
        if result["host"] is None and hasattr(device, "config"):
            result["host"] = device.config.get("host")
        if result["port"] is None and hasattr(device, "config"):
            result["port"] = device.config.get("port")

    # Remove None values
    return {k: v for k, v in result.items() if v is not None}


def save_roboteq_devices_to_json(devices, json_filename):
    """
    Save a list of RoboteqDriver objects to a JSON file.

    Args:
        devices (list): A list of RoboteqDriver objects to save.
        json_filename (str): The filename of the JSON file to save to.
    """
    # Get the directory of the current script
    current_dir = os.path.dirname(os.path.abspath(__file__))

    # Full path to the JSON file
    json_path = os.path.join(current_dir, json_filename)

    # Convert RoboteqDriver objects to serializable dictionaries
    devices_data = [_roboteq_driver_to_dict(device) for device in devices]

    # Save to JSON file
    with open(json_path, "w") as file:
        json.dump(devices_data, file, indent=4)


def _roboteq_driver_from_dict(device_data):
    """
    Create a RoboteqDriver object from a dictionary.

    Args:
        device_data (dict): Dictionary containing RoboteqDriver parameters

    Returns:
        RoboteqDriver: A new RoboteqDriver instance
    """
    # Handle both legacy and new format
    if "config" in device_data:
        config = device_data["config"]
        connection_type = config.get("type") or device_data.get("connect_using")
    else:
        config = device_data
        connection_type = device_data.get("type")

    # Create basic args dict with common parameters
    args = {
        "connect_using": connection_type,
        "num_axis": config.get("num_axes")
        or config.get("num_axis")
        or device_data.get("num_axis", 1),
        "timeout": config.get("timeout") or device_data.get("timeout", 1.0),
        "connected": False,  # Start as disconnected, will reconnect if needed
    }

    # Add connection-specific parameters
    if connection_type == roboteq_manager.RoboteqDriver.CONN_SERIAL:
        args["com_port"] = config.get("port") or device_data.get("com_port")
    elif connection_type == roboteq_manager.RoboteqDriver.CONN_TCP:
        args["host"] = config.get("host") or device_data.get("host")
        args["tcp_port"] = config.get("port") or device_data.get("tcp_port", 9571)

    # Create the driver instance
    driver = roboteq_manager.RoboteqDriver(**args)

    # Add name separately (not a constructor parameter)
    driver.name = config.get("name") or device_data.get("name", "Unnamed")

    # Create a flattened config directly on the driver
    driver.config = {
        "type": connection_type,
        "name": driver.name,
        "num_axes": args["num_axis"],
        "timeout": args["timeout"],
    }

    if connection_type == roboteq_manager.RoboteqDriver.CONN_SERIAL:
        driver.config["port"] = args["com_port"]
    elif connection_type == roboteq_manager.RoboteqDriver.CONN_TCP:
        driver.config["host"] = args["host"]
        driver.config["port"] = args["tcp_port"]

    return driver
