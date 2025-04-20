import json
import platform
from pathlib import Path

import roboteq_manager  # Assumed to be in the same directory

# -------------------------------THIS SCRIPT IS OPTIONAL--------------------------
#        "user_program.py" is the main program and will be run on completion
# --------------------------------------------------------------------------------

# This script is designed to automate the process of connecting to Roboteq motor drivers.
# It scans for devices, allows the user to select them, and establishes connections.
# It also saves the configuration to a JSON file for future use.


def load_config():
    """Load configuration from config.json if it exists."""
    config_path = Path("config.json")
    if not config_path.exists():
        print("Config file not found. Will create one when devices are discovered.")
        return None

    try:
        with open(config_path, "r") as f:
            config_data = json.load(f)

        # Add debugging to check structure
        if "devices" not in config_data:
            print(
                "Warning: config.json exists but has no 'devices' array. Expected format:"
            )
            print(
                '{\n  "devices": [\n    {"type": "serial", "port": "/dev/tty.usbserial-123", ...}\n  ]\n}'
            )
            return {"devices": []}

        if not config_data["devices"]:
            print("Config file exists but contains no device entries.")
        else:
            print(
                f"Successfully loaded {len(config_data['devices'])} device(s) from config.json"
            )

        return config_data

    except json.JSONDecodeError as e:
        print(f"Error parsing config.json: {e}")
        print("The file exists but contains invalid JSON.")
        return None
    except Exception as e:
        print(f"Unexpected error reading config.json: {e}")
        return None


def save_config(config_data):
    """Save configuration to config.json after filtering duplicates."""
    if "devices" in config_data:
        original_count = len(config_data["devices"])
        config_data["devices"], removed_count = filter_duplicate_devices(
            config_data["devices"]
        )

        if removed_count > 0:
            print(f"Removed {removed_count} duplicate device(s) from configuration")

    with open("config.json", "w") as f:
        json.dump(config_data, f, indent=4)

    print(
        f"Configuration saved to config.json with {len(config_data.get('devices', []))} device(s)"
    )


def scan_tcp_devices():
    """Scan for Roboteq devices on TCP."""
    print("Scanning for TCP devices...")

    # Prompt user for IP range and optional port
    while True:
        ip_input = input(
            "Enter IP range to scan (e.g., '192.168.1.1-10[:port]'): "
        ).strip()

        # Default values
        ip_range = ""
        port = None  # None will make scan_tcp use its default port

        # Parse input to extract IP range and optional port
        if ":" in ip_input:
            parts = ip_input.split(":")
            ip_range = parts[0]
            try:
                port = int(parts[1])
                print(f"Using custom port: {port}")
            except ValueError:
                print(f"Invalid port number: {parts[1]}. Using default port.")
                port = None
        else:
            ip_range = ip_input

        # Validate IP range format
        if not ip_range:
            print("Using default IP range: 192.168.1.1-254")
            ip_range = "192.168.1.1-254"
            break

        # Basic validation of format (could be more comprehensive)
        if "-" in ip_range and ip_range.count(".") == 3:
            break
        else:
            print("Invalid format. Please use format like '192.168.1.1-10[:port]'")

    # Implement using RoboteqManager's capabilities
    tcp_devices = []
    try:
        print(
            f"Scanning IP range: {ip_range}"
            + (f" on port {port}" if port else " on default port")
        )
        if port:
            tcp_devices = roboteq_manager.RoboteqDriver.scan_tcp(ip_range, port=port)
        else:
            tcp_devices = roboteq_manager.RoboteqDriver.scan_tcp(ip_range)
    except AttributeError:
        print("scan_tcp method not found in RoboteqManager")
    except Exception as e:
        print(f"Error during TCP scan: {str(e)}")

    return tcp_devices


def scan_com_ports():
    """Scan for COM ports based on the operating system."""
    system = platform.system()
    print(f"Scanning for COM ports on {system}...")

    com_devices = roboteq_manager.RoboteqDriver.scan_ports()

    return com_devices


def prompt_yes_no(question):
    """Ask a yes/no question and return True/False."""
    while True:
        response = input(f"{question} (y/n): ").lower().strip()
        if response in ["y", "yes"]:
            return True
        elif response in ["n", "no"]:
            return False
        print("Please answer with 'y' or 'n'.")


def select_devices(devices):
    """Let user select devices from a list."""
    if not devices:
        print("No devices found.")
        return []

    print("\nDiscovered devices:")
    for i, device in enumerate(devices, 1):
        print(f"{i}. {device}")

    while True:
        try:
            selections = input(
                "\nEnter numbers of devices to connect (comma-separated) or 'all': "
            ).strip()

            if selections.lower() == "all":
                return devices

            indices = [int(x.strip()) - 1 for x in selections.split(",") if x.strip()]

            if all(0 <= idx < len(devices) for idx in indices):
                return [devices[idx] for idx in indices]
            else:
                print("Invalid selection. Please enter valid device numbers.")
        except ValueError:
            print("Please enter numbers separated by commas.")


def filter_duplicate_devices(devices):
    """
    Filter out duplicate devices from the list.
    A duplicate is defined as having the same connection type and IP/COM port.
    Device names are allowed to be duplicated.

    Args:
        devices (list): List of device dictionaries

    Returns:
        tuple: (filtered_devices, removed_count)
    """
    unique_devices = []
    unique_signatures = set()
    duplicates = []

    for device in devices:
        # Create a signature based on connection type and address
        device_type = device.get("type")

        if device_type == "serial":
            signature = f"serial:{device.get('port')}"
        elif device_type == "tcp":
            signature = f"tcp:{device.get('host')}:{device.get('port', 9571)}"
        else:
            # Unknown device types are kept but warned about
            unique_devices.append(device)
            print(
                f"Warning: Unknown device type '{device_type}' for {device.get('name', 'Unnamed')}"
            )
            continue

        # Check if we've seen this signature before
        if signature in unique_signatures:
            duplicates.append(device)
        else:
            unique_signatures.add(signature)
            unique_devices.append(device)

    # Return the filtered list and count of duplicates
    return unique_devices, len(duplicates)


def establish_connections(devices):
    """Establish connections to the selected devices."""
    connections = []
    for device in devices:
        try:
            # Get common parameters
            device_type = device.get("type")
            timeout = device.get("timeout", 1.0)
            num_axes = device.get("num_axes", 1)
            name = device.get("name", "Unnamed")

            if device_type == "serial":
                # Connect using serial parameters
                port = device.get("port")
                if not port:
                    print(f"Missing port for serial device: {name}")
                    continue

                connection = roboteq_manager.RoboteqDriver(
                    connection_type="serial",
                    com_port=port,
                    timeout=float(timeout),
                    num_axis=int(num_axes),
                )
                print(f"✓ Connected to {name} via serial port {port}")

            elif device_type == "tcp":
                # Connect using TCP parameters
                host = device.get("host")
                port = device.get("port")
                if not host:
                    print(f"Missing host address for TCP device: {name}")
                    continue

                connection = roboteq_manager.RoboteqDriver(
                    connection_type="tcp",
                    host=host,
                    tcp_port=int(port),
                    timeout=float(timeout),
                    num_axis=int(num_axes),
                )
                print(f"✓ Connected to {name} via TCP at {host}:{port}")

            else:
                print(f"Unknown device type: {device_type} for device {name}")
                continue

            connections.append(
                {"device": device, "connection": connection, "name": name}
            )

        except Exception as e:
            print(f"✗ Error connecting to {device.get('name', 'Unnamed')}: {str(e)}")

    if connections:
        print(
            f"\nSuccessfully established {len(connections)} of {len(devices)} connections"
        )
    else:
        print("\nFailed to establish any connections")

    return connections


def print_devices_with_status(devices, connections):
    """
    Print all devices in a nicely formatted table with connection status.

    Args:
        devices (list): List of device dictionaries
        connections (list): List of successful connection dictionaries
    """
    if not devices:
        print("No devices configured.")
        return

    # Create a lookup for connected devices
    connected_devices = {}
    for conn in connections:
        device = conn.get("device")
        if device:
            # Create a signature to identify the device
            device_type = device.get("type")
            if device_type == "serial":
                signature = f"serial:{device.get('port')}"
            elif device_type == "tcp":
                signature = f"tcp:{device.get('host')}:{device.get('port', 9571)}"
            else:
                signature = f"unknown:{device.get('name', 'Unnamed')}"

            connected_devices[signature] = True

    print("\n" + "=" * 90)
    print(" DEVICE STATUS ".center(90, "="))
    print("=" * 90)

    # Determine the longest device name for formatting
    max_name_len = max([len(d.get("name", "Unnamed")) for d in devices], default=10)
    max_name_len = max(max_name_len, 10)  # At least 10 chars

    # Print header
    print(
        f"{'#':<3} | {'Name':<{max_name_len}} | {'Axes':<4} | {'Connection Details':<40} | {'Type':<8} | {'Status':<6}"
    )
    print("-" * 90)

    # Print each device
    for i, device in enumerate(devices, 1):
        device_type = device.get("type")
        name = device.get("name", "Unnamed")
        num_axes = device.get("num_axes", "-")

        # Format connection details based on type
        if device_type == "serial":
            port = device.get("port", "unknown")
            conn_details = f"Port: {port}"
            signature = f"serial:{port}"
        elif device_type == "tcp":
            host = device.get("host", "unknown")
            port = device.get("port", 9571)
            conn_details = f"Host: {host}:{port}"
            signature = f"tcp:{host}:{port}"
        else:
            conn_details = "Unknown connection type"
            signature = f"unknown:{name}"

        # Determine connection status
        status = "✓" if signature in connected_devices else "✗"

        # Print formatted row
        print(
            f"{i:<3} | {name:<{max_name_len}} | {num_axes:<4} | {conn_details:<40} | {device_type:<8} | {status:^6}"
        )

    print("=" * 90)
    print(
        f" {len(connections)} of {len(devices)} devices connected successfully ".center(
            90, "="
        )
    )
    print("=" * 90)
    print()


def main():
    # This is the main entry point of the program.
    # Thie program will load the configuration, scan for devices, and establish connections.
    # It will also handle user input and display the status of the devices.
    # At the end of the program, it will save the configuration to config.json.

    print("===========================")
    print("Roboteq Motor Driver Manager")
    print("===========================")

    # Load existing configuration
    config = load_config()
    devices = []

    if config and "devices" in config and config["devices"]:
        print(f"Found {len(config['devices'])} devices in config.json")
        devices = config["devices"]
    else:
        print("No devices found in configuration.")
    print("---------------------------")

    # Scan TCP for new devices
    if prompt_yes_no("Would you like to scan for TCP devices?"):
        tcp_devices = scan_tcp_devices()
        if tcp_devices:
            print(f"Found {len(tcp_devices)} TCP devices.")
            selected_tcp = select_devices(tcp_devices)

            # Add selected devices to the combined list
            combined_devices = devices + selected_tcp
            # Filter duplicates before saving
            unique_devices, removed = filter_duplicate_devices(combined_devices)

            if removed > 0:
                print(
                    f"Note: {removed} duplicate device(s) were detected and will not be added"
                )

            devices = unique_devices
        else:
            print("No TCP devices found.")
    print("---------------------------")

    # Scan COM ports
    if prompt_yes_no("Would you like to scan for COM port devices?"):
        com_devices = scan_com_ports()
        if com_devices:
            print(f"Found {len(com_devices)} COM port devices.")
            selected_com = select_devices(com_devices)
            devices.extend(selected_com)
        else:
            print("No COM port devices found.")
    print("---------------------------")

    # Filter duplicate devices
    devices, removed_count = filter_duplicate_devices(devices)
    if removed_count > 0:
        print(f"Removed {removed_count} duplicate device(s).")

    # Save the configuration
    if devices:
        config = {"devices": devices}
        save_config(config)
    else:
        print("No devices were selected.")
        return
    print("---------------------------")

    # Establish connections
    connections = establish_connections(devices)

    # Display all devices with connection status
    print_devices_with_status(devices, connections)

    if connections:
        print("Connections ready for use.")

        # Here you would add code to use the connections
        # For example, letting the user send commands to the motors

        # Clean up connections when done
        for conn in connections:
            try:
                conn["connection"].close()
            except:
                pass
    else:
        print("Failed to establish any connections.")


if __name__ == "__main__":
    main()
