import platform
from pathlib import Path

# ANSI color codes for terminal output
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
GREY = "\033[90m"
RESET = "\033[0m"  # Reset to default color

import roboteq_manager  # Assumed to be in the same directory
from json_parser import load_roboteq_devices_from_json, save_roboteq_devices_to_json

# -------------------------------THIS SCRIPT IS OPTIONAL--------------------------
#        "user_program.py" is the main program and will be run on completion
# --------------------------------------------------------------------------------

# This script is designed to automate the process of connecting to Roboteq motor drivers.
# It scans for devices, allows the user to select them, and establishes connections.
# It also saves the configuration to a JSON file for future use.


def load_config():
    """Load configuration from config.json using the JSON parser package."""
    try:
        # Use the new JSON parser package to load devices
        devices = load_roboteq_devices_from_json("config.json")

        if not devices:
            print(f"{YELLOW}No devices found in configuration.{RESET}")
            return []

        print(f"{GREEN}Found {len(devices)} devices in configuration file{RESET}")
        return devices

    except FileNotFoundError:
        print(
            f"{YELLOW}Config file not found. Will create one when devices are discovered.{RESET}"
        )
        return []
    except Exception as e:
        print(f"{RED}Error reading config.json: {e}{RESET}")
        return []


def save_config(devices):
    """
    Save device configurations to config.json using the JSON parser package.

    Args:
        devices: List of RoboteqDriver objects
    """
    if not devices:
        print(f"{YELLOW}No devices to save.{RESET}")
        return

    try:
        # Use the new JSON parser package to save devices
        save_roboteq_devices_to_json(devices, "config.json")
        print(f"Configuration saved to config.json with {len(devices)} device(s)")
    except Exception as e:
        print(f"{RED}Error saving configuration: {e}{RESET}")


def scan_tcp_devices():
    """Scan for Roboteq devices on TCP."""
    print(f"Scanning for TCP devices...")

    # Prompt user for IP range and optional port
    while True:
        ip_input = input(
            f"{YELLOW}Enter IP range to scan (e.g., '192.168.1.1-10[:port]'):{RESET} "
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
                print(
                    f"{YELLOW}Invalid port number: {parts[1]}. Using default port.{RESET}"
                )
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
    tcp_connections = []
    try:
        if port:
            tcp_connections = roboteq_manager.RoboteqDriver.scan_tcp(
                ip_range, port=port
            )
        else:
            tcp_connections = roboteq_manager.RoboteqDriver.scan_tcp(ip_range)

        # Extract host and port info from socket connections
        tcp_info = []
        for conn in tcp_connections:
            try:
                # Get peer name from socket before it's closed
                host, port = conn.getpeername()
                tcp_info.append((host, port))
            except:
                # Skip connections where we can't get info
                pass

        # Return the connection info, not the actual connections
        return tcp_info

    except Exception as e:
        print(f"{RED}Error during TCP scan: {str(e)}{RESET}")
        return []


def scan_com_ports():
    """Scan for COM ports based on the operating system."""

    # This returns actual serial.Serial objects
    com_connections = roboteq_manager.RoboteqDriver.scan_ports()

    # Return the actual connection objects instead of just the names
    return com_connections


def prompt_yes_no(question):
    """Ask a yes/no question and return True/False."""
    while True:
        response = input(f"{question} (y/n): ").lower().strip()
        if response in ["y", "yes"]:
            return True
        elif response in ["n", "no"]:
            return False
        print("Please answer with 'y' or 'n'.")


def select_devices(connection_objects):
    """
    Let user select devices from a list of connection objects.

    Args:
        connection_objects: List of connection objects (socket.socket or serial.Serial)

    Returns:
        list: List of selected connection objects
    """
    import socket
    import serial

    if not connection_objects:
        print(f"{YELLOW}No devices found.{RESET}")
        return []

    print("\nDiscovered devices:")
    for i, conn in enumerate(connection_objects, 1):
        if isinstance(conn, socket.socket):
            try:
                host, port = conn.getpeername()
                print(f"{i}. TCP {host}:{port}")
            except:
                print(f"{i}. TCP connection (details unavailable)")
        elif isinstance(conn, serial.Serial):
            try:
                port = conn.port
                print(f"{i}. Serial port {port}")
            except:
                print(f"{i}. Serial connection (details unavailable)")
        else:
            print(f"{i}. Unknown connection type: {type(conn).__name__}")

    while True:
        try:
            selections = input(
                "\nEnter numbers of devices to connect (comma-separated) or 'all'. Return to skip: "
            ).strip()

            if selections.lower() == "all":
                return connection_objects
            else:
                indices = [
                    int(x.strip()) - 1 for x in selections.split(",") if x.strip()
                ]
                if not all(0 <= idx < len(connection_objects) for idx in indices):
                    print("Invalid selection. Please enter valid device numbers.")
                    continue

            # Return the selected connection objects
            selected_connections = [connection_objects[idx] for idx in indices]

            # Provide feedback on selected connections
            print(f"\nSelected {len(selected_connections)} connections:")
            for i, conn in enumerate(selected_connections, 1):
                if isinstance(conn, socket.socket):
                    try:
                        host, port = conn.getpeername()
                        print(f"{GREEN}✓{RESET} Selected TCP device at {host}:{port}")
                    except:
                        print(f"{GREEN}✓{RESET} Selected TCP device #{i}")
                elif isinstance(conn, serial.Serial):
                    try:
                        port = conn.port
                        print(f"{GREEN}✓{RESET} Selected serial device at {port}")
                    except:
                        print(f"{GREEN}✓{RESET} Selected serial device #{i}")
                else:
                    print(f"{GREEN}✓{RESET} Selected unknown device type #{i}")

            return selected_connections

        except ValueError:
            print("Please enter numbers separated by commas.")


def filter_duplicate_devices(devices):
    """
    Filter out duplicate devices from the list.
    A duplicate is defined as having the same port (serial) or host:port (TCP).
    When duplicates are found, keeps the newer version (later in the list).

    Args:
        devices (list): List of RoboteqDriver objects

    Returns:
        tuple: (filtered_devices, removed_count)
    """
    # Reverse the list to keep the last occurrence of each device
    devices.reverse()
    # Use a set to track seen identifiers
    seen = set()
    filtered_devices = []
    removed_count = 0

    for device in devices:
        if isinstance(device, roboteq_manager.RoboteqDriver):
            # Extract connection details
            conn_type = device.config.get("type")
            if conn_type == "serial":
                identifier = device.config.get("port")
            elif conn_type == "tcp":
                host = device.config.get("host")
                port = device.config.get("port")
                identifier = f"{host}:{port}"
            else:
                identifier = None

            # Check for duplicates
            if identifier in seen:
                removed_count += 1
            else:
                seen.add(identifier)
                filtered_devices.append(device)

    # Reverse back to original order
    filtered_devices.reverse()

    return filtered_devices, removed_count


def print_devices_with_status(devices):
    """
    Print all devices in a nicely formatted table with connection status.

    Args:
        devices: List of RoboteqDriver objects
    """
    if not devices:
        print("No devices configured.")
        return

    print(f"\n{GREY}" + "=" * 90 + f"{RESET}")
    print(f"{GREY}" + " DEVICE STATUS ".center(90, "=") + f"{RESET}")
    print(f"{GREY}" + "=" * 90 + f"{RESET}")

    # Determine the longest device name for formatting
    max_name_len = max(
        [len(getattr(d, "name", "Unnamed")) for d in devices], default=10
    )
    max_name_len = max(max_name_len, 10)  # At least 10 chars

    # Print header
    print(
        f"{'#':<3} | {'Name':<{max_name_len}} | {'Axes':<4} | {'Connection Details':<40} | {'Type':<8} | {'Status':<6}"
    )
    print("-" * 90)

    # Track connected devices
    connected_count = 0

    # Print each device
    for i, device in enumerate(devices, 1):
        name = getattr(device, "name", f"Device {i}")
        config = getattr(device, "config", {})

        # Get device details
        device_type = config.get("type", "unknown")
        num_axes = config.get("num_axes", "-")

        # Format connection details
        if device_type == "serial":
            conn_details = f"Port: {config.get('port', 'unknown')}"
        elif device_type == "tcp":
            conn_details = (
                f"Host: {config.get('host', 'unknown')}:{config.get('port', 9571)}"
            )
        else:
            conn_details = "Unknown connection type"

        # Check connection status
        status = f"{RED}✗{RESET}"  # Default to error (not connected)
        try:
            # Check connection status
            if device.connected and device.connection:
                # Try a simple operation to confirm connection is alive
                if hasattr(device, "send_raw"):
                    device.send_raw("~FLTFLAG")
                    status = f"{GREEN}✓{RESET}"
                    connected_count += 1
            else:
                status = f"{RED}✗{RESET}"
        except:
            status = f"{RED}✗{RESET}"

        # Print formatted row
        print(
            f"{i:<3} | {name:<{max_name_len}} | {num_axes:<4} | {conn_details:<40} | {device_type:<8} | {status:^6}"
        )

    print(f"{GREY}" + "=" * 90 + f"{RESET}")
    print(
        f"{GREY}"
        + f" {connected_count} of {len(devices)} devices connected successfully ".center(
            90, "="
        )
        + f"{RESET}"
    )
    print(f"{GREY}" + "=" * 90 + f"{RESET}")
    print()


def create_driver_from_connection(conn, custom_name=None, num_axis=None):
    """
    Create a RoboteqDriver from a connection object.

    Args:
        conn: Connection object (either socket.socket or serial.Serial)
        custom_name: Optional custom name for the device, if None will prompt user
        num_axis: Optional number of axes (1-3), if None will prompt user

    Returns:
        RoboteqDriver: Configured driver object

    Raises:
        TypeError: If connection is not socket.socket or serial.Serial
    """
    import socket
    import serial

    # Check connection type and extract details for display
    if isinstance(conn, socket.socket):
        # Handle TCP connection
        try:
            host, port = conn.getpeername()
            connection_info = f"TCP device at {host}:{port}"
            default_name = f"TCP_{host.split('.')[-1]}"
            config_type = "tcp"
            config_details = {"host": host, "port": port}
        except Exception as e:
            raise Exception(
                f"{YELLOW}Failed to extract TCP connection details:{RESET} {str(e)}"
            )

    elif isinstance(conn, serial.Serial):
        # Handle Serial connection
        try:
            port = conn.port
            connection_info = f"serial device at {port}"
            default_name = f"Serial_{port.split('/')[-1]}"
            config_type = "serial"
            config_details = {"port": port}
        except Exception as e:
            raise Exception(
                f"{YELLOW}Failed to extract serial connection details:{RESET} {str(e)}"
            )
    else:
        raise TypeError("Connection must be either socket.socket or serial.Serial")

    # Prompt for custom name if not provided
    name = custom_name
    if name is None:
        name = (
            input(f"Enter a name for {connection_info} ({default_name}): ").strip()
            or default_name
        )

    # Prompt for number of axes if not provided
    axes = num_axis
    if axes is None:
        while True:
            try:
                axes_input = input(f"Enter number of axes (1-3) for '{name}': ").strip()
                axes = int(axes_input) if axes_input else 1  # Default to 1 if empty
                if 1 <= axes <= 3:
                    break
                else:
                    print("Number of axes must be between 1 and 3.")
            except ValueError:
                print("Please enter a valid number.")

    # Create RoboteqDriver object directly using the connection object
    device = roboteq_manager.RoboteqDriver(
        connection=conn, num_axis=axes, name=name  # Pass the connection object directly
    )

    # Add config for saving
    device.config = {
        "type": config_type,
        "name": name,
        "num_axes": axes,
        "timeout": 1.0,
        **config_details,
    }

    print(
        f"{GREEN}✓{RESET} Created driver '{name}' with {axes} axes for {connection_info}"
    )

    # We no longer need to close the original connection as the driver will use it
    # The old connection is now managed by the RoboteqDriver

    return device


def main():
    print(f"{GREY}======================================={RESET}")
    print(f"{GREY}      Roboteq Connection Wizard {RESET}")
    print(f"{GREY}======================================={RESET}")

    # Load existing devices - these are already RoboteqDriver objects with connections
    print(f"Loading existing devices...")
    existing_devices = load_config()

    # Create a list of connections from the loaded devices
    existing_connections = []
    for device in existing_devices:
        # Simply get the connection attribute, even if None
        if hasattr(device, "connection"):
            existing_connections.append(device.connection)
            if device.connection:
                print(
                    f"{GREEN}✓{RESET} Found existing connection for {device.name} on {device.connection_info}"
                )
            else:
                print(f"{RED}✗ {YELLOW}No active connection for {device.name}{RESET}")
        else:
            print(f"{YELLOW}No connection attribute for {device.name}{RESET}")

    # Scan TCP for new devices
    tcp_devices = []
    if prompt_yes_no("Would you like to scan for TCP devices?"):
        tcp_connections = scan_tcp_devices()  # This now returns socket objects
        if tcp_connections:
            print(f"{GREEN}Found {len(tcp_connections)} TCP devices.{RESET}")
            selected_tcp_connections = select_devices(tcp_connections)

            # Convert socket connections into RoboteqDriver objects
            for conn in selected_tcp_connections:
                try:
                    # Create driver using our helper function - all prompting happens inside
                    device = create_driver_from_connection(conn)
                    tcp_devices.append(device)
                except Exception as e:
                    print(
                        f"{RED}Error creating device from connection: {str(e)}{RESET}"
                    )
        else:
            print(f"{YELLOW}No TCP devices found.{RESET}")
    print(f"{GREY}----------------------------------------{RESET}")

    # Scan COM ports
    com_devices = []
    if prompt_yes_no("Would you like to scan for COM port devices?"):
        com_connections = scan_com_ports()  # This now returns serial objects
        if com_connections:
            print(f"{GREEN}Found {len(com_connections)} COM port devices.{RESET}")
            selected_com_connections = select_devices(com_connections)

            # Convert serial connections into RoboteqDriver objects
            for conn in selected_com_connections:
                try:
                    # Create driver using our helper function - all prompting happens inside
                    device = create_driver_from_connection(conn)
                    com_devices.append(device)
                except Exception as e:
                    print(
                        f"{RED}Error creating device from connection: {str(e)}{RESET}"
                    )
        else:
            print(f"{YELLOW}No COM port devices found.{RESET}")
    print(f"{GREY}----------------------------------------{RESET}")

    # Merge all devices (existing and newly found)
    all_devices = existing_devices + tcp_devices + com_devices

    # Save the configuration
    if all_devices:
        # Filter out duplicates before saving
        filtered_devices, removed_count = filter_duplicate_devices(all_devices)
        if removed_count > 0:
            print(
                f"{YELLOW}Removed {removed_count} duplicate device(s) from configuration{RESET}, keeping the newer version(s)."
            )

        # Update our working list to the filtered list
        all_devices = filtered_devices
    else:
        print(f"{YELLOW}No devices to save.{RESET}")
        return
    print(f"{GREY}----------------------------------------{RESET}")

    # Display all devices with connection status
    print_devices_with_status(all_devices)

    # Check if there are any disconnected devices
    disconnected_devices = []
    for device in all_devices:
        if not (hasattr(device, "connection") and device.connection):
            disconnected_devices.append(device)

    # Ask if user wants to delete disconnected devices
    if disconnected_devices and prompt_yes_no(
        "Would you like to remove disconnected devices from configuration?"
    ):
        print(f"\n{YELLOW}The following devices are disconnected:{RESET}")
        for i, device in enumerate(disconnected_devices, 1):
            name = getattr(device, "name", f"Device {i}")
            config = getattr(device, "config", {})

            # Show device details
            if config.get("type") == "serial":
                details = f"Port: {config.get('port', 'unknown')}"
            elif config.get("type") == "tcp":
                details = (
                    f"Host: {config.get('host', 'unknown')}:{config.get('port', 9571)}"
                )
            else:
                details = "Unknown connection type"

            print(f"{i}. {name} - {details}")

        # Let user select which devices to remove
        while True:
            try:
                selections = input(
                    f"\n{YELLOW}Enter numbers of devices to remove (comma-separated) or 'all'. Return to keep all:{RESET} "
                ).strip()

                if not selections:
                    print(f"{GREEN}Keeping all devices in configuration{RESET}")
                    break

                if selections.lower() == "all":
                    # Remove all disconnected devices
                    all_devices = [
                        d for d in all_devices if d not in disconnected_devices
                    ]
                    print(
                        f"{GREEN}Removed all disconnected devices from configuration{RESET}"
                    )
                    break
                else:
                    indices = [
                        int(x.strip()) - 1 for x in selections.split(",") if x.strip()
                    ]
                    if not all(0 <= idx < len(disconnected_devices) for idx in indices):
                        print("Invalid selection. Please enter valid device numbers.")
                        continue

                    # Get devices to remove
                    to_remove = [disconnected_devices[idx] for idx in indices]

                    # Remove selected devices
                    all_devices = [d for d in all_devices if d not in to_remove]
                    print(
                        f"{GREEN}Removed {len(to_remove)} device(s) from configuration{RESET}"
                    )
                    break

            except ValueError:
                print("Please enter numbers separated by commas.")

    # Save the configuration to JSON
    save_config(all_devices)

    if all_devices:
        print("Devices ready for use.")

        # Import and run the user program
        # Ask the user if they want to run the user program
        if prompt_yes_no("Would you like to run the user program?"):
            # Import the user program
            import user_program

            # Run the user program
            user_program.start_user_program(all_devices)
        else:
            pass

        # Close all connections
        import serial
        import socket

        for device in all_devices:
            if device.connected:
                try:
                    # Check if device has a connection attribute and it's not None
                    if hasattr(device, "connection") and device.connection is not None:
                        # Check what type of connection it is
                        conn = device.connection

                        if isinstance(conn, serial.Serial):
                            conn.close()
                            print(
                                f"{GREEN}✓{RESET} Closed serial connection for {device.name}"
                            )
                        elif isinstance(conn, socket.socket):
                            conn.close()
                            print(
                                f"{GREEN}✓{RESET} Closed TCP connection for {device.name}"
                            )
                        else:
                            print(
                                f"{YELLOW}Unknown connection type for {device.name}: {type(conn)}{RESET}"
                            )
                    else:
                        # No valid connection to close
                        print(f"{YELLOW}No active connection for {device.name}{RESET}")

                except Exception as e:
                    print(
                        f"{RED}Error closing connection for {device.name}: {str(e)}{RESET}"
                    )

    else:
        print("No devices available.")


if __name__ == "__main__":
    main()
