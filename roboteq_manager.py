import serial
import socket
import glob
import time
import sys
from commands import Command, Query, Config
from json_parser import *
import serial
import socket

# ANSI color codes for terminal output
RESET = "\033[0m"
GREY = "\033[90m"
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"


class RoboteqDriver:
    # Connection types
    CONN_SERIAL = "serial"
    CONN_TCP = "tcp"

    _MAX_AXES = 3  # Class constant for maximum number of axes
    _QRY_CHAR = b"\x05"  # Query character
    _ACK_CHAR = b"\x06"  # Acknowledge character

    # ----------------------------------------------------------------------------------------------------
    #           External static functions
    # ----------------------------------------------------------------------------------------------------

    @staticmethod
    def scan_tcp(host_range, port=9571, timeout=0.5):
        """
        Scan TCP/IP addresses for Roboteq controllers

        Args:
            host_range (str): IP address range (e.g., '192.168.1.1-10')
            port (int): TCP port to scan (default: 9571)
            timeout (float): Time to wait for response from each host in seconds

        Returns:
            list: List of socket.socket objects connected to detected Roboteq controllers
        """
        available_connections = []

        # Parse host range
        base_ip, start_end = host_range.split(".")[-1].split("-")
        base_prefix = ".".join(host_range.split(".")[:-1]) + "."
        start = int(base_ip)
        end = int(start_end) if "-" in host_range else start

        # Provide initial feedback
        print(f"{GREY}Scanning TCP/IP range {host_range} on port {port}...{RESET}")
        print(
            f"{GREY}This will check {end - start + 1} addresses with a {timeout}s timeout each{RESET}"
        )

        # Track progress
        total_hosts = end - start + 1
        hosts_checked = 0
        progress_step = max(1, total_hosts // 10)

        for i in range(start, end + 1):
            hosts_checked += 1
            host = base_prefix + str(i)

            # Show periodic progress
            if (
                hosts_checked % progress_step == 0
                or hosts_checked == 1
                or hosts_checked == total_hosts
            ):
                print(
                    f"Checking host {host} ({hosts_checked}/{total_hosts}, {hosts_checked/total_hosts*100:.1f}%)"
                )

            try:
                # Try to open connection
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(timeout)
                s.connect((host, port))

                # Use shared probe function
                if RoboteqDriver.probe_connection(s, timeout):
                    print(f"{GREEN}✓ Found Roboteq controller at {host}:{port}{RESET}")
                    available_connections.append(s)  # Keep the connection open
                else:
                    print(
                        f"{RED}× Device at {host}:{port} is not a Roboteq controller{RESET}"
                    )

                s.close()  # Close connection

            except (socket.error, socket.timeout):
                # Skip hosts that can't be connected to
                continue

        # Final summary
        print(
            f"{GREY}TCP scan complete. Found {len(available_connections)} Roboteq controller(s).{RESET}"
        )

        return available_connections

    @staticmethod
    def scan_ports(timeout=0.5):
        """
        Scan serial ports for Roboteq controllers

        Args:
            timeout (float): Time to wait for response from each port in seconds

        Returns:
            list: List of serial.Serial objects connected to detected Roboteq controllers
        """
        available_connections = []

        # Get list of all serial ports based on OS
        if sys.platform.startswith("win"):
            ports = ["COM%s" % (i + 1) for i in range(256)]
            print(
                f"Scanning Windows COM ports (COM1-COM256) for Roboteq controllers..."
            )
        elif sys.platform.startswith("linux"):
            ports = glob.glob("/dev/tty[A-Za-z]*")
            print(f"Scanning Linux serial ports for Roboteq controllers...")
        elif sys.platform.startswith("darwin"):  # macOS
            ports = glob.glob("/dev/tty.*")
            print(f"Scanning macOS serial ports for Roboteq controllers...")
        else:
            raise OSError("Unsupported platform")

        print(
            f"{GREY}Found {len(ports)} potential ports to check with {timeout}s timeout each{RESET}"
        )

        # Track progress
        total_ports = len(ports)
        ports_checked = 0
        progress_step = max(1, total_ports // 10)

        for port in ports:
            ports_checked += 1

            # Show periodic progress
            if (
                ports_checked % progress_step == 0
                or ports_checked == 1
                or ports_checked == total_ports
            ):
                print(
                    f"Checking port {port} ({ports_checked}/{total_ports}, {ports_checked/total_ports*100:.1f}%)"
                )

            try:
                # Create a serial connection (don't use 'with' as we want to keep it open)
                ser = serial.Serial(port, baudrate=115200, timeout=timeout)

                # Clear any existing data
                ser.reset_input_buffer()

                # Send query character and read response
                ser.write(RoboteqDriver._QRY_CHAR)
                response = ser.read()

                # Check if response is ACK
                if response == RoboteqDriver._ACK_CHAR:
                    print(f"{GREEN}✓ Found Roboteq controller on {port}{RESET}")
                    available_connections.append(ser)  # Keep the connection open
                else:
                    # Not a Roboteq controller, close the connection
                    ser.close()

            except (serial.SerialException, OSError):
                # Skip ports that can't be opened
                continue

        # Final summary
        print(
            f"{GREY}Serial port scan complete. Found {len(available_connections)} Roboteq controller(s).{RESET}"
        )

        return available_connections

    @staticmethod
    def probe_connection(connection, timeout=0.5):
        """
        Helper method to probe if a connection is to a Roboteq controller.

        Args:
            connection: Either socket.socket or serial.Serial connection object
            timeout: Time to wait for response

        Returns:
            bool: True if the connection is to a Roboteq controller
        """
        try:
            # Send query character and read response based on connection type
            if isinstance(connection, serial.Serial):
                connection.write(RoboteqDriver._QRY_CHAR)
                response = connection.read()
            elif isinstance(connection, socket.socket):
                connection.sendall(RoboteqDriver._QRY_CHAR)
                response = connection.recv(1)
            else:
                raise TypeError(
                    "Connection must be either serial.Serial or socket.socket"
                )

            # Check if response is ACK
            return response == RoboteqDriver._ACK_CHAR
        except:
            return False

    # ----------------------------------------------------------------------------------------------------
    #           Main functions
    # ----------------------------------------------------------------------------------------------------

    def __init__(
        self,
        connection=None,  # Pass an existing connection object if available
        connect_using=CONN_SERIAL,
        com_port=None,
        host=None,
        tcp_port=9571,
        timeout=0.1,
        name="Unnamed",
        num_axis=1,
    ):
        """
        Initialize Roboteq driver controller with either serial or TCP connection

        Args:
            connection (object, optional): Existing connection object (serial.Serial or socket.socket)
            connect_using (str, optional): Type of connection - 'serial' or 'tcp'
            com_port (str, conditional): Serial port name (e.g., 'COM1' or '/dev/ttyUSB0'), required for serial
            host (str, conditional): TCP host address (e.g., '192.168.1.10'), required for TCP
            tcp_port (int, conditional): TCP port number, default 9571
            timeout (float): Connection timeout in seconds
            name (str): Name of the device
            num_axis (int): Number of axes (1-3)
        """
        # Store basic properties
        self.name = name
        self.num_axis = num_axis
        self.connection_info = None  # Default to failed until proven otherwise
        self.connected = False

        try:
            # If a connection object is provided, use it instead of creating a new one
            if connection is not None:
                # Check if the connection is a serial or TCP socket
                if isinstance(connection, serial.Serial):
                    self.connection = connection
                    self.connect_using = self.CONN_SERIAL
                    self.connection_info = f"Serial port {connection.port}"
                elif isinstance(connection, socket.socket):
                    self.connection = connection
                    self.connect_using = self.CONN_TCP
                    try:
                        host, port = connection.getpeername()
                        self.connection_info = f"TCP {host}:{port}"
                    except:
                        self.connection_info = "TCP (details unavailable)"
                else:
                    print(
                        f"{YELLOW}Warning: Connection object type not recognized{RESET}"
                    )
                    self.connection = None
                    return  # Object created but not connected
            else:
                # Initialize connection based on type
                if connect_using == self.CONN_SERIAL:
                    if not com_port:
                        print(
                            f"{YELLOW}Warning: Serial port name is required for serial connection{RESET}"
                        )
                        return  # Object created but not connected

                    try:
                        self.connection = serial.Serial(
                            com_port, baudrate=115200, timeout=timeout
                        )
                        self.connection_info = f"Serial port {com_port}"
                    except (serial.SerialException, OSError) as e:
                        print(
                            f"{YELLOW}Warning: Failed to connect to serial port {com_port}: {str(e)}{RESET}"
                        )
                        self.connection = None
                        return  # Object created but not connected

                elif connect_using == self.CONN_TCP:
                    if not host:
                        print(
                            f"{YELLOW}Warning: Host address is required for TCP connection{RESET}"
                        )
                        return  # Object created but not connected

                    try:
                        self.connection = socket.socket(
                            socket.AF_INET, socket.SOCK_STREAM
                        )
                        self.connection.settimeout(timeout)
                        self.connection.connect((host, tcp_port))
                        self.connection_info = f"TCP {host}:{tcp_port}"
                    except (socket.error, socket.timeout) as e:
                        print(
                            f"{YELLOW}Warning: Failed to connect to TCP {host}:{tcp_port}: {str(e)}{RESET}"
                        )
                        self.connection = None
                        return  # Object created but not connected
                else:
                    print(
                        f"{YELLOW}Warning: Invalid connection type: {connect_using}{RESET}"
                    )
                    self.connection = None
                    return  # Object created but not connected

            # Probe the connection with colored output
            if self.probe_connection(self.connection, timeout):
                print(
                    f"{GREEN}Device at {self.connection_info} acknowledged connection{RESET}"
                )
                self.connected = True  # Connection exists and is a Roboteq device
            else:
                print(
                    f"{YELLOW}Warning: Device at {self.connection_info} did not acknowledge connection{RESET}"
                )
                self.connected = False  # Connection exists but not a Roboteq device, or some other error

            # Check if number of axes is valid
            if num_axis < 1 or num_axis > self._MAX_AXES:
                print(
                    f"{YELLOW}Warning: Number of axes must be between 1 and {self._MAX_AXES}{RESET}"
                )
                num_axis = 1  # Set to safe default

            self.num_axis = num_axis

            # Initialize dictionary to store linear movement configuration for each axis
            self.linear_config = {}

        except Exception as e:
            # Catch any other exceptions
            print(f"{RED}Error during connection initialization: {str(e)}{RESET}")
            self.connection = None
            self.connected = False
            # self.connection_info = "None"

    def _send_data(self, data):
        """
        Send data to the controller using the appropriate connection type

        Args:
            data (bytes): Data to send
        """
        if isinstance(self.connection, serial.Serial):
            self.connection.write(data)
        elif isinstance(self.connection, socket.socket):
            self.connection.sendall(data)
        else:
            raise TypeError("Connection must be either serial.Serial or socket.socket")

    def _read_data(self):
        """
        Read data from the controller using the appropriate connection type

        Returns:
            bytes: Received data
        """
        if isinstance(self.connection, serial.Serial):
            return self.connection.readline()

        elif isinstance(self.connection, socket.socket):
            # For TCP, read until we get a newline
            data = b""
            while True:
                char = self.connection.recv(1)
                if not char or char == b"\r":
                    break
                data += char
            return data + b"\r"
        else:
            raise TypeError("Connection must be either serial.Serial or socket.socket")

    def _format_string(self, input_str, cc=None, nn=None, mm=None, ee=None, set=False):
        # Determine which command type and set prefix accordingly
        if input_str in vars(Command).values():
            prefix = "!"
        elif input_str in vars(Query).values():
            prefix = "?"
        elif input_str in vars(Config).values():
            prefix = "^" if set else "~"
        else:
            # If the command does not start with !, ?, ^, or ~ raise an error
            if not input_str.startswith(("!", "?", "^", "~")):
                raise ValueError(f"Command '{input_str}' not recognized")
            prefix = ""

        # Create the initial command string with the appropriate prefix
        formatted_str = f"{prefix}{input_str}\r"

        # Replace cc with its value if present in the string
        if cc is not None:
            formatted_str = formatted_str.replace("cc", str(cc))

        # Replace nn with its value if present and nn is provided
        if nn is not None:
            formatted_str = formatted_str.replace("nn", str(nn))

        # Replace mm with its value if present and mm is provided
        if mm is not None:
            formatted_str = formatted_str.replace("mm", str(mm))

        # Replace ee with its value if present and ee is provided
        if ee is not None:
            formatted_str = formatted_str.replace("ee", str(ee))

        return formatted_str

    def send_raw(self, str, mute_echo=True, mute_feedback=False, mute_debug=False):
        """
        Send raw string to the Roboteq controller

        Args:
            str (str): Command string to send
        Returns:
            str: Response from the controller
        """
        # Check if string already ends with \r
        if str.endswith("\r"):
            to_send = str
        else:
            to_send = f"{str}\r"
        self._send_data(to_send.encode())
        # Wait for response
        response = self._read_data().decode().strip()
        # if ignore_echo, split at \r and take the last part
        if mute_echo:
            response = response.split("\r")[-1]
        response = response.replace("\r", "\r\n")

        # Check for error response
        if response == "-":
            # Raise error without color codes to avoid issues with exception handling
            raise ValueError(f"Command {to_send} failed")
        else:
            # Check if response is a valid command if not muted
            if not mute_feedback and not mute_debug:
                # Print the command and response with color codes
                print(
                    f"Command sent: {to_send.strip()}  {GREY}Response: {response.strip()}{RESET}"
                )

        return response

    def send(
        self,
        send_str,
        cc=None,
        nn=None,
        mm=None,
        ee=None,
        set=False,
        mute_feedback=False,
        mute_echo=True,
        mute_debug=False,
    ):
        """
        Send a runtime command to the controller

        Args:
            send_str (str): Command string
            cc (int or str): Usually Axis number (default: 1)
            nn (int, optional): Value for command if required
            set (bool): If True, command will set the config value (default: False). Ignored if not Config command

        Returns:
            str: Response from controller ('+' for success)

        Raises:
            ValueError: If command not in command class or command fails
        """
        formatted_str = self._format_string(
            send_str, cc=cc, nn=nn, mm=mm, ee=ee, set=set
        )

        # Use send_raw to handle sending the command
        response = self.send_raw(
            formatted_str,
            mute_echo=mute_echo,
            mute_debug=mute_debug,
            mute_feedback=mute_feedback,
        )
        # If feedback is muted, send "?" as the response
        if mute_feedback:
            response = "?"

        return response

    # ----------------------------------------------------------------------------------------------------
    #           Helper functions
    # ----------------------------------------------------------------------------------------------------

    def extract_value(self, response):
        """
        Extract the value from a response string

        Args:
            response (str): Response string from the controller

        Returns:
            str: Extracted value
        """
        # Split the response by '=' and return the second part
        try:
            return response.split("=")[1].strip()
        except IndexError:
            raise ValueError(f"Invalid response format: {response}")

    def extract_list(self, response):
        """
        Extract a list of values from a response string

        Args:
            response (str): Response string from the controller

        Returns:
            list: List of extracted values
        """
        # Split the response by '=' to get the ":" separated values
        try:
            values = response.split("=")[1].strip().split(":")
            return [int(value) for value in values]
        except (IndexError, ValueError):
            raise ValueError(f"Invalid response format: {response}")

    def is_position_reached(self, axis):
        """
        Wait until the motor reaches its target position.

        Args:
            axis (int): Axis number (1- axis count)

        Returns:
            bool: Returns True when position is reached
        """
        # Check if position is reached
        dr_response = self.send(
            Query.GET_DESTINATION_REACHED, cc=axis, mute_debug=True, mute_echo=True
        )

        # Parse response
        destination_reached = int(self.extract_value(dr_response))
        return destination_reached == 1

    def wait_for_position_reached(self, axis):
        """
        Wait until the motor(s) reach their target position.

        Args:
            axis: Either a single axis number (int) or a list of axis numbers (1-max axis)

        Returns:
            bool: Returns True when all specified axes reach their position
        """
        # Convert single axis to list for uniform processing
        axes = [axis] if isinstance(axis, int) else list(axis)

        # Validate axes
        for ax in axes:
            if not isinstance(ax, int) or ax < 1 or ax > self._MAX_AXES:
                raise ValueError(
                    f"Invalid axis value: {ax}. Must be an integer between 1 and {self._MAX_AXES}."
                )

        print(
            f"{YELLOW}{self.name} waiting for axis/axes {', '.join(map(str, axes))} to reach target position(s)...{RESET}"
        )

        dots = 0

        while True:
            # Check if all positions are reached
            all_reached = True
            for ax in axes:
                if not self.is_position_reached(ax):
                    all_reached = False
                    break

            if all_reached:
                print(f"{GREEN}✓ All positions reached{RESET}")
                return True

            # Visual feedback
            dots = (dots + 1) % 4
            print(f"{GREY}Waiting{'.' * dots}{' ' * 20}{RESET}\r", end="", flush=True)
            time.sleep(0.1)  # Small delay to prevent CPU overuse

    def send_batch(self, commands_list, mute_feedback=True):
        """
        Send multiple commands as a single string with \r delimiters.

        Args:
            commands_list (list): List of command specifications, each a dict with:
                - 'cmd': Command string (required)
                - 'cc': Channel/axis number (optional)
                - 'nn': First parameter (optional)
                - 'mm': Second parameter (optional)
                - 'ee': Third parameter (optional)
                - 'set': Boolean to set config value (optional, default False)
                - 'mute_feedback' (bool): Whether to suppress command feedback output

        Returns:
            str: Combined response (note: with multiple commands,
                only the response to the last command is typically returned)

        Example:
            device.send_commands([
                {'cmd': Command.STOP_ALL, 'cc': 1},
                {'cmd': Command.SET_SPEED, 'cc': 1, 'nn': 0},
                {'cmd': Config.OPERATING_MODE, 'cc': 1, 'nn': 3, 'set': True}
            ])
        """
        if not commands_list:
            return ""

        # Build the combined command string
        command_parts = []

        for cmd_spec in commands_list:
            # Extract parameters from command specification
            formatted_str = self._format_string(
                cmd_spec["cmd"],
                cc=cmd_spec.get("cc"),
                nn=cmd_spec.get("nn"),
                mm=cmd_spec.get("mm"),
                ee=cmd_spec.get("ee"),
                set=cmd_spec.get("set", False),
            )

            command_parts.append(formatted_str)

        # Join commands with \r
        combined_command = "\r".join(command_parts)
        # Print the batch command, but ensure \r are displayed as \r
        combined_command = combined_command.replace("\r", "\\r")
        # Print the command with color codes
        if mute_feedback:
            combined_command = combined_command.replace("\\r", "\r")
            print(f"{GREY}Sending batch command: {combined_command}{RESET}")
        else:
            # Print the command with color codes
            combined_command = combined_command.replace("\\r", "\r")
            # Print the command with color codes
            print(f"{GREY}Sending batch command: {combined_command}{RESET}")

        # Send the combined command
        return self.send_raw(combined_command, mute_feedback=mute_feedback)

    # ----------------------------------------------------------------------------------------------------
    #           Linear motion
    # ----------------------------------------------------------------------------------------------------

    def linear_configure(self, axis, mm_per_rev, min_mm=None, max_mm=None):
        """
        Configure an axis for linear movement with optional position limits

        Args:
            axis (int): Axis number
            mm_per_rev (float): Millimeters traveled per revolution
            min_mm (float, optional): Minimum allowed position in mm
            max_mm (float, optional): Maximum allowed position in mm

        Returns:
            bool: True if configuration successful

        Raises:
            ValueError: If axis is invalid or EPPR configuration cannot be retrieved
        """

        # Get the encoder PPR value from controller configuration
        try:
            eppr_response = self.send("~EPPR", cc=1, set=False)
            # Parse response (expected format: "EPPR=value")
            points_per_rev = abs(int(self.extract_value(eppr_response)))

        except (ValueError, IndexError) as e:
            raise ValueError(f"Failed to retrieve EPPR value from controller: {str(e)}")

        if points_per_rev <= 0:
            raise ValueError("Invalid EPPR received from controller.")

        # Validate min/max if provided
        if min_mm is not None and max_mm is not None:
            if min_mm >= max_mm:
                raise ValueError(
                    f"min_mm ({min_mm}) must be less than max_mm ({max_mm})"
                )

        # Store configuration for this axis
        self.linear_config[axis - 1] = {
            "points_per_rev": points_per_rev,
            "mm_per_rev": mm_per_rev,
            "enabled": True,
            "min_mm": min_mm,
            "max_mm": max_mm,
            "home_count": None,
        }

        # Put controller in closed-loop count position mode
        self.send(Config.OPERATING_MODE, set=True, cc=axis, nn=3)

        print(f"{GREEN}Axis {axis} configured for linear movement:{RESET}")
        print(
            f"  Points per revolution: {points_per_rev} (from controller EPPR configuration)"
        )
        print(f"  Millimeters per revolution: {mm_per_rev} mm/Rev")
        if min_mm is not None:
            print(f"  Minimum position: {min_mm} mm")
        if max_mm is not None:
            print(f"  Maximum position: {max_mm} mm")
        print(
            f"{YELLOW}  Note: Axis must be homed before absolute positioning can be used {RESET}"
        )

        return True

    # ----------------------------------------------------------------------------------------------------
    def linear_home_axis(
        self,
        axis,
        dio,
        direction="rev",
        speed=100,
        use_embedded_script=False,
        homing_bool=0,
    ):
        """
        Home an axis using a limit switch or sensor connected to a digital input

        Args:
            axis (int): Axis number
            dio (int): Digital input pin number for home sensor
            direction (str): Direction to move during homing ("fwd" or "rev")
            speed (int): Homing speed (default: 100)
            use_embedded_script (bool): If True, use embedded script for homing

        Returns:
            bool: True if homing successful, False otherwise
        """
        # If true, user is choosing to run a roboteq MicroBasic script which will set a bool = 1 when done.
        if use_embedded_script:
            # Run script
            self.send(Command.RUN_MICROBASIC, 1)
            # Wait for the script to finish
            while True:
                # Query the variable "homed" from the script
                response = self.send(Query.GET_BOOLEAN, nn=homing_bool)
                # Check if the response is "True"
                if int(self.extract_value(response)) == 1:
                    print(f"{GREEN}Homing completed successfully.{RESET}")
                    break
                else:
                    print(f"{GREY}Waiting for homing to complete...{RESET}")
                    time.sleep(0.1)
            # Get the current position and set as home
            try:
                home_count = self.extract_value(
                    self.send(Query.GET_ENCODER_COUNTER, cc=axis)
                )
                self.linear_config[axis - 1]["home_count"] = home_count
                # Mark axis as homed
                self.linear_config[axis - 1]["homed"] = True
                return True
            except Exception as e:
                print(f"{RED}Error setting home position: {str(e)}{RESET}")
                return False

        else:
            try:
                print(
                    f"{YELLOW}Homing axis {axis} using digital input {dio} in {direction} direction at speed {speed}...{RESET}"
                )

                # Validate axis is configured for linear movement
                is_valid, error_message = self._check_axis_configuration(axis)
                if not is_valid:
                    print(
                        f"{YELLOW}Warning: {error_message}. Call linear_configure first.{RESET}"
                    )
                    return False

                # Validate direction
                if direction not in ["fwd", "rev"]:
                    print(
                        f"{YELLOW}Warning: Direction must be 'fwd' or 'rev', using 'rev' as default{RESET}"
                    )
                    direction = "rev"

                # Validate DIO pin
                if not isinstance(dio, int) or dio < 0:
                    print(f"{YELLOW}Warning: DIO pin must be a positive integer{RESET}")
                    return False

                # Set direction multiplier
                dir_multiplier = 1 if direction == "fwd" else -1

                # Configure motion parameters individually
                self.send(Config.OPERATING_MODE, cc=axis, nn=0, set=True)
                self.send(Command.GO_TO, cc=axis, nn=0)
                self.send(
                    Command.SET_ACCELERATION, cc=axis, nn=speed * 10
                )  # Speed up slowly
                self.send(
                    Command.SET_DECELERATION, cc=axis, nn=speed * 50
                )  # Slow down quickly

                # Start moving in specified direction
                speed_value = speed * dir_multiplier
                self.send(Command.GO_TO, cc=axis, nn=speed_value)
                print(
                    f"Homing axis {axis} in {direction} direction at speed {speed}..."
                )

                # Wait for home sensor (digital input) to trigger
                dio_state = 0

                while dio_state == 0:
                    # Query all digital inputs
                    try:
                        response = self.send(Query.GET_DIN, cc=axis)
                        din_values = self.extract_list(response)
                        # Check the specific digital input (dio is 0-indexed)
                        if 0 <= dio < len(din_values):
                            dio_state = din_values[dio]
                        else:
                            print(
                                f"{YELLOW}Warning: DIO index {dio} out of range.{RESET}"
                            )
                            dio_state = 0
                    except (IndexError, ValueError) as e:
                        print(
                            f"{YELLOW}Warning: Error reading sensor: {str(e)}. Retrying...{RESET}"
                        )
                        continue

                # Get the current position and set as home
                try:
                    home_count = self.extract_value(
                        self.send(Query.GET_ENCODER_COUNTER, cc=axis)
                    )
                    self.linear_config[axis - 1]["home_count"] = home_count
                    # Mark axis as homed
                    self.linear_config[axis - 1]["homed"] = True
                except Exception as e:
                    print(f"{RED}Error setting home position: {str(e)}{RESET}")
                    return False

                # Stop and configure for position mode
                try:
                    self.send_batch(
                        [
                            {"cmd": Command.STOP_ALL, "cc": axis},
                            {"cmd": Command.SET_SPEED, "cc": axis, "nn": 0},
                            {
                                "cmd": Config.OPERATING_MODE,
                                "cc": axis,
                                "nn": 3,
                                "set": True,
                            },
                        ]
                    )
                except Exception as e:
                    print(f"{RED}Error stopping motor: {str(e)}{RESET}")
                    return False

                print(f"{GREEN}Successfully homed {axis}. Position set to 0mm.{RESET}")
                return True

            except Exception as e:
                print(f"{RED}Error during homing: {str(e)}{RESET}")
                # Emergency stop
                try:
                    self.send(Command.STOP_ALL, cc=axis)
                except:
                    pass
                return False

    # ----------------------------------------------------------------------------------------------------
    def linear_move_absolute_mm(self, axis, position_mm, speed=None, wait=False):
        """
        Move an axis to an absolute position in millimeters.

        Args:
            axis (int): Axis number (1-3)
            position_mm (float): Target position in millimeters
            speed (int, optional): Movement speed (if None, uses current speed)
            wait (bool): If True, wait for movement to complete

        Returns:
            bool: True if movement completed successfully, False otherwise
        """
        # Validate axis configuration
        is_valid, error_message = self._check_axis_configuration(
            axis, require_homed=True
        )
        if not is_valid:
            print(
                f"{YELLOW}Warning: {error_message}. Call linear_configure and linear_home_axis first.{RESET}"
            )
            return False

        try:
            # Get the linear configuration
            config = self.linear_config[axis - 1]
            home_count = config["home_count"]
            points_per_rev = config["points_per_rev"]
            mm_per_rev = config["mm_per_rev"]

            # Check position limits if configured
            min_mm = config.get("min_mm")
            max_mm = config.get("max_mm")

            if min_mm is not None and position_mm < min_mm:
                print(
                    f"{YELLOW}Warning: Target position {position_mm}mm is below minimum limit {min_mm}mm{RESET}"
                )
                position_mm = min_mm
                print(f"{YELLOW}Clamping to minimum position {min_mm}mm{RESET}")

            if max_mm is not None and position_mm > max_mm:
                print(
                    f"{YELLOW}Warning: Target position {position_mm}mm is above maximum limit {max_mm}mm{RESET}"
                )
                position_mm = max_mm
                print(f"{YELLOW}Clamping to maximum position {max_mm}mm{RESET}")

            # Calculate target encoder position
            counts_per_mm = points_per_rev / mm_per_rev  # Corrected calculation
            target_count = int(float(home_count)) + int(position_mm * counts_per_mm)

            print(
                f"{YELLOW}{self.name} moving axis {axis} to position {position_mm} mm (encoder count: {target_count}){RESET}"
            )

            # Set speed if provided
            if speed is not None:
                self.send(Command.SET_SPEED, cc=axis, nn=speed)

            # Send position command
            self.send(Command.GO_TO_POSITION, cc=axis, nn=target_count)

            if wait:
                try:
                    self.wait_for_position_reached(axis)
                except Exception as e:
                    print(
                        f"{YELLOW}Warning: Error while waiting for position: {str(e)}{RESET}"
                    )
                    return False

            return True

        except Exception as e:
            print(f"{RED}Error during linear movement: {str(e)}{RESET}")
            return False

    def _check_axis_configuration(self, axis, require_homed=False):
        """
        Check if an axis is properly configured for linear movement.

        Args:
            axis (int): Axis number to check
            require_homed (bool): Whether to check if the axis is homed

        Returns:
            tuple: (is_valid, error_message)
                - is_valid: True if configuration is valid, False otherwise
                - error_message: Description of the issue if not valid, None otherwise
        """
        # Check if axis is valid
        if not isinstance(axis, int) or axis < 1 or axis > self._MAX_AXES:
            return (
                False,
                f"Invalid axis number: {axis}. Must be between 1 and {self._MAX_AXES}",
            )

        # Check if axis is configured for linear movement
        if (axis - 1) not in self.linear_config:
            return False, f"Axis {axis} is not configured for linear movement"

        if not self.linear_config[axis - 1].get("enabled", False):
            return False, f"Axis {axis} is not enabled for linear movement"

        # Check if axis is homed when required
        if require_homed:
            if (
                "home_count" not in self.linear_config[axis - 1]
                or self.linear_config[axis - 1]["home_count"] is None
            ):
                return False, f"Axis {axis} is not homed"

        return True, None

    # ----------------------------------------------------------------------------------------------------
    #           Tidy up
    # ----------------------------------------------------------------------------------------------------

    def __del__(self):
        """Cleanup when object is destroyed"""
        if self.connected and self.connection != None:
            if hasattr(self, "connection"):
                if isinstance(self.connection, serial.Serial):
                    # Close the serial connection
                    if self.connection.is_open:
                        self.connection.close()
                elif isinstance(self.connection, socket.socket):
                    # Close the TCP connection
                    try:
                        self.connection.shutdown(socket.SHUT_RDWR)
                        self.connection.close()
                    except:
                        # Handle case where connection might already be closed
                        pass
