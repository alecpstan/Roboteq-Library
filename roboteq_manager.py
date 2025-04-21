import serial
import socket
import glob
import time
import sys
from commands import Command, Query, Config
from json_parser import *

# ANSI color codes for terminal output
RESET = "\033[0m"
GRAY = "\033[90m"
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
        print(f"{GRAY}Scanning TCP/IP range {host_range} on port {port}...{RESET}")
        print(
            f"{GRAY}This will check {end - start + 1} addresses with a {timeout}s timeout each{RESET}"
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
            f"{GREEN}TCP scan complete. Found {len(available_connections)} Roboteq controller(s).{RESET}"
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
            f"{GRAY}Found {len(ports)} potential ports to check with {timeout}s timeout each{RESET}"
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
            f"{GREEN}Serial port scan complete. Found {len(available_connections)} Roboteq controller(s).{RESET}"
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
        connect_using=CONN_SERIAL,
        com_port=None,
        host=None,
        tcp_port=9571,
        timeout=1,
        num_axis=1,
        connected=False,
    ):
        """
        Initialize Roboteq driver controller with either serial or TCP connection

        Args:
            connection_type (str): Type of connection - 'serial' or 'tcp'
            com_port (str): Serial port name (e.g., 'COM1' or '/dev/ttyUSB0'), required for serial
            host (str): TCP host address (e.g., '192.168.1.10'), required for TCP
            tcp_port (int): TCP port number, default 9571
            timeout (float): Connection timeout in seconds
            connected (bool): True if device acknowledged, False otherwise

        Raises:
            ValueError: If connection parameters are invalid or num_axis exceeds MAX_AXES
        """

        # Store connection type
        self.connect_using = connect_using

        # Initialize connection based on type
        if connect_using == self.CONN_SERIAL:
            if not com_port:
                raise ValueError("Serial port name is required for serial connection")
            self.connection = serial.Serial(com_port, baudrate=115200, timeout=timeout)
            self.connection_info = f"Serial port {com_port}"
        elif connect_using == self.CONN_TCP:
            if not host:
                raise ValueError("Host address is required for TCP connection")
            self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.connection.settimeout(timeout)
            self.connection.connect((host, tcp_port))
            self.connection_info = f"TCP {host}:{tcp_port}"
        else:
            raise ValueError(f"Invalid connection type: {connect_using}")

        # Probe the connection with colored output
        if not self.probe_connection(self.connection, timeout):
            self.connected = False
            print(
                f"{YELLOW}Warning: Device at {self.connection_info} did not acknowledge connection{RESET}"
            )
        else:
            self.connected = True
            print(
                f"{GREEN}Device at {self.connection_info} acknowledged connection{RESET}"
            )

        # Check if number of axes is valid
        if num_axis < 1 or num_axis > self._MAX_AXES:
            raise ValueError(f"Number of axes must be between 1 and {self._MAX_AXES}")
        self.num_axis = num_axis
        self.timeout = timeout

        # Initialize dictionary to store linear movement configuration for each cc
        self.linear_config = {}

        print(
            f"{GREEN}Connected to Roboteq controller via {self.connection_info}{RESET}"
        )

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
                if not char or char == b"\n":
                    break
                data += char
            return data + b"\n"
        else:
            raise TypeError("Connection must be either serial.Serial or socket.socket")

    def send_raw(self, str):
        """
        Send raw string to the Roboteq controller

        Args:
            str (str): Command string to send
        Returns:
            str: Response from the controller
        """

        # Check if string already ends with \r\n
        if str.endswith("\r\n"):
            to_send = str
        else:
            to_send = f"{str}\r\n"
        self._send_data(to_send.encode())
        return self._read_data().decode().strip()

    def send(self, send_str, cc=None, nn=None, mm=None, ee=None, set=False):
        """
        Send a runtime command to the controller

        Args:
            send_str (str): Command string from cmd class
            cc (int or str): Usually Axis number (default: 1)
            nn (int, optional): Value for command if required
            set (bool): If True, command will set the config value (default: False). Ignored if not Config command

        Returns:
            str: Response from controller ('+' for success)

        Raises:
            ValueError: If command not in command class or command fails
        """
        # Determine which command type and set prefix accordingly
        if hasattr(Command, send_str):
            prefix = "!"
        elif hasattr(Query, send_str):
            prefix = "?"
        elif hasattr(Config, send_str):
            prefix = "^" if set else "~"
        else:
            raise ValueError(
                f"Invalid command: {send_str}. Must be a command from Command, Query, or Config classes"
            )

        # Create the initial command string with the appropriate prefix
        formatted_str = f"{prefix}{send_str}\r\n"

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

        # Use send_raw to handle sending the command
        response = self.send_raw(formatted_str)

        # Check for error response
        if response == "-":
            raise ValueError(f"Command {formatted_str} failed")

        return response

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
            eppr_response = self.motor_config(Config.EPPR, set=False, cc=axis)
            # Parse response (expected format: "EPPR=value")
            points_per_rev = int(eppr_response.split("=")[1])
        except (ValueError, IndexError) as e:
            raise ValueError(f"Failed to retrieve EPPR value from controller: {str(e)}")

        if points_per_rev <= 0:
            raise ValueError(
                "Invalid EPPR received from controller. Configure EPPR first using motor_config method."
            )

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
            "homed": False,
            "min_mm": min_mm,
            "max_mm": max_mm,
        }

        # Put controller in closed-loop count position mode
        self.motor_config(Config.OPERATING_MODE, set=True, cc=axis, nn=3)

        print(f"Axis {axis} configured for linear movement:")
        print(
            f"  Points per revolution: {points_per_rev} (from controller EPPR configuration)"
        )
        print(f"  Millimeters per revolution: {mm_per_rev}")
        if min_mm is not None:
            print(f"  Minimum position: {min_mm}mm")
        if max_mm is not None:
            print(f"  Maximum position: {max_mm}mm")
        print(f"  Note: Axis must be homed before absolute positioning can be used")

        return True

    def linear_home_axis(
        self, axis, dio_pin, direction="rev", speed=100, deceleration=3000
    ):
        """
        Home an axis using a limit switch or sensor connected to a digital input

        Args:
            axis (int): Axis number
            dio_pin (int): Digital input pin number for home sensor
            direction (str): Direction to move during homing ("fwd" or "rev")
            speed (int): Homing speed (default: 100)
            deceleration (int): Deceleration nn (default: 3000)

        Returns:
            bool: True if homing successful

        Raises:
            ValueError: If cc is not configured for linear movement or parameters are invalid
        """
        # Validate axis is configured for linear movement
        if (
            axis not in self.linear_config
            or not self.linear_config[axis - 1]["enabled"]
        ):
            raise ValueError(
                f"Axis {axis} is not configured for linear movement. Call configure_linear first."
            )
        if not self.linear_config[axis - 1]["homed"]:
            raise ValueError(f"Axis {axis} has not been homed. Call home_axis first.")

        # Validate direction
        if direction not in ["fwd", "rev"]:
            raise ValueError("Direction must be 'fwd' or 'rev'")

        # Validate DIO pin
        if not isinstance(dio_pin, int) or dio_pin < 1:
            raise ValueError("DIO pin must be a positive integer")

        # Set direction multiplier
        dir_multiplier = 1 if direction == "fwd" else -1

        # Set deceleration
        self.run_command(Command.SET_DECELERATION, cc=axis, nn=deceleration)

        # Change to closed-loop speed mode
        self.motor_config(Config.OPERATING_MODE, set=True, cc=axis, nn=1)

        # Start moving in specified direction
        speed_value = speed * dir_multiplier
        print(f"Homing axis {axis} in {direction} direction at speed {speed}...")
        self.run_command(Command.SET_SPEED, cc=axis, nn=speed_value)

        # Wait for home sensor (digital input) to trigger
        dio_state = 0
        while dio_state == 0:
            # Query the specific digital input
            response = self.run_query(Query.GET_DIN_SINGLE, cc=dio_pin)

            # Parse response (format: "DI=0" or "DI=1")
            try:
                dio_state = int(response.split("=")[1])
            except (IndexError, ValueError):
                raise ValueError(f"Invalid response format: {response}")

            # Small delay to avoid excessive CPU usage
            time.sleep(0.01)

        # Stop the motor
        self.run_command(Command.SET_SPEED, cc=axis, nn=0)

        # Wait for motor to come to a complete stop
        moving = True
        while moving:
            # Query speed
            response = self.run_query(Query.GET_SPEED, cc=axis)

            # Parse response (format: "S=nn")
            try:
                speed_value = int(response.split("=")[1])
                moving = abs(speed_value) > 0
            except (IndexError, ValueError):
                raise ValueError(f"Invalid response format: {response}")

            time.sleep(0.05)

        # Reset encoder to zero
        self.run_command(Command.SET_ENCODER, cc=axis, nn=0)

        # Put controller back in closed-loop count position mode
        self.motor_config(Config.OPERATING_MODE, set=True, cc=axis, nn=3)

        # Mark cc as homed
        self.linear_config[axis - 1]["homed"] = True

        print(f"Successfully homed {axis}. Position set to 0mm.")

        return True

    def linear_move_absolute_mm(
        self, axis, position_mm, wait=True, position_tolerance=25
    ):
        """
        Move an axis to an absolute position in millimeters relative to home position

        Args:
            axis (int): Axis number
            position_mm (float or str): Target position in millimeters or "home" to move to home position
            wait (bool): Whether to wait for the move to complete (default: True)

        Returns:
            str: Response from controller

        Raises:
            ValueError: If axis is not configured for linear movement, not homed, or position is out of bounds
        """
        # Validate axis is configured for linear movement
        if (
            axis not in self.linear_config
            or not self.linear_config[axis - 1]["enabled"]
        ):
            raise ValueError(
                f"Axis {axis} is not configured for linear movement. Call linear_configure first."
            )

        # Check if axis has been homed
        if not self.linear_config[axis - 1]["homed"]:
            raise ValueError(
                f"Axis {axis} has not been homed. Call linear_home_axis first."
            )

        # Handle 'home' position keyword
        if position_mm == "home":
            position_mm = 0
            print(f"Moving axis {axis} to home position (0mm)")

        # Check if position is valid
        if not isinstance(position_mm, (int, float)):
            raise ValueError(
                f"Position must be a number or 'home', got {type(position_mm)}"
            )

        # Get configuration and check limits
        axis_config = self.linear_config[axis - 1]
        min_mm = axis_config.get("min_mm")
        max_mm = axis_config.get("max_mm")

        # Check position limits if they are defined
        if min_mm is not None and position_mm < min_mm:
            raise ValueError(
                f"Position {position_mm}mm is below minimum limit of {min_mm}mm"
            )
        if max_mm is not None and position_mm > max_mm:
            raise ValueError(
                f"Position {position_mm}mm is above maximum limit of {max_mm}mm"
            )

        # Get encoder parameters
        points_per_rev = axis_config["points_per_rev"]
        mm_per_rev = axis_config["mm_per_rev"]

        # Calculate position in encoder counts
        position_counts = int(position_mm * points_per_rev / mm_per_rev)

        # Send position command to controller
        print(f"Moving axis {axis} to {position_mm}mm from home")
        response = self.run_command(Command.GO_TO_POSITION, cc=axis, nn=position_counts)

        # If wait is not requested, return immediately
        if not wait:
            return response

        # Otherwise, wait for the move to complete by polling position
        print(f"Waiting for axis {axis} to reach target position...")

        # Set parameters for position monitoring
        max_wait_time = 30  # Maximum wait time in seconds
        poll_interval = 0.1  # Time between position checks in seconds
        start_time = time.time()

        while True:
            # Get current encoder position
            current_position_response = self.run_query(Query.GET_COUNTER, cc=axis)

            try:
                # Parse response (format: "C=1234")
                current_position = int(current_position_response.split("=")[1])

                # Check if position reached within tolerance
                if abs(current_position - position_counts) <= position_tolerance:
                    print(f"Position reached: {current_position} counts")
                    return response

                # Check for timeout
                if time.time() - start_time > max_wait_time:
                    print(
                        f"Warning: Move timeout after {max_wait_time} seconds. Target: {position_counts}, Current: {current_position}"
                    )
                    return response

            except (ValueError, IndexError):
                print(
                    f"Warning: Could not parse position response: {current_position_response}"
                )

            # Wait before next check
            time.sleep(poll_interval)

    def linear_get_position_mm(self, axis):
        """
        Get current position of an Axis in millimeters

        Args:
            axis (int): Axis number

        Returns:
            float: Current position in millimeters

        Raises:
            ValueError: If cc is not configured for linear movement
        """
        # Validate axis is configured for linear movement
        if (
            axis not in self.linear_config
            or not self.linear_config[axis - 1]["enabled"]
        ):
            raise ValueError(
                f"Axis {axis} is not configured for linear movement. Call configure_linear first."
            )
        if not self.linear_config[axis - 1]["homed"]:
            raise ValueError(f"Axis {axis} has not been homed. Call home_axis first.")

        # Get configuration
        motor_settings = self.linear_config[axis - 1]
        points_per_rev = motor_settings["points_per_rev"]
        mm_per_rev = motor_settings["mm_per_rev"]

        # Get current position in encoder counts
        response = self.run_query(Query.GET_COUNTER, cc=axis)

        # Parse response (format: "C=1234")
        try:
            position_counts = int(response.split("=")[1])
        except (IndexError, ValueError):
            raise ValueError(f"Invalid response format: {response}")

        # Convert to millimeters
        position_mm = position_counts * mm_per_rev / points_per_rev

        return position_mm

    # ----------------------------------------------------------------------------------------------------
    #           Tidy up
    # ----------------------------------------------------------------------------------------------------

    def __del__(self):
        """Cleanup when object is destroyed"""
        if hasattr(self, "connection"):
            if self.connect_using == self.CONN_SERIAL:
                if self.connection.is_open:
                    self.connection.close()
            else:  # TCP
                self.connection.close()
