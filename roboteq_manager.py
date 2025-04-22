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

    def send_raw(self, str, ignore_echo=True):
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
        if ignore_echo:
            response = response.split("\r")[-1]
        response = response.replace("\r", "\r\n")

        # Check for error response
        if response == "-":
            # Raise error without color codes to avoid issues with exception handling
            raise ValueError(f"Command {to_send} failed")
        else:
            # Check if response is a valid command
            print(
                f"Command sent: {to_send.strip()}  {GREY}Response: {response.strip()}{RESET}"
            )

        return response

    def send(self, send_str, cc=None, nn=None, mm=None, ee=None, set=False):
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
        # Determine which command type and set prefix accordingly
        if send_str in vars(Command).values():
            prefix = "!"
        elif send_str in vars(Query).values():
            prefix = "?"
        elif send_str in vars(Config).values():
            prefix = "^" if set else "~"
        else:
            raise ValueError(
                f"Invalid command: {send_str}. Must be a command from Command, Query, or Config classes"
            )

        # Create the initial command string with the appropriate prefix
        formatted_str = f"{prefix}{send_str}\r"

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
            eppr_response = self.send_raw("~EPPR 1")
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
    def linear_home_axis(self, axis, dio, direction="rev", speed=100):
        """
        Home an axis using a limit switch or sensor connected to a digital input

        Args:
            axis (int): Axis number
            dio (int): Digital input pin number for home sensor
            direction (str): Direction to move during homing ("fwd" or "rev")
            speed (int): Homing speed (default: 100)
            deceleration (int): Deceleration nn (default: 3000)

        Returns:
            bool: True if homing successful

        Raises:
            ValueError: If cc is not configured for linear movement or parameters are invalid
        """

        print(
            f"{YELLOW}Homing axis {axis} using digital input {dio} in {direction} direction at speed {speed}...{RESET}"
        )
        # Validate axis is configured for linear movement
        if (axis - 1) not in self.linear_config or not self.linear_config[axis - 1][
            "enabled"
        ]:
            raise ValueError(
                f"Axis {axis} is not configured for linear movement. Call configure_linear first."
            )

        # Validate direction
        if direction not in ["fwd", "rev"]:
            raise ValueError("Direction must be 'fwd' or 'rev'")

        # Validate DIO pin
        if not isinstance(dio, int) or dio < 0:
            raise ValueError("DIO pin must be a positive integer")

        # Set direction multiplier
        dir_multiplier = 1 if direction == "fwd" else -1

        # Change to open-loop speed mode
        self.send(Config.OPERATING_MODE, set=True, cc=axis, nn=0)
        # Set speed to 0 to start
        self.send(Command.GO_TO, cc=axis, nn=0)
        # Set acceleration
        self.send(Command.SET_ACCELERATION, cc=axis, nn=speed * 10)  # Speed up slowly
        # Set deceleration
        self.send(Command.SET_DECELERATION, cc=axis, nn=speed * 50)  # Slow down quickly

        # Start moving in specified direction
        speed_value = speed * dir_multiplier
        self.send(Command.GO_TO, cc=axis, nn=speed_value)
        print(f"Homing axis {axis} in {direction} direction at speed {speed}...")

        # Wait for home sensor (digital input) to trigger
        dio_state = 0
        while dio_state == 0:
            # Query the specific digital input
            response = self.send(Query.GET_DIN_SINGLE, cc=dio)

            # Parse response (format: "DI=0" or "DI=1")
            try:
                dio_state = self.extract_list(response)[0]
                dio_state = int(dio_state)

            except (IndexError, ValueError):
                raise ValueError(f"Invalid response format: {response}")
        # Get the current position and set as home
        self.linear_config[axis - 1]["home_count"] = self.extract_value(
            self.send(Query.GET_ENCODER_COUNTER, cc=axis)
        )

        # Mark axis as homed
        self.linear_config[axis - 1]["homed"] = True

        # Stop the motor
        self.send(Command.STOP_ALL, cc=axis)
        # Set speed to 0
        self.send(
            Command.SET_SPEED, cc=axis, nn=0
        )  # Put controller back in closed-loop count position mode
        self.send(Config.OPERATING_MODE, set=True, cc=axis, nn=3)

        print(f"{GREEN}Successfully homed {axis}. Position set to 0mm.{RESET}")

        return True

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
            bool: True if movement completed successfully

        Raises:
            ValueError: If axis is not configured for linear movement or not homed
        """
        # Validate axis is configured for linear movement
        if (axis - 1) not in self.linear_config or not self.linear_config[axis - 1][
            "enabled"
        ]:
            raise ValueError(
                f"{RED}Axis {axis} is not configured for linear movement. Call configure_linear first.{RESET}"
            )

        # Get the linear configuration
        config = self.linear_config[axis - 1]
        # Check if axis is homed
        if "home_count" not in config or config["home_count"] is None:
            raise ValueError(
                f"{RED}Axis {axis} is not homed. Call linear_home_axis first.{RESET}"
            )
        home_count = config["home_count"]
        points_per_rev = config["points_per_rev"]
        mm_per_rev = config["mm_per_rev"]

        # Check position limits if configured
        min_mm = config["min_mm"]
        max_mm = config["max_mm"]
        if min_mm is not None and position_mm < min_mm:
            raise ValueError(
                f"Target position {position_mm}mm is below minimum limit {min_mm}mm"
            )
        if max_mm is not None and position_mm > max_mm:
            raise ValueError(
                f"Target position {position_mm}mm is above maximum limit {max_mm}mm"
            )

        # Calculate target encoder position
        counts_per_mm = points_per_rev * mm_per_rev
        target_count = int(home_count) + int(position_mm * counts_per_mm)

        print(
            f"{YELLOW}Moving axis {axis} to position {position_mm} mm (encoder count: {target_count}){RESET}"
        )

        # Set speed if provided
        if speed is not None:
            self.send(Command.SET_SPEED, cc=axis, nn=speed)

        # Send position command
        self.send(Command.GO_TO_POSITION, cc=axis, nn=target_count)

        # Wait for movement to complete if requested
        if wait:
            print(f"{GREY}Waiting for axis {axis} to reach target position...{RESET}")
            # Loops while mf = 0
            mf = "DR=0"
            while mf == "DR=0":
                mf = self.send(Query.GET_DESTINATION_REACHED, cc=1, nn=0)

        return True

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
