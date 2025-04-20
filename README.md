# Roboteq Controller Library

A comprehensive Python library for controlling Roboteq motor controllers through serial or TCP connections.

## File Overview

### roboteq_manager.py
This is the core driver class that provides direct communication with Roboteq motor controllers.

- Establishes and manages connections via serial or TCP
- Sends commands and queries to the controller
- Implements linear motion control features with position tracking
- Provides homing functionality for precise positioning
- Auto-discovers devices on networks and serial ports

### commands.py
Contains structured definitions of all available controller commands organized into classes:

- **Command**: Runtime control commands (speed control, position control, etc.)
- **Query**: Feedback and status reading commands
- **Config**: Configuration parameters for controller settings

### start_roboteq.py
The main application entry point that manages device connections:

- Scans for available devices on serial ports and network
- Loads and saves device configurations to a JSON file
- Establishes connections to configured devices
- Provides a command-line interface for device management
- Handles device discovery and connection workflows

### user_program.py
Contains the logic for running custom control sequences on connected devices:

- Provides a framework to execute user-defined motor control programs
- Includes helper functions to find devices by name
- Demonstrates proper usage of the driver API

## Getting Started

1. Run `start_roboteq.py` to scan for and configure controllers
2. The program will save device configurations to `config.json`
3. Create custom control sequences in `user_program.py`
4. Connect to your devices and start controlling your motors

## Connection Types

The library supports two connection methods:

- **Serial**: Direct USB or RS232 connection to the controller
- **TCP**: Network connection to controllers with Ethernet capability

## Example Usage

### Device Configuration

Devices are configured in `config.json` with settings like:

- Connection type (serial or TCP)
- Port or IP address
- Number of axes
- Device name for identification