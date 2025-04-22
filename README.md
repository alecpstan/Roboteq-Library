# Roboteq Controller Library

A comprehensive Python library for controlling Roboteq motor controllers through serial or TCP connections.

## File Overview

### roboteq_manager.py
Core driver class that handles communication with Roboteq motor controllers.

- Manages connections via serial or TCP
- Sends commands and queries to the controller
- Implements motion control features with position tracking
- Provides homing functionality
- Discovers devices across networks and serial ports

### commands.py
Defines structured controller commands organized into classes:

- **Command**: Runtime control commands for motors
- **Query**: Status and feedback information requests
- **Config**: Configuration parameters for controller settings

### start_roboteq.py
Application entry point for device connection management:

- Scans for available devices
- Manages device configurations via JSON file
- Establishes connections to controllers
- Provides interface for device management

### user_program.py
Framework for implementing custom control sequences:

- Executes user-defined motor control programs
- Contains helper functions for device management
- Demonstrates API usage patterns

## Getting Started

1. Run `start_roboteq.py` to scan for and configure controllers
2. Device configurations will be saved to `config.json`
3. Create custom control sequences in `user_program.py`
4. Connect to your devices and control your motors

## Connection Types

The library supports two connection methods:

- **Serial**: Direct USB or RS232 connection
- **TCP**: Network connection to controllers with Ethernet capability

## Example Usage

### Device Configuration

Devices are configured in `config.json` with settings like:

- Connection type (serial or TCP)
- Port or IP address
- Number of axes
- Device name for identification
