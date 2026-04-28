# Motion-Controller-Python-Lib
Python CANopen helper library for FOC motor controller developed by Medplus India Pvt Ltd
# FOC Motion Controller Python Library

## Overview

This repository provides a Python3-based CANopen helper library to control a Field-Oriented Control (FOC) BLDC motor driver developed by **Medplus**.

The motor driver communicates over **CAN 2.0B**, and the implementation follows the **CANopen protocol** wherever applicable. This library simplifies interaction with the motor driver by abstracting CANopen communication and exposing a clean Python interface.

For deeper understanding of the device-specific CANopen implementation, refer to:

* `Motion Controller Reference.pdf`

This document can also be used to implement your own controller in other languages such as **C/C++**.

For general CANopen concepts and usage, refer to the official **python-canopen documentation**, which provides a solid explanation of the protocol.

---

## Repository Structure

```
MotionController_Python_Lib/
│
├── motor_driver.py              # Core Python3 library for the Medplus FOC BLDC motor driver
├── control_motor.py             # Primary example: position control using motor_driver.py
├── send_velocity.py             # Primary example: velocity control using motor_driver.py
├── DS301_profile.eds            # CANopen EDS file
├── Motion Controller Reference.pdf
│                              # Device-specific CANopen reference document
│
├── examples/                    # Optional folder for additional motor examples
├── docs/                        # Additional documentation
└── README.md
```

---

## Core Library

### motor_driver.py

This is the **main library file** that implements:

* CANopen communication (SDO/PDO)
* Object dictionary interaction
* Motor control interface abstraction

It acts as the bridge between:

* Python application layer
* CANopen stack
* FOC motor driver firmware

---

## Features

* CAN 2.0B communication support
* CANopen protocol implementation
* Abstraction over low-level CANopen operations
* Motor control interface (enable, disable, Position control)
* Extensible design for additional control modes

---

## Installation

### Requirements

* Python 3.x
* python-can
* canopen
* Linux SocketCAN support
* CAN interface hardware

This library was tested using a USB-to-CAN controller (CANAble) over SocketCAN.

You may also use:

* A dedicated CAN port, if your device has one
* A Waveshare CAN HAT for Raspberry Pi
* Any CAN adapter supported by SocketCAN

Note: The motor driver CAN baud rate is fixed at 500 kbps.

### Install dependencies

```bash
pip install python-can canopen
```
Install CAN utilities

On Ubuntu/Debian-based systems:

sudo apt update
sudo apt install net-tools can-utils

net-tools provides utilities such as ifconfig.

can-utils provides useful CAN debugging tools such as:

candump
cansend
cansniffer
SocketCAN Setup
1. Check CAN interface

After connecting your CAN adapter, check available network interfaces:

ifconfig -a

You should see an interface such as:

can0
2. Turn on CAN interface

Set the CAN interface bitrate to 500 kbps and bring it up:

sudo ip link set can0 type can bitrate 500000 && sudo ip link set can0 up
3. Verify CAN traffic

Use candump to monitor CAN messages:

candump can0

If the motor driver is connected and powered correctly, you should see periodic heartbeat messages from the motor driver.

Example:

can0  702   [1]  05

In this example:

702 indicates a heartbeat COB-ID from node ID 2
05 commonly indicates the node is in operational state

The exact node ID and state byte may vary depending on your configuration.
---

## Quick Start

The main library file is `motor_driver.py`. The files `control_motor.py` and `send_velocity.py` are the primary examples showing how to use this library.

Before running the examples, install the required Python packages:

```bash
pip install python-can canopen
```

Bring up the CAN interface. Example for SocketCAN on Linux:

```bash
 sudo ip link set can0 type can bitrate 500000 && sudo ip link set can0 up 
```

### Example: Position Control

The following example is based on `control_motor.py`.

```python
from motor_driver import Motor_Driver
import time

# Create and connect to the motor controller node
node_id = 2

drv = Motor_Driver(
    node_id=node_id,
    eds_path="DS301_profile.eds",
    bustype="socketcan",
    channel="can0"
)

print("Connecting to node...")
drv.connect()
print(f"NMT state: {drv.state}")

# Configure heartbeat if supported by the drive
try:
    drv.set_heartbeat(2000)  # Producer heartbeat time in ms
    print("Heartbeat configured.")
except Exception as e:
    print(f"Heartbeat skipped: {e}")

# Set mode of operation
# mode = 1 is used here for position control
try:
    drv.set_mode_of_operation(mode=1)
    print("Position control mode entered")
except Exception as e:
    print(f"Error, not able to set mode: {e}")

# Set controller gains
try:
    drv.set_iq_gains(kp=0.135, ki=510.790359)
    print("Iq gains set")
except Exception as e:
    print(f"Iq gains skipped: {e}")

try:
    drv.set_velocity_gains(kp=int(7.15 * 100), ki=int(257.44 * 100))
    print("Velocity gains set successfully")
except Exception as e:
    print(f"Velocity gains skipped: {e}")

try:
    drv.set_position_gains(kp=int(16 * 100))
    print("Position gains set successfully")
except Exception as e:
    print(f"Position gains skipped: {e}")

# Set acceleration and deceleration
# Value is in rps² multiplied by 100 before sending
try:
    drv.set_accel_decel(accel=10 * 100, decel=10 * 100)
except Exception as e:
    print(f"Unable to set acceleration/deceleration: {e}")

# Switch node to operational state
drv.to_operational()
print("Node switched to OPERATIONAL state.")

# Enable the drive
drv.set_controlword(0x000F)
print("Controlword set to 0x000F. Drive started.")

# Command target position and velocity
# position is in rotations, velocity is in RPM
drv.set_target_position_and_velocity(
    position=0,
    velocity=500.0,
    force_pdo=True
)
print("Moving to zero position.")

time.sleep(2)

# Move between two target positions
for i in range(20):
    if i % 2 == 0:
        print("Cycle:", i)
        drv.set_target_position_and_velocity(
            position=-20.0,
            velocity=1000.0,
            force_pdo=True
        )
        print("Moving to -20 rotations")
    else:
        drv.set_target_position_and_velocity(
            position=0,
            velocity=1000.0,
            force_pdo=True
        )
        print("Moving to 0 rotations")

    time.sleep((20 * 60 / 500) + 2)
```

### Example: Velocity Control

`send_velocity.py` demonstrates continuous velocity control. It connects to the motor driver, configures control parameters, switches the node to operational state, enables the drive, and continuously sends a velocity command.

```bash
python3 send_velocity.py
```

---

## CANopen Notes

* Communication is based on **CAN 2.0B**
* CANopen is implemented partially (device-specific behavior applies)
* Refer to `Motion Controller Reference.pdf` for:

  * Object dictionary details
  * Control/state machine behavior
  * Supported modes of operation

---

## Primary Examples

### control_motor.py

`control_motor.py` is the main position-control example. It demonstrates:

* Connecting to the CANopen node
* Setting heartbeat
* Selecting position-control mode
* Setting Iq, velocity, and position controller gains
* Setting acceleration/deceleration
* Switching the node to operational state
* Enabling the drive using the controlword
* Sending target position and velocity commands using PDO

### send_velocity.py

`send_velocity.py` is the main velocity-control example. It demonstrates:

* Connecting to the CANopen node
* Selecting velocity-control mode
* Setting controller gains
* Switching to operational state
* Enabling the drive
* Sending continuous velocity commands

---

## Intended Usage

This repository is intended as:

* A **reference implementation** for controlling the Medplus FOC motor driver
* A **starting point** for building custom control applications
* A **learning resource** for CANopen-based motor control systems

---

## Future Improvements

* Logging and debugging tools

---

## License


---

## Contact

(Add your contact or team details here)
