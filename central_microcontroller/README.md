# Drill RPi Node (Micro-ROS on Raspberry Pi Pico for Rover Freya Drill)

## Introduction
This project integrates Micro-ROS on a Raspberry Pi Pico to serve as the central microcontroller for the drill system of our rover Freya.
The Pico communicates over I2C with subsystems responsible for controlling linear actuators, the drilling motor, and the storage mechanism.
Acting as a lightweight and efficient control unit, it ensures smooth and reliable coordination of all subsystems.

This project is based on the [micro_ros_raspberrypi_pico_sdk](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk) by the Micro-ROS team, and extends it with custom functionality for the rover's drilling operations.

## Features
- Integration of Micro-ROS on Raspberry Pi Pico.
- USB or UART communication with Mars Rover.
- I2C communication with subsystems.
- Subscribes to `/joy`, `/drill_state`, and `/drill_parameters`.
- Publishes to `/drill_data`.

## Hardware Requirements
- Central microcontroller and power distribution board v0.2.0.
- Power supply 24V.
- Raspberry Pi Pico.

## Software Requirements
- Pico SDK (C/C++).
- ROS 2 (tested with Humble/rolling).
- `joy` package from joystick_drivers repo.
- Micro-ROS client library.
- Micro-ROS agent (can be run locally or in Docker).

## Installation
Same as here https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git


## Usage
- Connect the Pico to the computer via USB or UART. For connecting via UART first enable it in CMakeLists.txt.
- Run the Micro-ROS agent:
    ```bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
    ```
- Run joystick driver:
    ```bash
    ros2 run joy joy_node
    ```
- For manual controll:
    ```bash
    ros2 topic pub --once /drill_state std_msgs/msg/UInt8 data:\ 9\ 
    ```

## Project Structure
    ```bash
├── CMakeLists.txt
├── LICENSE
├── libmicroros     # MicroROS precompiled library for RPi Pico
├── src/
│   ├── pico_micro_ros_drill.c  # main, node init, i2c init, callbacks
│   ├── linear_driver.c
│   ├── motor_driver.c
│   ├── storage_driver.c
│   └── pico_uart_transport.c   # Original c-file from MicroROS, UART transport handling
└── include/
    ├── linear_driver.h         # Driver for linear subsystem
    ├── motor_driver.h          # Driver for motor subsystem
    ├── storage_driver.h        # Driver for storage unit subsystem
    └── pico_uart_transports.h  # Original header from MicroROS, UART transport handling
    ```

## Communication

### Topics

| Topic Name           | Type                              | Direction | Description                              |
|----------------------|-----------------------------------|-----------|------------------------------------------|
| `/joy`               | `sensor_msgs/msg/Joy`             | Input     | Commandsfromjoystick.                    |
| `/drill_state`       | `std_msgs/msg/UInt8`              | Input     | Selection of drill state.                |
| `/drill_parameters`  | `std_msgs/msg/UInt16MultiArray`   | Input     | Sets the drill parameters.               |
| `/drill_data`        | `std_msgs/msg/UInt16MultiArray`   | Output    | Published data from the drill subsystems.|

#### Drill States - Meaning

| State | Name              | Description                                       |
|-------|-------------------|---------------------------------------------------|
| 0     | Stop              | Stops all subsystems.                             |
| 1     | Drilling          | Rotates clockwise and moves downwards.            |
| 2     | Move to Position  | Moves to position without rotating the drill.     |
| 3     | Rotate Right      | Rotates clockwise in a static position.           |
| 4     | Rotate Left       | Rotates counterclockwise in a static position.    |
| 5     | Select Slot       | Rotates the storage to the selected position.     |
| 6     | Tare Weight       | Tares (zeros) the current weight of the storage.  |
| 7     | Weigh             | Weighs and stores the sample at the selected slot.|
| 8     | Reset Weights     | Resets the memory of stored samples.              |
| 9     | Manual Mode       | Direct control via `/joy` topic.                  |

#### Drill Parameters - Detail

| Index | Name               | Description                                                                     |
|-------|--------------------|---------------------------------------------------------------------------------|
| 0     | Spiral Speed       | Maximum rotational speed of the spiral [RPS], divided by a coefficient of 0.03. |
| 1     | Spiral Height      | Desired spiral height [mm].                                                     |
| 2     | Storage Slot       | Specification of the storage slot (e.g., for positioning or storing weight).    |

#### Drill Data - Detail

| Index   | Name                    | Description                                                           |
|---------|-------------------------|-----------------------------------------------------------------------|
| 0       | Status                  | Details described in the Drill Status table.                          |
| 1       | Spiral Speed            | Measured rotational speed of the spiral [RPS], divided by 0.03.       |
| 2       | Drilling Torque         | Measured drilling torque of the spiral [Nm], divided by 0.03.         |
| 3       | Motor Temperature       | Temperature of the DC motor [°C].                                     |
| 4       | Spiral Height           | Measured height of the spiral [mm].                                   |
| 5       | Structure Height        | Height of the structure perpendicular to the surface [mm].            |
| 6       | Storage Slot            | Currently active storage slot.                                        |
| 7-10    | Slot Weights            | Weight on each storage slot (slot 1 → index 7, etc.), divided by 0.1. |

#### Drill Status Bits (Index 0 in `/drill_data`)

| Subsystem         | Bit(s)  | Meaning                                                                                |
|-------------------|---------|----------------------------------------------------------------------------------------|
| -                 | 15:12   | Not used                                                                               |
| **Storage System**| 11      | 0 = Disconnected, 1 = Connected                                                        |
|                   | 10      | 0 = Weight not zeroed, 1 = Weight zeroed                                               |
|                   | 9:8     | 00 = No error, 01 = Maximum weight exceeded, 10 = Unknown command, 11 = Slot not found |
| **Linear Motion** | 7       | 0 = Disconnected, 1 = Connected                                                        |
|                   | 6:4     | 0 = No error, >0 = Error code                                                          |
| **DC Motor**      | 3       | 0 = Disconnected, 1 = Connected                                                        |
|                   | 2       | 0 = Spiral not blocked, 1 = Spiral blocked                                             |
|                   | 1:0     | 00 = No error, 01 = H-bridge fault                                                     |

### I2C

| I/O  | Byte | Name       | Type        | Range      | Description                                  |
|------|------|------------|-------------|------------|----------------------------------------------|
| **0x0A DC Motor Control**                                                                          |
| IN   | 0    | Speed      | `int8_t`    | -100…100   | RPS multiplied by 0.03                       |
| OUT  | 0    | Status     | `uint8_t`   | 0…255      | Status                                       |
|      | 1    | Speed      | `int8_t`    | -100…100   | RPS multiplied by 0.03                       |
|      | 2    | Torque     | `int8_t`    | -83…83     | Torque (Nm) multiplied by 0.03               |
|      | 3    | Temperature| `uint8_t`   | 0…255      | Motor temperature (°C)                       |
| **0x09 Linear Actuator Control**                                                                   |
| IN   | 0    | Command    | `uint8_t`   | 0…4        | Calibration, stop, move down, move up        |
|      | 1    | Speed      | `uint8_t`   | 0…255      | Step speed                                   |
| OUT  | 0    | Status     | `uint8_t`   | 0…255      | Status                                       |
|      | 1    | Height     | `uint16_t`  | 0…500      | Distance from reference (mm)                 |
|      | 2    | -          | -           | -          | -                                            |
|      | 3    | Ground Height | `uint16_t`| 0…65535    | Distance to the ground (mm)                 |
|      | 4    | -          | -           | -          | -                                            |
| **0x08 Storage System**                                                                            |
| IN   | 0    | Command    | `uint8_t`   | 0…255      | Rotation, zeroing, weighing                  |
| OUT  | 0    | Weight     | `uint16_t`  | 0…65535    | Weight (g) multiplied by 0.1                 |
|      | 1    | -          | -           | -          | -                                            |
|      | 2    | Status     | `uint8_t`   | 0…255      | Status                                       |



## Acknowledgments
- [Micro-ROS](https://micro-ros.github.io/) project for providing the base integration of Micro-ROS on Raspberry Pi Pico.
- [micro_ros_raspberrypi_pico_sdk](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk) as the foundation for this project's firmware.

## License

This project is licensed under the [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0).

It is based on the [micro_ros_raspberrypi_pico_sdk](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk) by the Micro-ROS team, which is also licensed under the Apache License 2.0.

All modifications, extensions, and customizations specific to the drill control system for Rover Freya are provided under the same license.

Copyright 2025.
