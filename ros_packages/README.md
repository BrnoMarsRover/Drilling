# Drill packages for Freya Rover Drill

## Introduction
Packages for the drilling kit.  
The `drill_controller` node exposes actions and services and controls a Raspberry Pi Pico centrall microcontroller of drilling kit via topics.

---

## Project Structure

```bash
├── drill       # Package for node drill_controller
│   ├── CMakeLists.txt
│   ├── include
│   │   └── drill
│   │       ├── drill_controller.h  # Header for node
│   │       ├── drill_logger.h      # Header for logging into th .txt file
│   │       └── drill_status.h      # Header for handling drill status register
│   ├── LICENSE
│   ├── package.xml
│   └── src
│       ├── drill_controller.cpp
│       ├── drill_logger.cpp
│       └── drill_status.cpp
└── drill_interfaces        # Package for interfaces (actions, services)
    ├── action
    │   ├── DrillCalibration.action 
    │   ├── DrillSample.action
    │   └── StoreSample.action
    ├── CMakeLists.txt
    ├── include
    │   └── drill_interfaces
    ├── LICENSE
    ├── package.xml
    ├── src
    └── srv
        ├── GetDrillStatus.srv
        └── GetSampleWeight.srv
```


## Actions
For the actual interface, check `/drill_interfaces/action`.

- Drill Calibration - Action used to calibrate the linear actuator of the drill and reset weights (optional). If the input is true, it resets (zeros) the weights; otherwise, it only calibrates the linear actuator and measures the construction height from the ground.

- Drill Sample - Performs a complete drilling sequence to extract a sample. The drill moves down, collects material, and retracts. The sequence ends if the auger or the linear actuator is overloaded. Inputs are the maximum rotation speed (ideal 2 RPS) and the target drilling depth (typically around 300 mm).

- Store Sample - Stores the collected sample into the storage subsystem. The storage slot can be selected according to a predefined table. Action return weight of the sample.

### Selecting where to store sample

| **Slot Number** | **Action**              | **Description**                                               |
|-----------------|-------------------------|---------------------------------------------------------------|
| 0               | Keep in tube            | The sample remains in the tube.                               |
| 1-4             | Store in respective slot | The sample is stored in the respective storage slot (1-4).   |
| 5 or higher     | Discard the sample      | The sample is discarded (no storage, just emptying).          |

## Services
For the actual interface, check `/drill_interfaces/srv`.

- Get Sample Weight - Service for getting the weight of a sample independently of the "store sample" action.
The client sends a slot number, and the service returns the weight assigned to that slot.

- Get Drill Status - Service that provides the status of the drilling tool and reports the state of its subsystems. e.g.

| Motor                 | Linear               | Storage               |
|:----------------------|:---------------------|:----------------------|
| i2c: `CONNECTED`       | i2c: `CONNECTED`      | i2c: `DISCONNECTED`    |
| stucked: `FREE`        | error: `NO ERROR`     | scale: `-`             |
| error: `H-BRIDGE ERROR`|                       | error: `-`             |

## Topics
Described in centrall_microcontroller project.