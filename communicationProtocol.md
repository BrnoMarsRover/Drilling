# Protocol for communication between the drill's microcontroller and rover's master computer

All messages are sent via UART with 38400 baud rate.\
All multi-byte variables are to be sent and received as big-endian - most significant byte is transmitted first.

1. The rover initiates communication by sending a message in the specified format. The payload of the message consists of a code of the function to be executed by the drill. The code is of type uint8. In some cases, the code must be followed by an argument. This is usually a numeric value, for example the desired depth to be drilled.

2. If the drill is able to start performing the function, it sends the function code back. If it is unable to perform the function, it sends back a zero. If further answer is expected (for example the weight of a sample), the function code is followed by the requested value.

## Message format
|Byte number  |Content         |
|-            |-|
|1            |0x02            |
|2            |Payload length  |
|3...n-2      |Payload  |
|n-1          |Checksum |
|n            |0x03     |

The chekcsum algorithm is "Sum complement". See: https://en.wikipedia.org/wiki/Checksum#Sum_complement
The checksum is computed only from the payload.

## Example 1
Rover: 0x02 (start) -> 0x01 (length 1) -> 0x01 (message - reset) -> 0x?? (checksum) -> 0x03 (end)\
Drill: 0x02 (start) -> 0x01 (length 1) -> 0x01 (message - request to reset received. Resetting.) -> 0x?? (checksum) -> 0x03 (end) 

## Example 2
Rover: 0x02 (start) -> 0x01 (length 1) -> 0x42 (GET WEIGHT DEEP) -> 0x?? (checksum) -> 0x03 (end)\
Drill: 0x02 (start) -> 0x01 (length 5) -> 0x42 (Weight request received. Weight follows.) -> 4 bytes-float -> 0x?? (checksum) -> 0x03 (end)

| Name and description | Message from rover | Function argument from rover | Response data from drill |
|-                     |-                   |-                             |-                         |
| RESTART - Restarts the microcontroller. Retracts the deep sample box. Returns the vertical drive to its uppermost position. Also calibrates the vertical position value. | 0x01 | None | None |
| STATE - Requests the state of the drilling mechanism | 0x02 | None | Response is in table below |
| DRILL AUTO - Automatically extract a deep sample from specified depth. Blocks manual commands. | 0x03 | Desired drill depth - uint8 [cm] | None |
| STOP AUTO - Stops the automatic drilling procedure. Unlocks manual commands. | 0x04 | None | None |
| CALIBRATE CARRIAGE DEPTH - Moves the carriage up, until it hits the top limit switch. Sets depth = 0 at that position. | 0x05 | None | None |
| START DEVICE CHECK - Checks whether peripheral devices are connected and responding. | 0x06 | None | None |
| GET DEVICE STATUS - Requests the result of the START DEVICE CHECK | 0x07 | None | uint16 (String of bits. Each bit corresponds to one peripheral device. 1 = OK, 0 = not OK. Order of devices in table below) |
| DRILL SPEED - sets the speed of the drill/spiral | 0x20 | int16 [RPM] | None |
| VERTICAL SPEED - sets the speed of the vertical drive | 0x21 | int8 [0,1 mm/s]<br>e.g.&nbsp;100 = 10mm/s | None |
| STORAGE POSITION - sets the position of the deep sample storage box | 0x22 | uint8 [position] | None |
| MEASURE HEIGHT ABOVE GROUND - requests height measurement. | 0x23 | None | None |
| GET HEIGHT ABOVE GROUND - requests the value of the last height measurement. | 0x24 | None | uint16 [mm] |
| WEIGH DEEP - Start weighing the deep sample. | 0x40 | None | None |
| WEIGH SURFACE - Start weighing the surface sample. | 0x41 | None | None |
| GET WEIGHT DEEP - Requests the result of WEIGH DEEP. | 0x42 | None | float [grams], uint32 [raw ADC value] |
| GET WEIGHT SURFACE - Requests the result of WEIGH SURFACE. | 0x43 | None | float [grams], uint32 [raw ADC value] |
| CALIBRATE 0 DEEP - Starts the calibration procedure. Calibrates the ADC value for empty storage. Needs to be followed by CALIBRATE X DEEP for successful calibration. | 0x44 | None | None |
| CALIBRATE X DEEP - Calibrates the ADC value for X grams in storage. Finishes calibration and saves data to nonvolatile flash memory. | 0x45 | Weight of the object put into storage -  float [grams] | None |
| CALIBRATE 0 SURFACE - Same as above, but for surface sample. | 0x46 | None | None |
| CALIBRATE X SURFACE - Same as above, but for surface sample. | 0x47 | Weight of the object put into storage -  float [grams] | None |
| ROCK OPEN - opens the rock sample box | 0x50 | None | None |
| ROCK CLOSE - closes the rock sample box | 0x51 | None | None |
| SAND OPEN - opens the sand sample box  | 0x52 | None | None |
| SAND CLOSE - closes the sand sample box  | 0x53 | None | None |

STATE response table
| Variable meaning | Data type | Unit |
|-                 |-          |-     |
| Current distance of the carriage from uppermost position | int16 | mm |
| Vertical drive speed | int8 | 0,1 mm/s (10 = 1 mm/s) |
| Vertical drive stepper current | uint8 | 0,01 A (100 = 1 A) |
| Spiral motor speed | int16 | RPM |
| Spiral motor temperature | uint8 | °C |
| Deep sample storage angle | uint16 | ° |
| Software state | uint8 | Code of the state - meaning in table below yet again :) |

Software state codes 
| Code | Meaning |
|-     |-        |
| 0x00 | Drill is initializing. |
| 0x01 | Error. Try restarting. |
| 0x02 | Device ready. Automatic drilling procedure disabled. |
| 0xF0 | Automatic procedure active. Drilling down. |
| 0xF1 | Automatic procedure active. Could not reach the desired depth. |
| 0xF2 | Automatic procedure active. Desired depth reached. Moving up. |
| 0xF3 | Automatic procedure active. Storing the sample. |

CHECK DEVICES response order
| Order | Device |
| - | - |
| least significant bit - 0 | Vertical drive stepper driver |
| 1 | Vertical drive encoder |
| 2 | Vertical drive current sensor |
| 3 | Spiral motor |
| 4 | Height sensor |
| 5 | Deep sample storage stepper driver |
| 6 | Deep sample storage encoder |
| 7 | Deep sample ADC |
| 8 | Surface sample ADC |
