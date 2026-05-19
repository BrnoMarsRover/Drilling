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
Rover: 0x02 (start) -> 0x01 (length 1) -> 0x98 (GET WEIGHT DEEP) -> 0x?? (checksum) -> 0x03 (end)\
Drill: 0x02 (start) -> 0x01 (length 5) -> 0x98 (Weight request received. Weight follows.) -> 4 bytes-float -> 0x?? (checksum) -> 0x03 (end)

| Name and description | Message from rover | Function argument from rover | Response data from drill |
|-                     |-                   |-                             |-                         |
| RESTART - Restarts the microcontroller. Retracts the deep sample box. Returns the vertical drive to its uppermost position. Also calibrates the vertical position value. | 1 | None | None |
| STATE - Requests the state of the drilling mechanism | 2 | None | Response is in table below |
| DRILL AUTO - Automatically extract a deep sample from specified depth. Blocks manual commands. | 3 | Desired drill depth - uint8 [cm] | None |
| STOP AUTO - Stops the automatic drilling procedure. Unlocks manual commands. | 4 | None | None |
| DRILL SPEED - sets the speed of the drill/spiral | 100 | int16 [RPM] | None |
| VERTICAL SPEED - sets the speed of the vertical drive| 101 | int8 [mm/s] | None |
| STORAGE POSITION - sets the position of the deep sample storage box | 102 | uint8 [position] | None |
| WEIGH DEEP - Start weighing the deep sample | 150 | None | None |
| WEIGH SURFACE - Start weighing the surface sample | 151 | None | None |
| GET WEIGHT DEEP - Requests the weight of the deep sample | 152 | None | float [grams] |
| GET WEIGHT SURFACE - Requests the weight of the surface sample | 153 | None | float [grams] |
| ROCK OPEN - opens the rock sample box | 200 | None | None |
| ROCK CLOSE - closes the rock sample box | 201 | None | None |
| SAND OPEN - opens the sand sample box  | 202 | None | None |
| SAND CLOSE - closes the sand sample box  | 203 | None | None |

STATE response table
| Variable meaning | Data type | Unit |
|-                 |-          |-     |
| Current distance of the carriage from uppermost position | uint8 | cm |
| Motor speed | int16 | RPM |
| Motor temperature | uint8 | °C |
| Tray angle | uint16 | ° |
| Software state | uint8 | Code of the state - meaning in table below yet again :) |

Software state codes 
| Code | Meaning |
|-     |-        |
| 0 | Drill is initializing. |
| 1 | Error. Try restarting. |
| 2 | Device ready. Automatic drilling procedure disabled. |
| 100 | Automatic procedure active. Drilling down. |
| 101 | Automatic procedure active. Could not reach the desired depth. |
| 102 | Automatic procedure active. Desired depth reached. Moving up. |
| 103 | Automatic procedure active. Storing the sample. |
