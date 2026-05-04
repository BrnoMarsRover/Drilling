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
Rover: 0x02 (start) -> 0x01 (length 1) -> 0x01 (message - stop) -> 0x?? (checksum) -> 0x03 (end)\
Drill: 0x02 (start) -> 0x01 (length 1) -> 0x01 (message - request to stop received, stopping) -> 0xidk (checksum) -> 0x03 (end) 

## Example 2
Rover: ...0x03... (send vertical drive position)\
Drill: ...0x03 (request received) -> 0x00 (position: 0 cm)...

|Name and description | Message from rover | Function argument from rover | Response data from drill |
|-                    |-                   |-                             |-                         |
| STOP - Restarts the microcontroller. Retracts the deep sample box. Returns the vertical drive to its uppermost position. Also calibrates the vertical position value. | 1 | None | None |
| DRILL - Extract a deep sample from specified depth | 2 | uint8 - desired drill depth [cm] | None |
| STATE - Requests the state of the drilling mechanism | 3 | None | uint8 [code of the current state] |
| HEIGHT - Requests the current height of the vertical drive | 4 | None | uint8 - current distance from the uppermost position (higher number means lower height) [cm] |
| WEIGHT DEEP - Requests the weight of the deep sample | 5 | None | float [grams] |
| WEIGHT SURFACE - Requests the weight of the surface sample | 6 | None | float [grams] |
| DRILL SPEED - sets the speed of the drill/spiral | 100 | int8 [RPM] | None |
| VERTICAL SPEED - sets the speed of the vertical drive| 101 | int8 [mm/s] | None |
| STORAGE POSITION - sets the position of the deep sample storage box | 102 | uint8 [position] | None |
| WEIGH DEEP - Start weighing the deep sample | 103 | None | None |
| WEIGH SURFACE - Start weighing the surface sample | 104 | None | None |
| ROCK OPEN - opens the rock sample box | 252 | None | None |
| ROCK CLOSE - closes the rock sample box | 253 | None | None |
| SAND OPEN - opens the sand sample box  | 254 | None | None |
| SAND CLOSE - closes the sand sample box  | 255 | None | None |
