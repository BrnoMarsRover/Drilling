# Protocol for communication between the drill's microcontroller and rover's master computer

All messages are sent via UART.\
All multi-byte variables are to be sent and received as big-endian - most significant byte is transmitted first.

1. The rover initiates communication by sending a single byte. This byte is the code of a function to be executed by the drill. In some cases, the code must be followed by an argument. This is usually a numeric value, for example the desired depth to be drilled.

2. If the drill is able to start performing the function, it sends the function code back. If it is unable to perform the function, it sends back a zero. If further answer is expected (for example the weight of a sample), the function code is followed by the requested value.

## Example 1
Rover: 0x01 (stop)\
Drill: 0x01 (request to stop received, stopping)

Rover: 0x03 (send vertical drive position)\
Drill: 0x03 0x00 (request received, sending position: 0 cm - default position)

|Name and description | Message from rover | Function argument from rover | Response data from drill |
|-                    |-                   |-                             |-                         |
| STOP - stops all drill's actions. Restarts the microcontroller. Retracts the deep sample box. Returns the vertical drive to its uppermost position. Also calibrates the vertical position value. | 1 | None | None |
| DRILL - Extract a deep sample from specified depth | 2 | uint8 - desired drill depth [cm] | None |
| STATE - Requests the state of the drilling mechanism | 3 | None | uint8_t [code of the current state] |
| HEIGHT - Requests the height of the vertical drive | 4 | None | uint8 - current distance from the uppermost position (higher number means lower height) [cm] |
| WEIGHT_DEEP - Requests the weight of the deep sample | 5 | None | float [grams] |
| WEIGHT_SURF - Requests the weight of the surface sample | 6 | None | float [grams] |
| SURF_OPEN - opens the surface sample box  | 7 | None | None |
| SURF_CLOSE - closes the surface sample box  | 8 | None | None |
