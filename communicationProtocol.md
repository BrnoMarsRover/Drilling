# Protocol for communication between the drill's microcontroller and rover's master computer

All messages are sent via UART. All multi-byte variables are to be sent and received as big-endian - most significant byte is transmitted first.

1. The rover initiates communication by sending a single byte. This byte is the code of a function to be executed by the drill. In some cases, the code must be followed by an argument. This is usually a numeric value, for example the desired depth to be drilled.

2. If the drill is able to start performing the function, it immediately sends the function code back. If it is unable to perform the function, it sends back a zero.

3. After the action has been completed, the drill again sends the function code, sometimes followed by a numeric value - for example the mass of the extracted sample.

The last step can sometimes happen after a significant delay, for example after the vertical drive reaches a certain position. In other cases, the second response can arrive quickly. If the rover requests data which is readily available to the drill, the drill responds with function code as acknowledgement of the request. This is then almost immediately followed by the same function code, followed by the requested value.

## Example 1
Rover: 1 (stop)\
Drill: 1 (request to stop received, stopping)\
*a thousand centuries go by*\
Drill: 1 (drilling mechanism is in default position)

Rover: 3 (send vertical drive position)\
Drill: 3 (request received, fetching position)\
*you don't even have time to blink*\
Drill: 3 (position fetched, it will be sent in the next byte)\
Drill: 0 (The drive is in uppermost position)

|Name and description | Message from rover | Function argument from rover | Response data from drill |
|-                    |-                   |-                             |-                         |
| STOP - stops all drill's actions. Retracts the deep sample box. Returns the vertical drive to its uppermost position. Also serves to calibrate the vertical position value. | 1 | None | None |
| DRILL - Extracts a deep sample from specified depth | 2 | uint8 - desired drill depth [cm] | None |
| STATE - Requests the state of the drilling mechanism |  |  |  |
| HEIGHT - Requests the height of the vertical drive | 3 | None | uint8 - current distance from the uppermost position (higher number means lower height) [cm] |
| WEIGHT_DEEP - Requests the weight of the deep sample |   |   |  |
| WEIGHT_SURF - Requests the weight of the surface sample |   |   |  |
