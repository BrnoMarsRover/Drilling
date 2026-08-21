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

## Communication program
A program has been developed for communicating with the drill according to this protocol. The program supports two modes of communication
1. The drill can be connected directly to a computer running the program via USB. This is simpler and meant for testing purposes when the drill is not mounted on the rover.
2. When the drill is mounted and connected to the rover, direct USB communication with a PC is not possible. A broader, main program exists for wireless communication with the rover. Our drill communication program then runs on the same computer as the main program. These two programs communicate via UDP localhost. The main program and the rover's computer then act as middlemen between this app and the drill itself. The drill program should send messages to port 5610 and can receive the drill's responses on port 5611.

## Example 1
Rover: 0x02 (start) -> 0x01 (length 1) -> 0x01 (message - reset) -> 0x?? (checksum) -> 0x03 (end)\
Drill: 0x02 (start) -> 0x01 (length 1) -> 0x01 (message - request to reset received. Resetting.) -> 0x?? (checksum) -> 0x03 (end) 

## Example 2
Rover: 0x02 (start) -> 0x01 (length 1) -> 0x42 (GET WEIGHT DEEP) -> 0x?? (checksum) -> 0x03 (end)\
Drill: 0x02 (start) -> 0x01 (length 5) -> 0x42 (Weight request received. Weight follows.) -> 4 bytes-float -> 0x?? (checksum) -> 0x03 (end)

## Function list
| Name and description | Message from rover | Function argument from rover | Response data from drill |
|-                     |-                   |-                             |-                         |
| RESTART - Restarts the microcontroller. Retracts the deep sample box. Returns the vertical drive to its uppermost position. Also calibrates the vertical position value. | 0x01 | None | None |
| STATE - Requests the state of the drilling mechanism | 0x02 | None | Response is in table below |
| CALIBRATE CARRIAGE DEPTH - Moves the carriage up, until it hits the top limit switch. Sets depth = 0 at that position. | 0x03 | None | None |
| START DEVICE CHECK - Checks whether peripheral devices are connected and responding. | 0x04 | None | None |
| GET DEVICE STATUS - Requests the result of the START DEVICE CHECK | 0x05 | None | uint16 (String of bits. Each bit corresponds to one peripheral device. 1 = OK, 0 = not OK. Order of devices in table below) |
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
| SET HOLD MODE - sets hold mode on the deep storage - locks it in place, but continuously draws current. | 0x54 | None | None |
| CLEAR HOLD MODE - clears hold mode on the deep storage. | 0x55 | None | None |
| STOP AUTO - Stops all automatic procedures. Unlocks manual commands. | 0x60 | None | None |
| DRILL AUTO - Automatically extract, store and weigh a deep sample from specified depth. Blocks manual commands. | 0x61 | Desired drill depth - uint8 [cm] | None |
| STORE AUTO - Automatically store and weigh a sample that is already in the spiral. Blocks manual commands. | 0x62 | None | None |

## STATE response table
| Variable meaning | Data type | Unit |
|-                 |-          |-     |
| Carriage depth - current distance of the carriage from uppermost position.<br>It is possible to calculate other distances:<br>Depth under surface = CarriageDepth + carriageTopToSpiralTipMM - linAxisZeroToSensorMM - heightAboveGround (the sensor value).<br>Depth under rover = CarriageDepth + carriageTopToSpiralTipMM - linAxisZeroToSensorMM<br>Where carriageTopToSpiralTipMM = 720<br>linAxisZeroToSensorMM = 775   | int16 | mm |
| Vertical drive speed | int8 | 0,1 mm/s (10 = 1 mm/s) |
| Vertical drive stepper current | uint8 | 0,01 A (100 = 1 A) |
| Spiral motor speed | int16 | RPM |
| Spiral motor winding current | int16 | 0,01 A (100 = 1 A) |
| Spiral motor current draw | int16 | 0,01 A (100 = 1 A) |
| Spiral motor torque | int16 | 0,01 Nm (100 = 1 Nm) |
| Spiral motor temperature | uint8 | °C |
| Deep sample storage angle | uint16 | ° |
| DeepSampler state | uint8 | Code of the state of the autonomous state machine of DeepSampler - meaning in table below. |
| DrillController state | uint8 | Meaning in table below. |
| DeepSampleHolder state | uint8 | Meaning in table below. |


DeepSampler state codes 
| Code | Meaning |
|-     |-        |
| 0x00 | MANUAL |
| 0x01 | WAITING_FOR_STORAGE_CLEAR |
| 0x02 | DRILLING |
| 0x03 | MOVE_CARRIAGE_TO_STORE |
| 0x04 | MOVING_CARRIAGE_TO_STORE |
| 0x05 | MOVING_STORAGE |
| 0x06 | STORING |
| 0x07 | WEIGHING |
| 0x08 | MOVING_UP |
| 0xFE | DONE |
| 0xFF | ERROR |

## DrillController state codes
| Code | Meaning |
|-     |-        |
| 0x00 | MANUAL |
| 0x01 | WAITING_FOR_HEIGHT |
| 0x02 | MOVING_DOWN |
| 0x03 | DRILLING |
| 0xFE | DONE |
| 0xFF | ERROR |

## DeepSampleHolder state codes
| Code | Meaning |
|-     |-        |
| 0x00 | MANUAL |
| 0x01 | STORAGE_MOVING |
| 0x02 | WAITING_FOR_SETTLE |
| 0x03 | WEIGHING |
| 0xFE | DONE |
| 0xFF | ERROR |

## CHECK DEVICES response order
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
