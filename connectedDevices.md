List of devices connected to the drill's microcontroller:

|Device                        |Means of communication                    |Bus address / GPIO pin|
|---------                     |----------------------                    |---|
|Rover's master computer       |UART via USB port on devkit               |pins 1, 3|
|DC motor AK45-10              |UART                                      |pins 16, 17|
|Vertical drive stepper driver |?                                         ||
|Vertical drive stepper encoder|I2C                                       |Programmable I2C address|
|Vertical drive limit sensor 2x|+V/GND to GPIO pin                        ||
|Storage stepper driver        |?                                         ||
|Storage stepper encoder       |I2C                                       |Programmable I2C address|
|Storage ADC                   |I2C                                       |3 possible I2C addresses|
|Height sensor                 |I2C (possible analog and PWM)             |Programmable I2C address|
