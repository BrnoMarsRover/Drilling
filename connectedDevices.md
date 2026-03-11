List of devices connected to the drill's microcontroller:

|Device                                |Means of communication                    |Bus address / GPIO pin      |Communication voltage      |Power voltage    |
|---------                             |----------------------                    |----------------------------|---------------------------|-----------------|
|Rover's master computer               |UART via USB port on devkit               |pins 1, 3                   |5 V, through USB converter |x     |
|DC motor AK45-10                      |UART                                      |pins 16, 17                 |3,3 V                      |24 V  |
|Vertical drive stepper driver (5160)  |SPI                                       |                            |3,3 V                      |24 V  |
|Vertical drive stepper encoder        |I2C                                       |Programmable I2C address    |3,3 V                      |3,3 V |
|Vertical drive limit sensor 2x        |+V/GND to GPIO pin                        |                            |3,3 V                      |3,3 V |
|Storage stepper driver (2209)         |SPI                                       |                            |3,3 V                      |24 V  |
|Storage stepper encoder               |I2C                                       |Programmable I2C address    |3,3 V                      |3,3 V |
|Storage ADC                           |I2C                                       |3 possible I2C addresses    |5 V                        |5 V   |
|Height sensor                         |I2C (possible analog and PWM)             |Programmable I2C address    |3,3 V                      |3,3 V |
