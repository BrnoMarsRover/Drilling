List of devices connected to the drill's microcontroller:

|Device                        |Means of communication                    |
|---------                     |----------------------                    |
|Rover's master computer       |UART via USB port on devkit|
|DC motor AK45-10              |UART|
|Vertical drive stepper driver |?|
|Vertical drive stepper encoder|I2C|
|Vertical drive limit sensor 2x|+V/GND to GPIO pin|
|Storage stepper driver        |?|
|Storage stepper encoder       |I2C|
|Storage ADC                   |I2C|
|Height sensor                 |I2C (possible analog and PWM|
