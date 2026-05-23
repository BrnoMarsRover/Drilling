# Drilling
Repo for Deep Sampling Sub-Task
Release 2025 was mostly a success and allowed us to identify its shortcomings.
Currently working on an overhauled design.

## Authors

- **Vilem Strachon** *(Leader)*  
  Responsible for the weighing and storing.

- **Martin Kriz**  
  Senior consultant.

- **Filip Slima**  
  Responsible for mechanical construction, linear motion system.

- **Ondrej Stafa**  
  Responsible for DC motor control.

## ESP software structure
```mermaid
graph TD;
    Main-->RoverComm;
    Main-->DeepSampler;
      DeepSampler-->DrillController;
        DrillController-->LinearAxis;
          LinearAxis-->Stepper1;
          LinearAxis-->Encoder1;
          LinearAxis-->LimitSwitch;
          LinearAxis-->CurrentSensor;
        DrillController-->SpiralMotor;
      	DrillController-->HeightSensor;
      DeepSampler-->DeepSampleHolder;
        DeepSampleHolder-->ADC1
        DeepSampleHolder-->StoragePositioner
          StoragePositioner-->Stepper2;
          StoragePositioner-->Encoder2;
  Main-->SurfaceSampleHolder;
    SurfaceSampleHolder-->ADC2
    SurfaceSampleHolder-->Servo;
```

## Communication protocol
[Here](communicationProtocol.md)
