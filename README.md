# Diffy swerve for MiniFRC
Source code for a diffy swerve robot for MiniFRC based off WpiLIB's swerve libraries converted to c++.

[original repo](https://github.com/ethanprivette/DIFFYV2)
## Prerequisites
- Alfredo-Connect library
  - Follow all instructions [here](https://github.com/AlfredoSystems/Alfredo-NoU2) to set up library
    - This is how the robot will be controlled over bluetooth
- Adafruit_MotorShield library
  - Download from the library manager tab in arduinoIDE
    - This is the motor controller for arduino UNO
- Adafruit_BNO08x library
  - Download from the library manager tab in arduinoIDE
    - BNO08x gyro for field oriented driving
## Setup
All prerequisites should be downloaded, if not go [here](https://github.com/ethanprivette/DIFFYV2/new/master?readme=1#prerequisites)
- Download the .ino file from the repo
  - Make modifications as needed
## Use
This code was written to run on an Arduino UNO with two Adafruit motor shields stacked. 

N20 encoders will most likely be connected to other boards and communicated via I2C to the UNO ***subject to change***

Connect via I2C to a NoU board for control
## Things to look out for
- Calculations to move modules to correct angle
- Comments so ppl can understand this mess
- Bug fixes after I actually get to test
