# Controller Interface/API
This document outlines the API of the controller interface that is used to control the motors.
For purposes of design we are assuming I2C is used and that there is a set of registers that
are read-only or read/write as appropriate. Setting a register affects the motorcontroller as
it uses that value to control the motors.

## Read/Write registers

  - Target speed left motor - The target speed for the left motor the PID should aim for. (units for speed, tbd)
  - P value, left motor - The P value to use for PID controller of left motor.
  - I value, left motor - The I value to use for PID controller of left motor.
  - D value, left motor - The D value to use for PID controller of left motor.
  - Target speed right motor - The target speed for the right motor the PID should aim for. (units for speed, tbd)
  - P value, right motor - The P value to use for PID controller of right motor.
  - I value, right motor - The I value to use for PID controller of right motor.
  - D value, right motor - The D value to use for PID controller of right motor.
  - Command timeout - The amount of time before the motorcontroller will stop all motors if no command is received.

## Read-Only registers

  - Current speed left motor - The current speed of the left motor.
  - Current speed right motor - The current speed of the right motor.
