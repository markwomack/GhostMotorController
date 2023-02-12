# Ghost Motor Controller
<p>The Ghost Motor Controller is currently a proof-of-concept project that is creating motor controller hardware and software to manage two BLDC hoverboard/eScooter motors with 3-phase motor encoders. The goal is to use this motor controller as part of a mobile, differential drive robot, and to have a design that can eventually be reused with different underlying hardware and motors. The parameters of the project are to use as many 'off-the-shelf' parts as possible and as few custom designed parts. This is where the project name comes from: at the end of the day we are just putting a sheet over a collection of parts and calling it a motor controller. Custom PCBs are allowed since maintaining breadboards and/or protoboards in a robot can be cumbersome and require more room than desired or needed. However, much of the PCB design is (currently) integrating existing break out boards, mostly from Pololu and Sparkfun.</p>

## Status
<p>Ghost is currently in the 'proof-of-concept' phase. Most of the time thus far has been spent choosing the initial set of parts, getting a power system in place to support the parts, and designing the first version of circuitry that will make up the motor controller. The power system, motors, and a control circuit has been designed and tested. A custom PCB has been designed and will be integrated with the test system soon.</p>

## Hardware
Instead of creating a custom circuit to control the motors, an existing eScooter controller is being used. But since this controller doesn't have a programmatic interface, a microcontroller circuit is being designed. Basically a controller of a controller. This is the Ghost Motor Controller.

## Software
The Ghost Motor Controller will provide an API accessible through I2C. Using this API the user will be able to set/get configuration and set/get information and actions on the motors and associated encoders. Everything one needs to control the motors on a mobile, differential drive robot will be provided. It will be very similar to existing APIs for other popular controllers, like the RoboClaw.
