# Proof of Concept Hardware (work in progress)
<p>For the proof of concept the parts below were used. If building your own version you can use
  these same parts or substitute different parts for anything in the 'Motor and Power' category
  as long as it conforms to the following requirements:</p>
  
  - The motors must be 3 phase and provide encoder signals that can be connected to the 3.3v
    Teensy 4.0.
  - The motorcontrollers must be compatible with the chosen motors and must allow for the
    speed to be set via a PWM signal.
  - Proper voltage must be provided to the motors, the motorcontrollers, and the Teensy 4.0.

<p>Substituting a different microcontroller will most likely require major reworking of the code,
  and so it is recommended that the Teensy 4.0 be used and not substituted.</p>

## Motor and Power
  - 2 - [12" scooter/ebike BLDC motors](https://www.amazon.com/dp/B08ZXYND7G)
  - 2 - [BLDC motorcontrollers](https://www.amazon.com/RioRand-6-60V-Brushless-Electric-Controller/dp/B087M2378D)
  - 1 - [36v ebike lithium ion battery](https://www.amazon.com/dp/B08FWRZYJ3)
  - 1 - [XT60 inline on/off switch](https://www.amazon.com/dp/B0993C65H5)
  - 2 - [24v DC regulator](https://www.amazon.com/dp/B06Y5JVHX8)
  - 2 - [5v DC step-down regulator](https://www.amazon.com/dp/B00J3MHRNO)
  - 2 - [Shunt Regulator](https://www.pololu.com/product/3779)
  - 2 - [Big Pushbutton Power Switch with Reverse Voltage Protection](https://www.pololu.com/product/2813)
  - 1 - [3.3v step-down regulator](https://www.pololu.com/product/2122)

## Microcontroller
  - 1 - [Teensy 4.0](https://www.pjrc.com/store/teensy40.html)
  - voltage level shifter
  - 1 Momentary pushbutton
