# Proof of Concept Hardware (work in progress)
<p>For the proof of concept the parts below were used. If building your own version you can use
  these same parts or substitute different parts for anything in the 'Motor and Power' category
  as long as it conforms to the following requirements:</p>
  
  - The motors must be 3 phase and provide encoder signals that can be connected to the 3.3v
    Teensy 4.0 (using a 5v->3.3v logic converter, if needed, is acceptable).
  - The BLDC motor controllers must be compatible with the chosen motors and must allow for the
    speed to be set via a PWM signal.
  - Proper voltage must be provided to the motors, the motorcontrollers, and the Teensy 4.0.
  - The [36v->5v step-down regulator](https://www.amazon.com/dp/B07YCQTSXQ) is used for intial
    voltage conversion, and a second, smaller [5v step-down regulator](https://www.pololu.com/product/4941)
    is used for added protection.

<p>Substituting a different microcontroller will most likely require major reworking of the code,
  and so it is recommended that the Teensy 4.0 be used and not substituted. YMMV.</p>

## Motor and Power
  - 2 - [12" scooter/ebike BLDC motors](https://www.amazon.com/dp/B08ZXYND7G)
  - 2 - [BLDC motor controllers](https://www.amazon.com/RioRand-6-60V-Brushless-Electric-Controller/dp/B087M2378D)
  - 1 - [36v ebike lithium ion battery](https://www.amazon.com/dp/B08FWRZYJ3)
  - 1 - [Battery Disconnect Switch](https://www.amazon.com/dp/B08YXGXW2W)
  - 1 - [Inline Fuse Holder](https://www.amazon.com/dp/B081DHT8Y7)
  - 2 - [24v DC regulator](https://www.amazon.com/gp/product/B098D8HMJ9)
  - 1 - [5v DC step-down regulator](https://www.amazon.com/dp/B07YCQTSXQ)
  - 2 - [Shunt Regulator](https://www.pololu.com/product/3779)
  - 1 - [Mini Pushbutton Power Switch with Reverse Voltage Protection](https://www.pololu.com/product/2808)
  - 2 - [Big Pushbutton Power Switch with Reverse Voltage Protection](https://www.pololu.com/product/2813)
  - 1 - [3.3v step-down regulator](https://www.pololu.com/product/4940)
  - 1 - [5v step-down/up regulator](https://www.pololu.com/product/4941)
  - 1 - Diode (to prevent backflow to the power switches)
  - 1 - Momentary pushbutton

## Microcontroller
  - 1 - [Teensy 4.0](https://www.pjrc.com/store/teensy40.html)
  - 5 - [Logic Converter](https://www.sparkfun.com/products/12009)
  - 2 - [Hex Inverters with Schmitt Triggers IC chips)](https://www.digikey.com/en/products/detail/texas-instruments/SN74HC14N/277223)
  - 6 - [1000pF capacitors](https://www.digikey.com/en/products/detail/vishay-beyschlag-draloric-bc-components/A102K15X7RF5TAA/2356716)
  - 1 - Momentary pushbutton

## Miscellaneous
  - [Wire connectors](https://www.amazon.com/dp/B0882TNS73)
  - [12 AWG wire](https://www.amazon.com/dp/B01ABOPMEI)
