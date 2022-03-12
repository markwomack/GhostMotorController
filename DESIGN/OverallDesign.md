# Overall Design
<p>Simply stated, the goal of this project is to create a motor controller that can
control two scooter Brushless DC (BLDC) motors and that can be used in robotic projects in
replacement of more typical motors and motor controllers. Scooter BLDC motors, due
to their design, can handle large loads (on the order of a grown adult) and can
provide lots of torque. These features make them very ideal for many robotic
applications.</p>

<p>BLDC motors have been around forever, and are a staple of most 
robotic projects, especially mobile robots. BLDC motors used on scooters have been
on the scene more recently but have gained in popularity and availability. One can
find many different models on Amazon and scooter supply websites. But using them
in a robotics project poses some difficulty.</p>

<p>Scooter BLDC motors use a 3-phase design for the motor, not just the simple two
wire configuration used in most robotic motors. In addition, while most scooter
motors have encoders built in, they are three encoders to match the 3-phases.
Two sensor, quadrature encoder support doesn't work. Both of these major differences
mean that none of the normal robotic motor controllers can be used to control scooter
motors.</p>

<p>While the usual stable of robotic motor controllers can't be used for scooter
BLDC motors, there are motor controllers that exist to control them. Scooters
obviously need some component to manage the motor speed, braking, and stopping
as the user drives the scooter around. These controllers are fairly simple in
design, providing for the 3-phase power signals and using the encoders to determine
wheel position and how to control the motor. As part of this project, one could design
and build such a controller from scratch, but instead we have opted to incorporate the
existing simple controller in the design. Doing so allows the builder to swap out the
simple controller to better match the power and motor requirements of their project.
As long as the replacement controller uses the same required interface for the power
and encoders, it should just be a matter of a drop-in replacement.</p>

<p>Similarly, the motors themselves can be swapped out as needed for the project
requirements. As long as the motor is a 3-phase motor with connections for power and
encoders, it will be compatible.</p>

<p>The real innovation will be the circuitry that encapsulates the simple controllers
and motors. It will provide an interface to control the motors based on a popular 
motor controller protocol, the RoboClaw. It will be a motor controller of motor
controllers, so to speak. Commands to control the motors will be sent to this top
level controller (via I2C or Serial, tbd), and the top level controller will in turn
send the proper commands to the simple controllers. In addition, PID control will be
implemented, using the 3-phase encoder signals from the motors. But putting the simple
controllers and motors behind a higher-level motor controller interface, scooter BLDC
motors will be easier to incorporate into a robotics project.</p>
