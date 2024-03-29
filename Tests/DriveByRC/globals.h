//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>

// Constants used with PID controller in the motor controller
const double KP(0.25);
const double KI(0.0008);
const double KD(0.0005);

const int NO_RC_SIGNAL(-10000);

//*** These are the contants based on the specific motors and encoders being used.
//*** You should use values that match your own robot.

// Total encoder ticks for single wheel revolution
const double TICKS_PER_ROTATION(120.0); // 120 ticks for BLDC motor

// This constant is based on measurements of the motor performance using
// full motor power your robot is using. Your mileage may vary and you should
// verify your robots values by running the motors at full power (1.0) for a
// second and recording the encoder values. Or running the motors at full
// power for an extended time and taking the average ticks per second.
// When motor is run at full for one second, this is approximately how
// many ticks will be counted.
const double MAX_TICKS_PER_SECOND(925.0);

// Wheel diameter in meters (12 inches)
const double WHEEL_DIAM_M(0.3048);

// *** Everything below here is calculated using the three values above

// Code will use Radians (angular velocity) for velocity measurements
// Note: There are 2 pi radians in a single rotation (360 degrees)
const double RADIANS_PER_TICK(2 * PI / TICKS_PER_ROTATION);

// This is the number of wheel rotations when run at full for one second.
const double MAX_ROTATIONS_PER_SECOND(MAX_TICKS_PER_SECOND / TICKS_PER_ROTATION);

// This is the max radians per second when run at full speed.
// Note: there are 2 pi radians in one rotation.
const double MAX_RADIANS_PER_SECOND(MAX_ROTATIONS_PER_SECOND * 2 * PI);

// Just for reference, this the max meters per second when run at full speed.
// It is just for reference, since the MotorController will use radians/second.
const double WHEEL_CIRCUMFERENCE_M(PI * WHEEL_DIAM_M); // C=2*pi*r -> C=pi*2r -> C=pi*D
const double MAX_METERS_PER_SECOND(MAX_ROTATIONS_PER_SECOND * WHEEL_CIRCUMFERENCE_M);

// Maximum velocity allowed for linear velocity (forward/reverse)
const double MAX_LINEAR_VELOCITY_ALLOWED(0.25 * MAX_RADIANS_PER_SECOND);  // 25% of maximum speed

// Maximum velocity allowed for angular velocity (left/right)
const double MAX_ANGULAR_VELOCITY_ALLOWED(0.07 * MAX_RADIANS_PER_SECOND); // 15% of maximum speed

const double MAX_SPEED_INCREMENT_ALLOWED(0.15 * MAX_RADIANS_PER_SECOND); // 15% of maximum speed
#endif // GLOBALS_H
