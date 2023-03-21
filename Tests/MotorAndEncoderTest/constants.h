//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

#ifndef CONSTANTS_H
#define CONSTANTS_H

// The rate at which the motor speed will be increased
const int speedIncrement(100);

// The maximum speed allowed for a motor
// This value is based on the settings of the PWM pin set in MotorAndEncoderTest.ino.
const int maxSpeed(8191); //8191 this value is based on the frequency set on the PWM

// Replace this value once you have determined the ticks per rotation for your motor
// 120 is typical for built-in encoders for BLDC motors
const int32_t NUM_TICKS_PER_ROTATION(120);

#endif // CONSTANTS_H
