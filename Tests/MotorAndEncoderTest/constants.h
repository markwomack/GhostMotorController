//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

#ifndef CONSTANTS_H
#define CONSTANTS_H

// Arduino includes
#include <inttypes.h>

// *** Modify these to limit max speed or the rate of acceleration

// Number of units to increase/decrease when adjusting motor speed
const int SPEED_INCREMENT(100);

// Maximum speed allowed. This value is based on the frequency set
// on the speed PWM pins (see pinSetup in MotorAndEncoderTest)
const int MAX_SPEED(8191);

// *** Set this to the number of ticks/rotation once you have determined it
const int32_t NUM_TICKS_PER_ROTATION(120);

#endif // CONSTANTS_H
