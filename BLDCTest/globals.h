//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

#ifndef GLOBALS_H
#define GLOBALS_H

// Arduino includes
#include <inttypes.h>

struct MotorEncoderInfo {
  // All volatile members are accessed by interrupts and regular code
  int32_t volatile totalInterrupts; // Number of times interrupts have been called
  int32_t volatile tickCount;       // Number of valid ticks counted
  int32_t volatile uTickCount;      // Number of U ticks
  int32_t volatile vTickCount;      // Number of V ticks
  int32_t volatile wTickCount;      // Number of W ticks
  int volatile uFaultCount;         // Number of faults recorded in U interrupt
  int volatile vFaultCount;         // Number of faults recorded in V interrupt
  int volatile wFaultCount;         // Number of faults recorded in W interrupt
  char volatile prevUVal;           // Previous value read on U interrupt pin (0/1)
  char volatile prevVVal;           // Previous value read on V interrupt pin (0/1)
  char volatile prevWVal;           // Previous value read on W interrupt pin (0/1)
  char volatile prevEncoder;        // Previous interrupt that was executed
  bool volatile motorDirection;     // Current motor direction
  bool incrementDirection;          // Speed increment direction (speeding up/slowing down)
  int speed;                        // Current speed (power)
};

extern MotorEncoderInfo motor1;
extern MotorEncoderInfo motor2;

#endif // GLOBALS_H
