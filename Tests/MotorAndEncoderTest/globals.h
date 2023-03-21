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
  int32_t volatile totalInterrupts;   // Number of times interrupts have been called
  int32_t volatile tickCount;         // Number of valid ticks counted in total
  int32_t volatile uTickCount;        // Number of valid U ticks
  int32_t volatile vTickCount;        // Number of valid V ticks
  int32_t volatile wTickCount;        // Number of valid W ticks
  int volatile uFaultCount;           // Number of U faults
  int volatile vFaultCount;           // Number of V faults
  int volatile wFaultCount;           // Number of W faults
  char volatile prevUVal;             // Previous value read on U interrupt pin (0/1)
  char volatile prevVVal;             // Previous value read on V interrupt pin (0/1)
  char volatile prevWVal;             // Previous value read on W interrupt pin (0/1)
  char volatile prevInterrupt;        // Previous interrupt that was executed
  bool volatile actualMotorDirection; // Actual motor direction as determined by encoder signals
  bool volatile motorDirection;       // Current motor direction being applied to speed
  bool incrementDirection;            // Speed increment direction (speeding up/slowing down)
  int speed;                          // Current speed (power)
};

extern MotorEncoderInfo motorEncoderM0;
extern MotorEncoderInfo motorEncoderM1;

#endif // GLOBALS_H
