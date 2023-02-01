
#ifndef GLOBALS_H
#define GLOBALS_H

// Arduino includes
#include <inttypes.h>

struct MotorEncoderInfo {
  // All volatile members are accessed by interrupts and regular code
  volatile int32_t totalInterrupts; // Number of times interrupts have been called
  volatile int32_t tickCount;       // Number of valid ticks counted
  volatile int32_t uTickCount;      // Number of U ticks
  volatile int32_t vTickCount;      // Number of V ticks
  volatile int32_t wTickCount;      // Number of W ticks
  volatile int uFaultCount;         // Number of faults recorded in U interrupt
  volatile int vFaultCount;         // Number of faults recorded in V interrupt
  volatile int wFaultCount;         // Number of faults recorded in W interrupt
  volatile char prevUVal;           // Previous value read on U interrupt pin (0/1)
  volatile char prevVVal;           // Previous value read on V interrupt pin (0/1)
  volatile char prevWVal;           // Previous value read on W interrupt pin (0/1)
  volatile char prevEncoder;        // Previous interrupt that was executed
  volatile bool motorDirection;     // Current motor direction
  bool incrementDirection;          // Speed increment direction (speeding up/slowing down)
  int speed;                        // Current speed (power)
};

extern MotorEncoderInfo motor1;
extern MotorEncoderInfo motor2;

#endif // GLOBALS_H
