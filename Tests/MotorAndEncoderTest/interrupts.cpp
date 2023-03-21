//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

// Arduino includes
#include <Arduino.h>

// Local includes
#include "interrupts.h"
#include "globals.h"
#include "pin_assignments.h"

/**
  Interrupt handler for the M0 U encoder signal.
**/
void countM0UTick() {
  // Read the current signal value
  int val = digitalRead(M0_U_SIGNAL_PIN);
  
  motorM0.totalInterrupts++;

  // Check for an encoder fault  
  bool encoderFault = (motorM0.prevInterrupt == 'U') || // Last encoder was this one
                      (motorM0.prevInterrupt == 'W' && motorM0.motorDirection) || // Last encoder was W, but going forward
                      (motorM0.prevInterrupt == 'V' && !motorM0.motorDirection);  // Last encoder was V, but going reverse
  
  if (encoderFault) {
    motorM0.uFaultCount++;
  }

  // DebugMsgs.notification().print("U ").print(motorM0.prevUVal).print(' ').print(val).print(' ')
  //   .print(motorM0.lastEncoder).println(encoderFault ? " *" : "");
      
  // If there is a fault, don't record a tick.
  if (encoderFault) { return; }
  
  int increment = (motorM0.prevInterrupt == 'W') ? 1 : -1;
  motorM0.actualMotorDirection = increment == 1 ? true : false;
  motorM0.uTickCount += increment;
  motorM0.tickCount += increment;
  motorM0.prevInterrupt = 'U';
  motorM0.prevUVal = val;
}

/**
  Interrupt handler for the M0 V encoder signal.
**/
void countM0VTick() {
  // Read the current signal value
  int val = digitalRead(M0_V_SIGNAL_PIN);
  
  motorM0.totalInterrupts++;

  // Check for an encoder fault  
  bool encoderFault = (motorM0.prevInterrupt == 'V') || // Last encoder was this one
                      (motorM0.prevInterrupt == 'U' && motorM0.motorDirection) || // Last encoder was U, but going forward
                      (motorM0.prevInterrupt == 'W' && !motorM0.motorDirection);  // Last encoder was W, but going reverse
  
  if (encoderFault) {
    motorM0.vFaultCount++;
  }

  // DebugMsgs.notification().print("V ").print(motorM0.prevVVal).print(' ').print(val).print(' ')
  //   .print(motorM0.prevInterrupt).println(encoderFault ? " *" : "");

  // If there is a fault, don't record a tick.
  if (encoderFault) { return; }
  
  int increment = (motorM0.prevInterrupt == 'U') ? 1 : -1;
  motorM0.actualMotorDirection = increment == 1 ? true : false;
  motorM0.vTickCount += increment;
  motorM0.tickCount += increment;
  motorM0.prevInterrupt = 'V';
  motorM0.prevVVal = val;
}

/**
  Interrupt handler for the M0 V encoder signal.
**/

void countM0WTick() {
  // Read the current signal value
  int val = digitalRead(M0_W_SIGNAL_PIN);

  motorM0.totalInterrupts++;
  
  // Check for an encoder fault  
  bool encoderFault = (motorM0.prevInterrupt == 'W') || // Last encoder was this one
                      (motorM0.prevInterrupt == 'V' && motorM0.motorDirection) || // Last encoder was V, but going forward
                      (motorM0.prevInterrupt == 'U' && !motorM0.motorDirection);  // Last encoder was U, but going reverse

  if (encoderFault) {
    motorM0.wFaultCount++;
  }
  
  // DebugMsgs.notification().print("W ").print(motorM0.prevWVal).print(' ').print(val).print(' ')
  //   .print(motorM0.prevInterrupt).println(encoderFault ? " *" : "");

  // If there is a fault, don't record a tick.
  if (encoderFault) { return; }
  
  int increment = (motorM0.prevInterrupt == 'V') ? 1 : -1;  
  motorM0.actualMotorDirection = increment == 1 ? true : false;       
  motorM0.wTickCount += increment;
  motorM0.tickCount += increment;
  motorM0.prevInterrupt = 'W';
  motorM0.prevWVal = val;
}

/**
  Interrupt handler for the M1 U encoder signal.
**/
void countM1UTick() {
  // Read the current signal value
  int val = digitalRead(M1_U_SIGNAL_PIN);
  
  motorM1.totalInterrupts++;

  // Check for an encoder fault  
  bool encoderFault = (motorM1.prevInterrupt == 'U') || // Last encoder was this one
                      (motorM1.prevInterrupt == 'W' && motorM1.motorDirection) || // Last encoder was W, but going forward
                      (motorM1.prevInterrupt == 'V' && !motorM1.motorDirection);  // Last encoder was V, but going reverse
  
  if (encoderFault) {
    motorM1.uFaultCount++;
  }

  // DebugMsgs.notification().print("U ").print(motorM1.prevUVal).print(' ').print(val).print(' ')
  //   .print(motorM1.lastEncoder).println(encoderFault ? " *" : "");
      
  // If there is a fault, don't record a tick.
  if (encoderFault) { return; }
  
  int increment = (motorM1.prevInterrupt == 'W') ? 1 : -1;
  motorM1.actualMotorDirection = increment == 1 ? true : false;
  motorM1.uTickCount += increment;
  motorM1.tickCount += increment;
  motorM1.prevInterrupt = 'U';
  motorM1.prevUVal = val;
}

/**
  Interrupt handler for the M1 V encoder signal.
**/
void countM1VTick() {
  // Read the current signal value
  int val = digitalRead(M1_V_SIGNAL_PIN);
  
  motorM1.totalInterrupts++;

  // Check for an encoder fault  
  bool encoderFault = (motorM1.prevInterrupt == 'V') || // Last encoder was this one
                      (motorM1.prevInterrupt == 'U' && motorM1.motorDirection) || // Last encoder was U, but going forward
                      (motorM1.prevInterrupt == 'W' && !motorM1.motorDirection);  // Last encoder was W, but going reverse
  
  if (encoderFault) {
    motorM1.vFaultCount++;
  }

  // DebugMsgs.notification().print("V ").print(motorM1.prevVVal).print(' ').print(val).print(' ')
  //   .print(motorM1.prevInterrupt).println(encoderFault ? " *" : "");

  // If there is a fault, don't record a tick.
  if (encoderFault) { return; }
  
  int increment = (motorM1.prevInterrupt == 'U') ? 1 : -1;
  motorM1.actualMotorDirection = increment == 1 ? true : false;
  motorM1.vTickCount += increment;
  motorM1.tickCount += increment;
  motorM1.prevInterrupt = 'V';
  motorM1.prevVVal = val;
}

/**
  Interrupt handler for the M1 V encoder signal.
**/

void countM1WTick() {
  // Read the current signal value
  int val = digitalRead(M1_W_SIGNAL_PIN);

  motorM1.totalInterrupts++;
  
  // Check for an encoder fault  
  bool encoderFault = (motorM1.prevInterrupt == 'W') || // Last encoder was this one
                      (motorM1.prevInterrupt == 'V' && motorM1.motorDirection) || // Last encoder was V, but going forward
                      (motorM1.prevInterrupt == 'U' && !motorM1.motorDirection);  // Last encoder was U, but going reverse

  if (encoderFault) {
    motorM1.wFaultCount++;
  }
  
  // DebugMsgs.notification().print("W ").print(motorM1.prevWVal).print(' ').print(val).print(' ')
  //   .print(motorM1.prevInterrupt).println(encoderFault ? " *" : "");

  // If there is a fault, don't record a tick.
  if (encoderFault) { return; }
  
  int increment = (motorM1.prevInterrupt == 'V') ? 1 : -1;
  motorM1.actualMotorDirection = increment == 1 ? true : false;        
  motorM1.wTickCount += increment;
  motorM1.tickCount += increment;
  motorM1.prevInterrupt = 'W';
  motorM1.prevWVal = val;
}
