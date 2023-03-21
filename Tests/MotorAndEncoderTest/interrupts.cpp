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

// These are interrupt handlers attached to the pins
// connected to the U, V, and W signals from the motor
// encoder. Though the code is similar for each interrupt,
// each interrupt is specific to the signal and the motor,
// they are not shared.

/**
  Interrupt handler for the M0 U encoder signal.
**/
void countM0UTick() {
  // Read the current signal value
  int val = digitalRead(M0_U_ENCODER_SIGNAL_PIN);
  
  motorEncoderM0.totalInterrupts++;

  // Check for an encoder fault  
  bool encoderFault = (motorEncoderM0.prevInterrupt == 'U') || // Last encoder was this one
                      (motorEncoderM0.prevInterrupt == 'W' && motorEncoderM0.motorDirection) || // Last encoder was W, but going forward
                      (motorEncoderM0.prevInterrupt == 'V' && !motorEncoderM0.motorDirection);  // Last encoder was V, but going reverse
  
  if (encoderFault) {
    motorEncoderM0.uFaultCount++;
  }

  // Not recommended for a 'real' robot, but if you need to see what is going on, uncomment
  // DebugMsgs.debug().print("U ").print(motorEncoderM0.prevUVal).print(' ').print(val).print(' ')
  //   .print(motorEncoderM0.lastEncoder).println(encoderFault ? " *" : "");
      
  // If there is a fault, don't record a tick.
  if (encoderFault) { return; }
  
  int increment = (motorEncoderM0.prevInterrupt == 'W') ? 1 : -1;
  motorEncoderM0.uTickCount += increment;
  motorEncoderM0.tickCount += increment;
  motorEncoderM0.prevInterrupt = 'U';
  motorEncoderM0.prevUVal = val;
  motorEncoderM0.actualMotorDirection = increment == 1 ? true : false;
}

/**
  Interrupt handler for the M0 V encoder signal.
**/
void countM0VTick() {
  // Read the current signal value
  int val = digitalRead(M0_V_ENCODER_SIGNAL_PIN);
  
  motorEncoderM0.totalInterrupts++;

  // Check for an encoder fault  
  bool encoderFault = (motorEncoderM0.prevInterrupt == 'V') || // Last encoder was this one
                      (motorEncoderM0.prevInterrupt == 'U' && motorEncoderM0.motorDirection) || // Last encoder was U, but going forward
                      (motorEncoderM0.prevInterrupt == 'W' && !motorEncoderM0.motorDirection);  // Last encoder was W, but going reverse
  
  if (encoderFault) {
    motorEncoderM0.vFaultCount++;
  }

  // Not recommended for a 'real' robot, but if you need to see what is going on, uncomment
  // DebugMsgs.debug().print("V ").print(motorEncoderM0.prevVVal).print(' ').print(val).print(' ')
  //   .print(motorEncoderM0.prevInterrupt).println(encoderFault ? " *" : "");

  // If there is a fault, don't record a tick.
  if (encoderFault) { return; }
  
  int increment = (motorEncoderM0.prevInterrupt == 'U') ? 1 : -1;
  motorEncoderM0.vTickCount += increment;
  motorEncoderM0.tickCount += increment;
  motorEncoderM0.prevInterrupt = 'V';
  motorEncoderM0.prevVVal = val;
  motorEncoderM0.actualMotorDirection = increment == 1 ? true : false;
}

/**
  Interrupt handler for the M0 V encoder signal.
**/

void countM0WTick() {
  // Read the current signal value
  int val = digitalRead(M0_W_ENCODER_SIGNAL_PIN);

  motorEncoderM0.totalInterrupts++;
  
  // Check for an encoder fault  
  bool encoderFault = (motorEncoderM0.prevInterrupt == 'W') || // Last encoder was this one
                      (motorEncoderM0.prevInterrupt == 'V' && motorEncoderM0.motorDirection) || // Last encoder was V, but going forward
                      (motorEncoderM0.prevInterrupt == 'U' && !motorEncoderM0.motorDirection);  // Last encoder was U, but going reverse

  if (encoderFault) {
    motorEncoderM0.wFaultCount++;
  }
  
  // Not recommended for a 'real' robot, but if you need to see what is going on, uncomment
  // DebugMsgs.debug().print("W ").print(motorEncoderM0.prevWVal).print(' ').print(val).print(' ')
  //   .print(motorEncoderM0.prevInterrupt).println(encoderFault ? " *" : "");

  // If there is a fault, don't record a tick.
  if (encoderFault) { return; }
  
  int increment = (motorEncoderM0.prevInterrupt == 'V') ? 1 : -1;         
  motorEncoderM0.wTickCount += increment;
  motorEncoderM0.tickCount += increment;
  motorEncoderM0.prevInterrupt = 'W';
  motorEncoderM0.prevWVal = val;
  motorEncoderM0.actualMotorDirection = increment == 1 ? true : false;
}

/**
  Interrupt handler for the M2 U encoder signal.
**/
void countM1UTick() {
  // Read the current signal value
  int val = digitalRead(M1_U_ENCODER_SIGNAL_PIN);
  
  motorEncoderM1.totalInterrupts++;

  // Check for an encoder fault  
  bool encoderFault = (motorEncoderM1.prevInterrupt == 'U') || // Last encoder was this one
                      (motorEncoderM1.prevInterrupt == 'W' && motorEncoderM1.motorDirection) || // Last encoder was W, but going forward
                      (motorEncoderM1.prevInterrupt == 'V' && !motorEncoderM1.motorDirection);  // Last encoder was V, but going reverse
  
  if (encoderFault) {
    motorEncoderM1.uFaultCount++;
  }

  // Not recommended for a 'real' robot, but if you need to see what is going on, uncomment
  // DebugMsgs.debug().print("U ").print(motorEncoderM1.prevUVal).print(' ').print(val).print(' ')
  //   .print(motorEncoderM1.lastEncoder).println(encoderFault ? " *" : "");
      
  // If there is a fault, don't record a tick.
  if (encoderFault) { return; }
  
  int increment = (motorEncoderM1.prevInterrupt == 'W') ? 1 : -1;
  motorEncoderM1.uTickCount += increment;
  motorEncoderM1.tickCount += increment;
  motorEncoderM1.prevInterrupt = 'U';
  motorEncoderM1.prevUVal = val;
  motorEncoderM1.actualMotorDirection = increment == 1 ? true : false;
}

/**
  Interrupt handler for the M2 V encoder signal.
**/
void countM1VTick() {
  // Read the current signal value
  int val = digitalRead(M1_V_ENCODER_SIGNAL_PIN);
  
  motorEncoderM1.totalInterrupts++;

  // Check for an encoder fault  
  bool encoderFault = (motorEncoderM1.prevInterrupt == 'V') || // Last encoder was this one
                      (motorEncoderM1.prevInterrupt == 'U' && motorEncoderM1.motorDirection) || // Last encoder was U, but going forward
                      (motorEncoderM1.prevInterrupt == 'W' && !motorEncoderM1.motorDirection);  // Last encoder was W, but going reverse
  
  if (encoderFault) {
    motorEncoderM1.vFaultCount++;
  }

  // Not recommended for a 'real' robot, but if you need to see what is going on, uncomment
  // DebugMsgs.debug().print("V ").print(motorEncoderM1.prevVVal).print(' ').print(val).print(' ')
  //   .print(motorEncoderM1.prevInterrupt).println(encoderFault ? " *" : "");

  // If there is a fault, don't record a tick.
  if (encoderFault) { return; }
  
  int increment = (motorEncoderM1.prevInterrupt == 'U') ? 1 : -1;
  motorEncoderM1.vTickCount += increment;
  motorEncoderM1.tickCount += increment;
  motorEncoderM1.prevInterrupt = 'V';
  motorEncoderM1.prevVVal = val;
  motorEncoderM1.actualMotorDirection = increment == 1 ? true : false;
}

/**
  Interrupt handler for the M2 V encoder signal.
**/

void countM1WTick() {
  // Read the current signal value
  int val = digitalRead(M1_W_ENCODER_SIGNAL_PIN);

  motorEncoderM1.totalInterrupts++;
  
  // Check for an encoder fault  
  bool encoderFault = (motorEncoderM1.prevInterrupt == 'W') || // Last encoder was this one
                      (motorEncoderM1.prevInterrupt == 'V' && motorEncoderM1.motorDirection) || // Last encoder was V, but going forward
                      (motorEncoderM1.prevInterrupt == 'U' && !motorEncoderM1.motorDirection);  // Last encoder was U, but going reverse

  if (encoderFault) {
    motorEncoderM1.wFaultCount++;
  }
  
  // Not recommended for a 'real' robot, but if you need to see what is going on, uncomment
  // DebugMsgs.debug().print("W ").print(motorEncoderM1.prevWVal).print(' ').print(val).print(' ')
  //   .print(motorEncoderM1.prevInterrupt).println(encoderFault ? " *" : "");

  // If there is a fault, don't record a tick.
  if (encoderFault) { return; }
  
  int increment = (motorEncoderM1.prevInterrupt == 'V') ? 1 : -1;         
  motorEncoderM1.wTickCount += increment;
  motorEncoderM1.tickCount += increment;
  motorEncoderM1.prevInterrupt = 'W';
  motorEncoderM1.prevWVal = val;
  motorEncoderM1.actualMotorDirection = increment == 1 ? true : false;
}
