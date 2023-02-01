
// Arduino includes
#include <Arduino.h>

// Local includes
#include "interrupts.h"
#include "globals.h"
#include "pin_assignments.h"

/**
  Interrupt handler for the M1 U encoder signal.
**/
void countM1UTick() {
  // Read the current signal value
  int val = digitalRead(M1_U_ENCODER_SIGNAL_PIN);
  
  motor1.totalInterrupts++;

  // Check for an encoder fault  
  bool encoderFault = (motor1.prevEncoder == 'U') || // Last encoder was this one
                      (motor1.prevEncoder == 'W' && motor1.motorDirection) || // Last encoder was W, but going forward
                      (motor1.prevEncoder == 'V' && !motor1.motorDirection);  // Last encoder was V, but going reverse
  
  if (encoderFault) {
    motor1.uFaultCount++;
  }

  // DebugMsgs.notification().print("U ").print(motor1.prevUVal).print(' ').print(val).print(' ')
  //   .print(motor1.lastEncoder).println(encoderFault ? " *" : "");
      
  // If there is a fault, don't record a tick.
  if (encoderFault) { return; }
  
  int increment = (motor1.prevEncoder == 'W') ? 1 : -1;
  motor1.uTickCount += increment;
  motor1.tickCount += increment;
  motor1.prevEncoder = 'U';
  motor1.prevUVal = val;
}

/**
  Interrupt handler for the M1 V encoder signal.
**/
void countM1VTick() {
  // Read the current signal value
  int val = digitalRead(M1_V_ENCODER_SIGNAL_PIN);
  
  motor1.totalInterrupts++;

  // Check for an encoder fault  
  bool encoderFault = (motor1.prevEncoder == 'V') || // Last encoder was this one
                      (motor1.prevEncoder == 'U' && motor1.motorDirection) || // Last encoder was U, but going forward
                      (motor1.prevEncoder == 'W' && !motor1.motorDirection);  // Last encoder was W, but going reverse
  
  if (encoderFault) {
    motor1.vFaultCount++;
  }

  // DebugMsgs.notification().print("V ").print(motor1.prevVVal).print(' ').print(val).print(' ')
  //   .print(motor1.prevEncoder).println(encoderFault ? " *" : "");

  // If there is a fault, don't record a tick.
  if (encoderFault) { return; }
  
  int increment = (motor1.prevEncoder == 'U') ? 1 : -1;
  motor1.vTickCount += increment;
  motor1.tickCount += increment;
  motor1.prevEncoder = 'V';
  motor1.prevVVal = val;
}

/**
  Interrupt handler for the M1 V encoder signal.
**/

void countM1WTick() {
  // Read the current signal value
  int val = digitalRead(M1_W_ENCODER_SIGNAL_PIN);

  motor1.totalInterrupts++;
  
  // Check for an encoder fault  
  bool encoderFault = (motor1.prevEncoder == 'W') || // Last encoder was this one
                      (motor1.prevEncoder == 'V' && motor1.motorDirection) || // Last encoder was V, but going forward
                      (motor1.prevEncoder == 'U' && !motor1.motorDirection);  // Last encoder was U, but going reverse

  if (encoderFault) {
    motor1.wFaultCount++;
  }
  
  // DebugMsgs.notification().print("W ").print(motor1.prevWVal).print(' ').print(val).print(' ')
  //   .print(motor1.prevEncoder).println(encoderFault ? " *" : "");

  // If there is a fault, don't record a tick.
  if (encoderFault) { return; }
  
  int increment = (motor1.prevEncoder == 'V') ? 1 : -1;         
  motor1.wTickCount += increment;
  motor1.tickCount += increment;
  motor1.prevEncoder = 'W';
  motor1.prevWVal = val;
}

/**
  Interrupt handler for the M2 U encoder signal.
**/
void countM2UTick() {
  // Read the current signal value
  int val = digitalRead(M2_U_ENCODER_SIGNAL_PIN);
  
  motor2.totalInterrupts++;

  // Check for an encoder fault  
  bool encoderFault = (motor2.prevEncoder == 'U') || // Last encoder was this one
                      (motor2.prevEncoder == 'W' && motor2.motorDirection) || // Last encoder was W, but going forward
                      (motor2.prevEncoder == 'V' && !motor2.motorDirection);  // Last encoder was V, but going reverse
  
  if (encoderFault) {
    motor2.uFaultCount++;
  }

  // DebugMsgs.notification().print("U ").print(motor2.prevUVal).print(' ').print(val).print(' ')
  //   .print(motor2.lastEncoder).println(encoderFault ? " *" : "");
      
  // If there is a fault, don't record a tick.
  if (encoderFault) { return; }
  
  int increment = (motor2.prevEncoder == 'W') ? 1 : -1;
  motor2.uTickCount += increment;
  motor2.tickCount += increment;
  motor2.prevEncoder = 'U';
  motor2.prevUVal = val;
}

/**
  Interrupt handler for the M2 V encoder signal.
**/
void countM2VTick() {
  // Read the current signal value
  int val = digitalRead(M2_V_ENCODER_SIGNAL_PIN);
  
  motor2.totalInterrupts++;

  // Check for an encoder fault  
  bool encoderFault = (motor2.prevEncoder == 'V') || // Last encoder was this one
                      (motor2.prevEncoder == 'U' && motor2.motorDirection) || // Last encoder was U, but going forward
                      (motor2.prevEncoder == 'W' && !motor2.motorDirection);  // Last encoder was W, but going reverse
  
  if (encoderFault) {
    motor2.vFaultCount++;
  }

  // DebugMsgs.notification().print("V ").print(motor2.prevVVal).print(' ').print(val).print(' ')
  //   .print(motor2.prevEncoder).println(encoderFault ? " *" : "");

  // If there is a fault, don't record a tick.
  if (encoderFault) { return; }
  
  int increment = (motor2.prevEncoder == 'U') ? 1 : -1;
  motor2.vTickCount += increment;
  motor2.tickCount += increment;
  motor2.prevEncoder = 'V';
  motor2.prevVVal = val;
}

/**
  Interrupt handler for the M2 V encoder signal.
**/

void countM2WTick() {
  // Read the current signal value
  int val = digitalRead(M2_W_ENCODER_SIGNAL_PIN);

  motor2.totalInterrupts++;
  
  // Check for an encoder fault  
  bool encoderFault = (motor2.prevEncoder == 'W') || // Last encoder was this one
                      (motor2.prevEncoder == 'V' && motor2.motorDirection) || // Last encoder was V, but going forward
                      (motor2.prevEncoder == 'U' && !motor2.motorDirection);  // Last encoder was U, but going reverse

  if (encoderFault) {
    motor2.wFaultCount++;
  }
  
  // DebugMsgs.notification().print("W ").print(motor2.prevWVal).print(' ').print(val).print(' ')
  //   .print(motor2.prevEncoder).println(encoderFault ? " *" : "");

  // If there is a fault, don't record a tick.
  if (encoderFault) { return; }
  
  int increment = (motor2.prevEncoder == 'V') ? 1 : -1;         
  motor2.wTickCount += increment;
  motor2.tickCount += increment;
  motor2.prevEncoder = 'W';
  motor2.prevWVal = val;
}
