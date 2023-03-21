//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

// Arduino includes
#include <Arduino.h>

// My includes
#include <DebugMsgs.h>
#include <TaskManager.h>

// Local includes
#include "tasks.h"
#include "constants.h"
#include "pin_assignments.h"
#include "globals.h"

void stopMotorsGradually() {
  if (motorM0.speed > 0 || motorM1.speed > 0) {
    DebugMsgs.debug().println("Stopping motors...");
    // Spin down the motor in controlled manner
    while (motorM0.speed > 0 || motorM1.speed > 0) {
      motorM0.speed = max(motorM0.speed - 500, 0);
      analogWrite(M0_SPEED_PIN, motorM0.speed);
      motorM1.speed = max(motorM1.speed - 500, 0);
      analogWrite(M1_SPEED_PIN, motorM1.speed);
      delay(500);
    }
    // Don't allow the motor to spin
    digitalWrite(M0_BRAKE_PIN, HIGH);
    digitalWrite(M1_BRAKE_PIN, HIGH);
    DebugMsgs.debug().println("...motors stopped.");
  }
}

void stopMotorsImmediately() {
  analogWrite(M0_SPEED_PIN, 0);
  digitalWrite(M0_BRAKE_PIN, HIGH);
  analogWrite(M1_SPEED_PIN, 0);
  digitalWrite(M1_BRAKE_PIN, HIGH);
  motorM0.speed = 0; 
  motorM1.speed = 0; 
}

void ControlMotorTask::setup(MotorEncoderInfo* motor, String label, uint8_t pwmSpeedPin, uint8_t motorDirPin, uint8_t brakePin) {
  _motor = motor;
  _label = label;
  _pwmSpeedPin = pwmSpeedPin;
  _motorDirPin = motorDirPin;
  _brakePin = brakePin;
}
    
void ControlMotorTask::start(void) {
  // Clear the counts
  _motor->tickCount = _motor->totalInterrupts = 0;
  _motor->uTickCount = _motor->vTickCount = _motor->wTickCount = 0;
  _motor->uFaultCount = _motor->vFaultCount = _motor->wFaultCount = 0;
  
  _motor->incrementDirection = true; // false == slow it down, true == speed it up
  _motor->speed = 0;
  _motor->motorDirection = false;  // false == forward, true == reverse

  // Set up the speed (0) and set the direction
  analogWrite(_pwmSpeedPin, _motor->speed);
  digitalWrite(_motorDirPin, _motor->motorDirection ? HIGH : LOW);
  
  // Allow the motor to spin
  digitalWrite(_brakePin, LOW);
}

void ControlMotorTask::update(void) {
  // Motor is accelerating  
  if (_motor->incrementDirection) {
    _motor->speed = min(_motor->speed + speedIncrement, maxSpeed);

    // When we match the max speed, then start decelerating
    if (_motor->speed == maxSpeed) {
      _motor->incrementDirection = !_motor->incrementDirection;
    }
  // Motor is decelerating   
  } else {
    _motor->speed = max(_motor->speed - speedIncrement, 0);
    
    // When we reach 0, then switch the direction of the motor
    if (_motor->speed == 0) {
      _motor->incrementDirection = !_motor->incrementDirection;
      _motor->motorDirection = !_motor->motorDirection;
      digitalWrite(_motorDirPin, _motor->motorDirection ? HIGH : LOW);
    }
  }
  // Update the speed
  analogWrite(_pwmSpeedPin, _motor->speed);
}

void ControlMotorTask::stop(void) {
  stopMotorsGradually();
}

void CountRotationsTask::setRotations(int numRotations) {
  _numRotations = numRotations;
}

void CountRotationsTask::update(void) {
  noInterrupts();
  int32_t currentTicks = motorM0.tickCount;
  interrupts();
  if (currentTicks > (_numRotations * NUM_TICKS_PER_ROTATION)) {
    stopMotorsImmediately();
    taskManager.stop();
    DebugMsgs.debug().print("Stopped motors after ")
      .print(_numRotations).print(" rotations and ")
      .print(currentTicks).println(" ticks");
  }
}

void PrintMotorTickCountsTask::setup(MotorEncoderInfo* motor, String label) {
  _motor = motor;
  _label = label;      
}
    
void PrintMotorTickCountsTask::update(void) {
  DebugMsgs.debug()
    .print(_label)
    .print(" U: ").print(_motor->uTickCount)
    .print(" V: ").print(_motor->vTickCount)
    .print(" W: ").print(_motor->wTickCount)
    .print(" tick: ").print(_motor->tickCount)
    .print(" totalInterrupts: ").println(_motor->totalInterrupts);
  DebugMsgs.debug()
    .print(_label)
    .print(" Faults- U: ").print(_motor->uFaultCount)
    .print(" V: ").print(_motor->vFaultCount)
    .print(" W: ").println(_motor->wFaultCount);
}

void PrintTickSpeedTask::start(void) {
  lastM0TickCount = 0;
  lastM1TickCount = 0;
  lastTickCountTime = 0;      
};

void PrintTickSpeedTask::update(void) {
  int32_t curM0TickCount = motorM0.tickCount;
  int32_t curM1TickCount = motorM1.tickCount;
  int32_t curTickTime = millis();
  
  if (lastTickCountTime != 0) {
    int32_t diffTime = curTickTime - lastTickCountTime;
    int32_t diffM0Ticks = abs(curM0TickCount - lastM0TickCount);
    int32_t diffM1Ticks = abs(curM1TickCount - lastM1TickCount);

    DebugMsgs.debug().print("M0 ticks/second: ").print(diffM0Ticks * (1000/diffTime)).print(", speed: ").println(motorM0.speed);
    DebugMsgs.debug().print("M1 ticks/second: ").print(diffM1Ticks * (1000/diffTime)).print(", speed: ").println(motorM1.speed);
  }

  lastM0TickCount = curM0TickCount;
  lastM1TickCount = curM1TickCount;
  lastTickCountTime = curTickTime;
}
