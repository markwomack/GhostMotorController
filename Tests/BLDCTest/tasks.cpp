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
#include "pin_assignments.h"
#include "globals.h"

// TODO: are these processor model specific?
const int speedIncrement(100); //200
const int maxSpeed(8191); //8191 this value is based on the frequency set on the PWM

const int32_t NUM_TICKS_PER_ROTATION(120);

void stopMotorsGradually() {
  if (motor1.speed > 0 || motor2.speed > 0) {
    DebugMsgs.debug().println("Stopping motors...");
    // Spin down the motor in controlled manner
    while (motor1.speed > 0 || motor2.speed > 0) {
      motor1.speed = max(motor1.speed - 500, 0);
      analogWrite(M1_PWM_SPEED_PIN, motor1.speed);
      motor2.speed = max(motor2.speed - 500, 0);
      analogWrite(M2_PWM_SPEED_PIN, motor2.speed);
      delay(500);
    }
    // Don't allow the motor to spin
    digitalWrite(M1_BRAKE_PIN, HIGH);
    digitalWrite(M2_BRAKE_PIN, HIGH);
    DebugMsgs.debug().println("...motors stopped.");
  }
}

void stopMotorsImmediately() {
  analogWrite(M1_PWM_SPEED_PIN, 0);
  digitalWrite(M1_BRAKE_PIN, HIGH);
  analogWrite(M2_PWM_SPEED_PIN, 0);
  digitalWrite(M2_BRAKE_PIN, HIGH);
  motor1.speed = 0; 
  motor2.speed = 0; 
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
      // taskManager.stop();
      // return;
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
  //DebugMsgs.debug().print("Speed: ").println(_motor->speed);
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
  int32_t currentTicks = motor1.tickCount;
  interrupts();
  //DebugMsgs.debug().print("CountRotationsTask motor1.tickCount = ").println(motor1.tickCount);
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
  lastM1TickCount = 0;
  lastM2TickCount = 0;
  lastTickCountTime = 0;      
};

void PrintTickSpeedTask::update(void) {
  int32_t curM1TickCount = motor1.tickCount;
  int32_t curM2TickCount = motor2.tickCount;
  int32_t curTickTime = millis();
  
  if (lastTickCountTime != 0) {
    int32_t diffTime = curTickTime - lastTickCountTime;
    int32_t diffM1Ticks = abs(curM1TickCount - lastM1TickCount);
    int32_t diffM2Ticks = abs(curM2TickCount - lastM2TickCount);

    DebugMsgs.debug().print("M1 ticks/second: ").print(diffM1Ticks * (1000/diffTime)).print(", speed: ").println(motor1.speed);
    DebugMsgs.debug().print("M2 ticks/second: ").print(diffM2Ticks * (1000/diffTime)).print(", speed: ").println(motor2.speed);
  }

  lastM1TickCount = curM1TickCount;
  lastM2TickCount = curM2TickCount;
  lastTickCountTime = curTickTime;
}