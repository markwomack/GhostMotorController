//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

#include <DebugMsgs.h>

#include "pin_assignments.h"
#include "globals.h"
#include "tasks.h"

const int NO_RC_SIGNAL(-10000);
const int MAX_SIGNAL_VAL(100);

const int MAX_SPEED(8191); //8191 this value is based on the frequency set on the PWM
const int MAX_SPEED_INCREMENT(500);
const float MOTOR_INCREMENT(((float)MAX_SPEED)/MAX_SIGNAL_VAL);

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

int readChannel(int channelPin, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelPin, HIGH, 30000);
  if (ch < 100) return defaultValue;
  DebugMsgs.debug().println(ch);
  return map(ch, 1020, 1980, minLimit, maxLimit);
}

void ReadRCChannelTask::setChannelPin(int8_t channelPin) {
  _channelPin = channelPin;
}

int ReadRCChannelTask::getValue(void) {
  return _value;
}

void ReadRCChannelTask::start(void) {
  _value = 0;
}

void ReadRCChannelTask::update(void) {
  _value = readChannel(_channelPin, -MAX_SIGNAL_VAL, MAX_SIGNAL_VAL, NO_RC_SIGNAL);
}

void AdjustMotorSpeedsTask::setRCChannels(ReadRCChannelTask* rcChannel1, ReadRCChannelTask* rcChannel2) {
  _rcChannel1 = rcChannel1;
  _rcChannel2 = rcChannel2;
}

void AdjustMotorSpeedsTask::setMotor1Info(String label, MotorEncoderInfo* motorEncoderInfo, uint8_t speedPin, uint8_t dirPin, uint8_t brakePin) {
  _motor1.label = label;
  _motor1.motorEncoderInfo = motorEncoderInfo;
  _motor1.speedPin = speedPin;
  _motor1.dirPin = dirPin;
  _motor1.brakePin = brakePin;
}

void AdjustMotorSpeedsTask::setMotor2Info(String label, MotorEncoderInfo* motorEncoderInfo, uint8_t speedPin, uint8_t dirPin, uint8_t brakePin) {
  _motor2.label = label;
  _motor2.motorEncoderInfo = motorEncoderInfo;
  _motor2.speedPin = speedPin;
  _motor2.dirPin = dirPin;
  _motor2.brakePin = brakePin;
}

void AdjustMotorSpeedsTask::start(void) {
  
  // Clear the counts
  _motor1.motorEncoderInfo->tickCount = _motor1.motorEncoderInfo->totalInterrupts = 0;
  _motor1.motorEncoderInfo->uTickCount = _motor1.motorEncoderInfo->vTickCount = _motor1.motorEncoderInfo->wTickCount = 0;
  _motor1.motorEncoderInfo->uFaultCount = _motor1.motorEncoderInfo->vFaultCount = _motor1.motorEncoderInfo->wFaultCount = 0;
  _motor2.motorEncoderInfo->tickCount = _motor2.motorEncoderInfo->totalInterrupts = 0;
  _motor2.motorEncoderInfo->uTickCount = _motor2.motorEncoderInfo->vTickCount = _motor2.motorEncoderInfo->wTickCount = 0;
  _motor2.motorEncoderInfo->uFaultCount = _motor2.motorEncoderInfo->vFaultCount = _motor2.motorEncoderInfo->wFaultCount = 0;
  
  _motor1.motorEncoderInfo->speed = 0;
  _motor1.motorEncoderInfo->motorDirection = false;  // false == forward, true == reverse
  _motor2.motorEncoderInfo->speed = 0;
  _motor2.motorEncoderInfo->motorDirection = false;  // false == forward, true == reverse

  // Set up the speed (0) and set the direction
  analogWrite(_motor1.speedPin, _motor1.motorEncoderInfo->speed);
  digitalWrite(_motor1.dirPin, _motor1.motorEncoderInfo->motorDirection ? HIGH : LOW);
  analogWrite(_motor2.speedPin, _motor2.motorEncoderInfo->speed);
  digitalWrite(_motor2.dirPin, _motor2.motorEncoderInfo->motorDirection ? HIGH : LOW);
  
  // Allow the motor to spin
  digitalWrite(_motor1.brakePin, LOW);
  digitalWrite(_motor2.brakePin, LOW);
}

void AdjustMotorSpeedsTask::update(void) {
  int motor1TargetSpeed = (int)(_rcChannel1->getValue() * MOTOR_INCREMENT);
  DebugMsgs.debug().print("Channel 1 value: ").println(_rcChannel1->getValue());
  int newM1Speed;
  if (motor1TargetSpeed < _motor1.motorEncoderInfo->speed) {
    // slowing down
    newM1Speed = max(motor1TargetSpeed, _motor1.motorEncoderInfo->speed - MAX_SPEED_INCREMENT);
  } else {
    // speeding up
    newM1Speed = min(motor1TargetSpeed, _motor1.motorEncoderInfo->speed + MAX_SPEED_INCREMENT);
  }
  if ((newM1Speed < 0 && _motor1.motorEncoderInfo->speed >= 0) || (newM1Speed >= 0 && _motor1.motorEncoderInfo->speed < 0)) {
    newM1Speed = 0;
  }

  int motor2TargetSpeed = (int)(_rcChannel1->getValue() * MOTOR_INCREMENT);
  DebugMsgs.debug().print("Channel 2 value: ").println(_rcChannel2->getValue());
  int newM2Speed;
  if (motor2TargetSpeed < _motor2.motorEncoderInfo->speed) {
    // slowing down
    newM2Speed = max(motor2TargetSpeed, _motor2.motorEncoderInfo->speed - MAX_SPEED_INCREMENT);
  } else {
    // speeding up
    newM2Speed = min(motor2TargetSpeed, _motor2.motorEncoderInfo->speed + MAX_SPEED_INCREMENT);
  }
  if ((newM2Speed < 0 && _motor2.motorEncoderInfo->speed >= 0) || (newM2Speed >= 0 && _motor2.motorEncoderInfo->speed < 0)) {
    newM2Speed = 0;
  }

  DebugMsgs.debug().print(_motor1.label).print(": old = ").print(_motor1.motorEncoderInfo->speed).print(", new = ").println(newM1Speed);
  DebugMsgs.debug().print(_motor2.label).print(": old = ").print(_motor2.motorEncoderInfo->speed).print(", new = ").println(newM2Speed);

  _motor1.motorEncoderInfo->speed = newM1Speed;
  _motor1.motorEncoderInfo->motorDirection = newM1Speed >= 0 ? false : true;
  _motor2.motorEncoderInfo->speed = newM2Speed;
  _motor2.motorEncoderInfo->motorDirection = newM2Speed >= 0 ? false : true;
  
  // Set the speed and set the direction
//  digitalWrite(_motor1.dirPin, _motor1.motorEncoderInfo->motorDirection ? HIGH : LOW);
//  digitalWrite(_motor2.dirPin, _motor2.motorEncoderInfo->motorDirection ? HIGH : LOW);
//  analogWrite(_motor1.speedPin, abs(_motor1.motorEncoderInfo->speed));
//  analogWrite(_motor2.speedPin, abs(_motor2.motorEncoderInfo->speed));
}

void AdjustMotorSpeedsTask::stop(void) {
  stopMotorsGradually();
}
