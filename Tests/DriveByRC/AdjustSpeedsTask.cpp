//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

#include <Arduino.h>

#include <DebugMsgs.h>
#include <TaskManager.h>

#include "pin_assignments.h"
#include "globals.h"
#include "AdjustSpeedsTask.h"

AdjustSpeedsTask::AdjustSpeedsTask() {
  _useMotorController = false;
}

void AdjustSpeedsTask::useMotorController(bool useMotorController) {
  _useMotorController = useMotorController;
}

void AdjustSpeedsTask::setMotorManagerAndController(
    MotorAndEncoderManager* motorManager, MotorController* motorController) {
  _motorManager = motorManager;
  _motorController = motorController;
}

void AdjustSpeedsTask::setSpeedFromRCTask(SpeedFromRCTask* speedFromRCTask) {
  _speedFromRCTask = speedFromRCTask;
}

void AdjustSpeedsTask::start(void) {
  digitalWrite(M0_BRAKE_PIN, LOW);
  digitalWrite(M1_BRAKE_PIN, LOW);
  _targetM0Speed = 0;
  _targetM1Speed = 0;
  _stopped = false;
  if (_useMotorController) {
    _motorController->start();
    _motorController->setDesiredSpeeds(_targetM0Speed, _targetM1Speed);
  } else {
    _motorManager->setMotorSpeeds(_targetM0Speed, _targetM1Speed);
  }
}

void AdjustSpeedsTask::stop(void) {
  if (!_stopped) {
    performControlledStop();
  }
  digitalWrite(M0_BRAKE_PIN, HIGH);
  digitalWrite(M1_BRAKE_PIN, HIGH);
}

// Performs a controlled stop on both motors
void AdjustSpeedsTask::performControlledStop(void) {
  if (_useMotorController) {
    _motorController->setDesiredSpeeds(0, 0);
  } else {
    _motorManager->setMotorSpeeds(0, 0);
  }
  uint32_t checkTime = 0;
  int32_t lastM0EncoderCount = 0;
  int32_t lastM1EncoderCount = 0;
  while(true) {
    if (millis() >= checkTime) {
      int32_t curM0EncoderCount = _motorManager->readEncoder(M0);
      int32_t curM1EncoderCount = _motorManager->readEncoder(M1);
      if (curM0EncoderCount - lastM0EncoderCount == 0 && curM1EncoderCount - lastM1EncoderCount == 0) {
        return;
      }
      lastM0EncoderCount = curM0EncoderCount;
      lastM1EncoderCount = curM1EncoderCount;
      checkTime = millis() + 50;
    }
  }
  
  if (_useMotorController) {
    _motorController->stop();
  }
  _stopped = true;
}

// Called periodically to set the desired motor speeds from the
// the RC signals.
void AdjustSpeedsTask::update(void) {

  double userM0Speed = _speedFromRCTask->getM0Speed();
  double userM1Speed = _speedFromRCTask->getM1Speed();
  
  // Set the desired speeds on the motor controller
  if (_useMotorController) {
    _targetM0Speed = userM0Speed;
    _targetM1Speed = userM1Speed;
    _motorController->setDesiredSpeeds(_targetM0Speed, _targetM1Speed);
    _motorController->adjustSpeeds();
  } else {
    // set speed according to an acceleration/deceleration ramp
    _motorManager->setMotorSpeeds(_targetM0Speed, _targetM1Speed);
  }
}
