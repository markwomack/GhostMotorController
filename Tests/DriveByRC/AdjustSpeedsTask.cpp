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
  // does nothing
}

void AdjustSpeedsTask::setMotorManagerAndController(
    MotorAndEncoderManager* motorManager, MotorController* motorController) {
  _motorManager = motorManager;
  _motorController = motorController;
}

void AdjustSpeedsTask::setReadFromRCTask(ReadFromRCTask* readFromRCTask) {
  _readFromRCTask = readFromRCTask;
}

void AdjustSpeedsTask::start(void) {
  digitalWrite(M0_BRAKE_PIN, LOW);
  digitalWrite(M1_BRAKE_PIN, LOW);
  _stopped = false;
  _motorController->start();
  _motorController->setDesiredSpeeds(0, 0);
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
  DebugMsgs.debug().println("Performing controlled stop");
  // Turn of the motor controller
  _motorController->disengage();
  
  // Convert target speeds to values between -1 and 1
  double _targetM0Speed = _motorController->getLastM0SetSpeed()/MAX_RADIANS_PER_SECOND;
  double _targetM1Speed = _motorController->getLastM1SetSpeed()/MAX_RADIANS_PER_SECOND;

  uint32_t checkTime = 0;
  int32_t lastM0EncoderCount = 0;
  int32_t lastM1EncoderCount = 0;
  while(true) {
    if (millis() >= checkTime) {
      int32_t curM0EncoderCount = _motorManager->readEncoder(M0);
      int32_t curM1EncoderCount = _motorManager->readEncoder(M1);
      if (curM0EncoderCount - lastM0EncoderCount == 0 && curM1EncoderCount - lastM1EncoderCount == 0) {
        break;
      }
      lastM0EncoderCount = curM0EncoderCount;
      lastM1EncoderCount = curM1EncoderCount;

      // Do a controlled slow down to 0 for both motors
      if (_targetM0Speed > 0) {
        _targetM0Speed -= 0.1;
        if (_targetM0Speed < 0) {
          _targetM0Speed = 0;
        }
      } else if (_targetM0Speed < 0) {
        _targetM0Speed += 0.1;
        if (_targetM0Speed > 0) {
          _targetM0Speed = 0;
        }
      }
      if (_targetM1Speed > 0) {
        _targetM1Speed -= 0.1;
        if (_targetM1Speed < 0) {
          _targetM1Speed = 0;
        }
      } else if (_targetM1Speed < 0) {
        _targetM1Speed += 0.1;
        if (_targetM1Speed > 0) {
          _targetM1Speed = 0;
        }
      }
      _motorManager->setMotorSpeeds(_targetM0Speed, _targetM1Speed);
      checkTime = millis() + 50;
    }
  }

  _motorManager->setMotorSpeeds(0, 0);
  _stopped = true;
  DebugMsgs.debug().println("All motors are stopped");
}

// Called periodically to set the desired motor speeds from the
// the RC signals.
void AdjustSpeedsTask::update(void) {

  // Get RC signal values between -1 and 1
  double signal1Value = _readFromRCTask->getSignal1Value();
  double signal2Value = _readFromRCTask->getSignal2Value();
  
  // Channel 2 is for linear (forward/reverse) velocity in radians/second
  double _linearVelocity = signal2Value * MAX_LINEAR_VELOCITY_ALLOWED;

  // Channel 1 is for angular (left/right) velocity in radians/second
  double _angularVelocity = signal1Value * MAX_ANGULAR_VELOCITY_ALLOWED;

  // Calculate the motor speeds to match, in radians/second
  double desiredM0Speed = _linearVelocity - _angularVelocity;
  double desiredM1Speed = _linearVelocity + _angularVelocity;
  
  // Set the desired speeds on the motor controller, adjust speeds
  _motorController->setDesiredSpeeds(desiredM0Speed, desiredM1Speed);
  _motorController->adjustSpeeds();
//  } else {
//    // Convert desired speeds to values between -1 and 1
//    desiredM0Speed = desiredM0Speed/MAX_RADIANS_PER_SECOND;
//    desiredM1Speed = desiredM1Speed/MAX_RADIANS_PER_SECOND;
//    
//    // set speed according to an acceleration/deceleration ramp
//    double m0Diff = desiredM0Speed - _targetM0Speed;
//    double m0Increment = (m0Diff >= 0) ? min(MAX_SPEED_INCREMENT, m0Diff) : max(-MAX_SPEED_INCREMENT, m0Diff);
//    double m1Diff = desiredM1Speed - _targetM1Speed;
//    double m1Increment = (m1Diff >= 0) ? min(MAX_SPEED_INCREMENT, m1Diff) : max(-MAX_SPEED_INCREMENT, m1Diff);
//    
//    _targetM0Speed += m0Increment;
//    _targetM1Speed += m1Increment;
//    
//    DebugMsgs.debug().print("Setting motor speeds: ").print(_targetM0Speed).print(" ").print(_targetM1Speed)
//      .print(" ").print(m0Diff).print(" ").println(m1Diff);
//
//    _motorManager->setMotorSpeeds(_targetM0Speed, _targetM1Speed);
//  }
}
