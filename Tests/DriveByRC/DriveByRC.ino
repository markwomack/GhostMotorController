//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

// This is a program to drive a MotorController
// instance using input from the RC channels.

// Arduino includes
#include <Arduino.h>

// Third Party includes
#include <DebugMsgs.h>   // https://github.com/markwomack/ArduinoLogging
#include <TaskManager.h> // https://github.com/markwomack/TaskManager
#include <BlinkTask.h>
#include <MotorAndEncoderManager.h>  // https://github.com/markwomack/MotorAndEncoderManager
#include <MotorController.h>         // https://github.com/markwomack/MotorAndEncoderManager
#include <GhostMotorManager.h>
#include <ThreePhaseMotorEncoder.h>

// Local includes
#include "globals.h"
#include "pin_assignments.h"

// Motor manager
MotorAndEncoderManager* motorManager;

// Motor Controller
MotorController* motorController;

BlinkTask idleTask;

// A task to set the desired motor speeds based on input
// from two RC signals.
class SetSpeedFromRCTask : public Task {
  public:
    void setRCPins(uint8_t chan1Pin, uint8_t chan2Pin) {
      _chan1Pin = chan1Pin;
      _chan2Pin = chan2Pin;
    };
    
    void start(void) {
      digitalWrite(M0_BRAKE_PIN, LOW);
      digitalWrite(M1_BRAKE_PIN, LOW);
      motorController->start();
      motorController->setDesiredSpeeds(0, 0);
    };
    
    void stop(void) {
      digitalWrite(M0_BRAKE_PIN, HIGH);
      digitalWrite(M1_BRAKE_PIN, HIGH);
      motorController->stop();
    };

    // Reads the current RC signal on the given pin, returns it
    // as a value between minLimit and maxLimit. Returns NO_RC_SIGNAL
    // if there is no RC signal.
    int readChannel(int channelPin, int minLimit, int maxLimit) {
      int ch = pulseIn(channelPin, HIGH, 30000);
      if (ch < 100) return NO_RC_SIGNAL;
      int value = map(ch, 1020, 1980, minLimit, maxLimit);
      return min(max(minLimit, value), maxLimit);
    }

    // Performs a controlled stop on both motors
    void performControlledStop(void) {
      motorController->setDesiredSpeeds(0, 0);
      uint32_t checkTime = 0;
      int32_t lastM0EncoderCount = 0;
      int32_t lastM1EncoderCount = 0;
      while(true) {
        if (millis() >= checkTime) {
          int32_t curM0EncoderCount = motorManager->readEncoder(M0);
          int32_t curM1EncoderCount = motorManager->readEncoder(M1);
          if (curM0EncoderCount - lastM0EncoderCount == 0 && curM1EncoderCount - lastM1EncoderCount == 0) {
            return;
          }
          lastM0EncoderCount = curM0EncoderCount;
          lastM1EncoderCount = curM1EncoderCount;
          checkTime = millis() + 5;
        }
      }
    };
    
    // Called periodically to set the desired moto speeds from the
    // the RC signals.
    void update(void) {

      // Read the current RC signals on both channels
      int ch1Val = readChannel(_chan1Pin, -100, 100);
      int ch2Val = readChannel(_chan2Pin, -100, 100);

      DebugMsgs.debug().print("RC Channels: ").print(ch1Val).print(" : ").println(ch2Val);

      // If either channel has been lost, then stop the robot, stop all the tasks.
      if (ch1Val == NO_RC_SIGNAL || ch2Val == NO_RC_SIGNAL) {
        DebugMsgs.debug().println("RC channel lost, stopping...");
        performControlledStop();
        taskManager.stop();
        return;
      }

      // maintain a value of zero if close to zero
      // avoids shimming
      if (abs(ch1Val) <= 5) {
        ch1Val = 0;
      }
      if (abs(ch2Val) <= 5) {
        ch2Val = 0;
      }

      // Channel 2 is for linear (forward/reverse) velocity
      double linearVelocity = ((double)ch2Val/100.0) * MAX_LINEAR_VELOCITY;

      // Channel 1 is for angular (left/right) velocity
      double angularVelocity = ((double)ch1Val/100.0) * MAX_ANGULAR_VELOCITY;

      // Calculate the motor speeds to match
      double m0Speed = linearVelocity - angularVelocity;
      double m1Speed = linearVelocity + angularVelocity;
      
      DebugMsgs.debug().print("Setting speeds: ").print(m0Speed).print(" : ").println(m1Speed);

      // Set the desired speeds on the motor controller
      motorController->setDesiredSpeeds(m0Speed, m1Speed);
    };

  private:
    uint8_t _chan1Pin;
    uint8_t _chan2Pin;
};
SetSpeedFromRCTask setSpeedFromRCTask;

// Task to allow the motor controller to periodically adjust
// the motor speeds to match the current desired speeds
class AdjustSpeedsTask : public Task {
  public:
    void update(void) {
      motorController->adjustSpeeds();
    };
};
AdjustSpeedsTask adjustSpeedsTask;

void setup() {
  Serial.begin(9600);

  // Comment out to disable debug output
  DebugMsgs.enableLevel(DEBUG);
  
  // RC Channel inputs
  pinMode(RC_CH1_PIN, INPUT);
  pinMode(RC_CH2_PIN, INPUT);

  // Setup the encoders
  ThreePhaseMotorEncoder* m0ThreePhaseEncoder = new ThreePhaseMotorEncoder();
  m0ThreePhaseEncoder->begin(M0_V_SIGNAL_PIN, M0_W_SIGNAL_PIN, M0_U_SIGNAL_PIN);
  ThreePhaseMotorEncoder* m1ThreePhaseEncoder = new ThreePhaseMotorEncoder();
  m1ThreePhaseEncoder->begin(M1_V_SIGNAL_PIN, M1_W_SIGNAL_PIN, M1_U_SIGNAL_PIN);

  // Setup the motor manager
  GhostMotorManager* ghostMotorManager = new GhostMotorManager();
  ghostMotorManager->begin(M0_SPEED_PIN, M0_DIR_PIN, M0_BRAKE_PIN,
  M1_SPEED_PIN, M1_DIR_PIN, M1_BRAKE_PIN);
  motorManager = (MotorAndEncoderManager*)ghostMotorManager;
  motorManager->setEncoders(m0ThreePhaseEncoder, m1ThreePhaseEncoder);

  DebugMsgs.debug().print("RADIANS_PER_TICK: ").println(RADIANS_PER_TICK);
  DebugMsgs.debug().print("MAX_RADIANS_PER_SECOND: ").println(MAX_RADIANS_PER_SECOND);
  DebugMsgs.debug().print("MAX_LINEAR_VELOCITY: ").println(MAX_LINEAR_VELOCITY);
  DebugMsgs.debug().print("MAX_ANGULAR_VELOCITY: ").println(MAX_ANGULAR_VELOCITY);

  // Create the motor controller
  motorController = new MotorController(motorManager, KP, KI, KD, 50, RADIANS_PER_TICK, MAX_RADIANS_PER_SECOND);

  // Set the RC pins into the setSpeedFromRCTask
  setSpeedFromRCTask.setRCPins(RC_CH1_PIN, RC_CH2_PIN);

  // Set up the task manager with tasks
  taskManager.addIdleTask(&idleTask, 100);
  taskManager.addBlinkTask(500);
  taskManager.addTask(&setSpeedFromRCTask, 50);
  taskManager.addTask(&adjustSpeedsTask, 10);

  // Wait for the user to press the button to start
  taskManager.startMonitoringButton(BUTTON_PIN, HIGH);
}

void loop() {
  taskManager.update();
}
