//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

// Arduino includes
#include <Arduino.h>

// Third Party includes
#include <DebugMsgs.h>
#include <TaskManager.h>
#include <BlinkTask.h>
#include <MotorAndEncoderManager.h>
#include <MotorController.h>
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

class SetSpeedFromRC : public Task {
  public:
    void setRCPins(uint8_t chan1Pin, uint8_t chan2Pin) {
      _chan1Pin = chan1Pin;
      _chan2Pin = chan2Pin;
    };
    
    void start(void) {
      digitalWrite(M1_BRAKE_PIN, LOW);
      digitalWrite(M2_BRAKE_PIN, LOW);
      motorController->start();
      motorController->setDesiredSpeeds(0, 0);
    };
    
    void stop(void) {
      digitalWrite(M1_BRAKE_PIN, HIGH);
      digitalWrite(M2_BRAKE_PIN, HIGH);
      motorController->stop();
    };
    
    int readChannel(int channelPin, int minLimit, int maxLimit, int defaultValue) {
      int ch = pulseIn(channelPin, HIGH, 30000);
      if (ch < 100) return defaultValue;
      int value = map(ch, 1020, 1980, minLimit, maxLimit);
      return min(max(minLimit, value), maxLimit);
    }
    
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
    
    void update(void) {
      int ch1Val = readChannel(_chan1Pin, -100, 100, NO_RC_SIGNAL);
      int ch2Val = readChannel(_chan2Pin, -100, 100, NO_RC_SIGNAL);
      
      if (ch1Val == NO_RC_SIGNAL || ch2Val == NO_RC_SIGNAL) {
        DebugMsgs.debug().println("RC channel lost, stopping...");
        performControlledStop();
        taskManager.stop();
      }
      
      DebugMsgs.debug().print("RC Channels: ").print(ch1Val).print(" : ").println(ch2Val);
            
      double linearVelocity = ((double)ch2Val/100.0) * MAX_LINEAR_VELOCITY;
      double angularVelocity = ((double)ch1Val/100.0) * MAX_ANGULAR_VELOCITY;
      
      double m0Speed = linearVelocity - angularVelocity;
      double m1Speed = linearVelocity + angularVelocity;
      
      DebugMsgs.debug().print("Setting speeds: ").print(m0Speed).print(" : ").println(m1Speed);
      
      motorController->setDesiredSpeeds(m0Speed, m1Speed);
    };
};
SetSpeedsTask setSpeedsTask;

class AdjustSpeedsTask : public Task {
  public:
    void update(void) {
      motorController->adjustSpeeds();
    };
};
AdjustSpeedsTask adjustSpeedsTask;

void setup() {
  Serial.begin(9600);
  
  DebugMsgs.enableLevel(DEBUG);
  
  // RC Channel inputs
  pinMode(RC_CH1_PIN, INPUT);
  pinMode(RC_CH2_PIN, INPUT);

  // Setup the encoders
  ThreePhaseMotorEncoder* m0ThreePhaseEncoder = new ThreePhaseMotorEncoder();
  m0ThreePhaseEncoder->begin(M0_V_SIGNAL_PIN, M0_W_SIGNAL_PIN, M0_U_SIGNAL_PIN);
  m0Encoder = m0ThreePhaseEncoder;
  ThreePhaseMotorEncoder* m1ThreePhaseEncoder = new ThreePhaseMotorEncoder();
  m1ThreePhaseEncoder->begin(M1_V_SIGNAL_PIN, M1_W_SIGNAL_PIN, M1_U_SIGNAL_PIN);
  m1Encoder = m1ThreePhaseEncoder;

  // Setup the motor manager
  GhostMotorManager* ghostMotorManager = new GhostMotorManager();
  ghostMotorManager->begin(M0_SPEED_PIN, M0_DIR_PIN, M0_BRAKE_PIN,
  M1_SPEED_PIN, M1_DIR_PIN, M1_BRAKE_PIN);
  motorManager = (MotorAndEncoderManager*)ghostMotorManager;
  motorManager->setEncoders(m0Encoder, m1Encoder);

  DebugMsgs.debug().print("RADIANS_PER_TICK: ").println(RADIANS_PER_TICK);
  DebugMsgs.debug().print("MAX_RADIANS_PER_SECOND: ").println(MAX_RADIANS_PER_SECOND);
  DebugMsgs.debug().print("MAX_LINEAR_VELOCITY: ").println(MAX_LINEAR_VELOCITY);
  DebugMsgs.debug().print("MAX_ANGULAR_VELOCITY: ").println(MAX_ANGULAR_VELOCITY);

  motorController = new MotorController(motorManager, KP, KI, KD, 50, RADIANS_PER_TICK, MAX_RADIANS_PER_SECOND);
  
  taskManager.addIdleTask(&idleTask, 100);
  taskManager.addBlinkTask(500);
  taskManager.addTask(&adjustSpeedsTask, 10);
  taskManager.addTask(&setSpeedsTask, 50);
  
  taskManager.startMonitoringButton(BUTTON_PIN, HIGH);
}

void loop() {
  taskManager.update();
}
