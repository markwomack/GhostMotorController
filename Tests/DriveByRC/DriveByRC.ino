//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

// This is a program to drive a MotorController
// instance using input from the RC channels.

// Arduino includes
#include <Arduino.h>

// Third Party includes
#include <DebugMsgs.h>               // https://github.com/markwomack/ArduinoLogging
#include <TaskManager.h>             // https://github.com/markwomack/TaskManager
#include <BlinkTask.h>
#include <MotorAndEncoderManager.h>  // https://github.com/markwomack/MotorAndEncoderManager
#include <MotorController.h>         // https://github.com/markwomack/MotorAndEncoderManager
#include <GhostMotorManager.h>
#include <ThreePhaseMotorEncoder.h>

// Local includes
#include "globals.h"
#include "pin_assignments.h"
#include "SpeedFromRCTask.h"
#include "AdjustSpeedsTask.h"
#include "CheckForOTATask.h"

// Motor manager
MotorAndEncoderManager* motorManager;

// Motor Controller
MotorController* motorController;

BlinkTask idleTask;
SpeedFromRCTask speedFromRCTask;
AdjustSpeedsTask adjustSpeedsTask;
CheckForOTATask checkForOTATask;

void setup() {
  Serial.begin(9600);
  Serial5.begin(115200);
  delay(1000); // Give serials a chance to catch up

  // Comment out to disable debug output
  DebugMsgs.enableLevel(DEBUG);

  pinMode(RC_ENABLE_PIN, INPUT);
  if (digitalRead(RC_ENABLE_PIN) == LOW) {
    DebugMsgs.debug().println("RC IS NOT ENABLED, exiting");
    Serial.flush();
    exit(0);
  }
  
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
  speedFromRCTask.setRCPins(RC_CH1_PIN, RC_CH2_PIN);

  // Set the needed references into the adjustSpeedsTask
  adjustSpeedsTask.setMotorManagerAndController(motorManager, motorController);
  adjustSpeedsTask.setSpeedFromRCTask(&speedFromRCTask);

  // Set the serial port to be monitored
  checkForOTATask.setSerial(&Serial5);

  // Set up the task manager with tasks
  taskManager.addIdleTask(&idleTask, 100);
  taskManager.addBlinkTask(500);
  taskManager.addTask(&speedFromRCTask, 50);
  taskManager.addTask(&adjustSpeedsTask, 10);
  taskManager.addTask(&checkForOTATask, 1000);

  // Wait for the user to press the button to start
  taskManager.startMonitoringButton(BUTTON_PIN, HIGH);
}

void loop() {
  taskManager.update();
  
  // If there is an OTA, stop everything and process
  if (checkForOTATask.otaIsAvailable()) {
    DebugMsgs.debug().println("OTA update available, processing...");

    // stop normal operation
    taskManager.stop();

    // perform the OTA update
    checkForOTATask.performUpdate();
  }
}
