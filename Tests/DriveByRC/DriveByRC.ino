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
#include <RampedMotorPID.h>

// Local includes
#include "globals.h"
#include "pin_assignments.h"
#include "ReadFromRCTask.h"
#include "AdjustSpeedsTask.h"
#include "CheckForOTATask.h"

// Motor manager
MotorAndEncoderManager* motorManager;

// Motor Controller
MotorController* motorController;

#define SERIAL_BUFFER_SIZE 8192
uint8_t incomingBuffer[SERIAL_BUFFER_SIZE];
uint8_t outgoingBuffer[SERIAL_BUFFER_SIZE];

ReadFromRCTask readFromRCTask;
AdjustSpeedsTask adjustSpeedsTask;
CheckForOTATask checkForOTATask;

void setup() {
  Serial.begin(115200);
  Serial5.begin(115200);
  Serial5.addMemoryForRead(incomingBuffer, SERIAL_BUFFER_SIZE);
  Serial5.addMemoryForWrite(outgoingBuffer, SERIAL_BUFFER_SIZE);
  delay(1000); // Give serials a chance to catch up

  // Comment out to disable debug output
  DebugMsgs.enableLevel(DEBUG);

  // All output now goes to Serial5
  DebugMsgs.setPrint(new FlushingPrintWrapper(new PrintWrapper(&Serial5)));

  DebugMsgs.debug().println("Starting Drive By RC");
  
  pinMode(RC_ENABLE_PIN, INPUT);
  if (digitalRead(RC_ENABLE_PIN) == LOW) {
    DebugMsgs.debug().println("RC IS NOT ENABLED, exiting");
    DebugMsgs.flush();
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

  // Should MAX_SPEED_INCREMENT_ALLOWED be adjusted to the frequency adjustSpeedsTask will be called?
  MotorPID* m0MotorPID = new RampedMotorPID(MAX_SPEED_INCREMENT_ALLOWED);
  MotorPID* m1MotorPID = new RampedMotorPID(MAX_SPEED_INCREMENT_ALLOWED);
  
  // Create the motor controller
  motorController = new MotorController(motorManager, m0MotorPID, m1MotorPID, 50, RADIANS_PER_TICK, MAX_RADIANS_PER_SECOND);

  DebugMsgs.debug().print("RADIANS_PER_TICK: ").println(RADIANS_PER_TICK);
  DebugMsgs.debug().print("MAX_RADIANS_PER_SECOND: ").println(MAX_RADIANS_PER_SECOND);
  DebugMsgs.debug().print("MAX_LINEAR_VELOCITY_ALLOWED: ").println(MAX_LINEAR_VELOCITY_ALLOWED);
  DebugMsgs.debug().print("MAX_ANGULAR_VELOCITY_ALLOWED: ").println(MAX_ANGULAR_VELOCITY_ALLOWED);
  DebugMsgs.debug().print("MAX_SPEED_INCREMENT_ALLOWED: ").println(MAX_SPEED_INCREMENT_ALLOWED);
  
  // Set the RC pins into the readFromRCTask
  readFromRCTask.setRCPins(RC_CH1_PIN, RC_CH2_PIN);

  // Set the needed references into the adjustSpeedsTask
  adjustSpeedsTask.setMotorManagerAndController(motorManager, motorController);
  adjustSpeedsTask.setReadFromRCTask(&readFromRCTask);

  // Set the serial port to be monitored
  checkForOTATask.setSerial(&Serial5);

  // Set up the task manager with idle tasks
  taskManager.addIdleBlinkTask(100);
  taskManager.addIdleTask(&checkForOTATask, 1000);

  // Set up the task manager with regular tasks
  taskManager.addBlinkTask(500);
  taskManager.addTask(&readFromRCTask, 50);
  taskManager.addTask(&adjustSpeedsTask, 50);
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

    // update aborted, restart task manager
    taskManager.start();
  }
}
