//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

// This is a test program that can be used to test the connections
// to the motor controller and motor encoder hardware. It doesn't use
// any pre-built libraries to interface with the controller or encoders,
// everything is done directly to aid in debugging. From this program
// you can add tasks to drive the motors forward and reverse, print the
// tick and fault counts from the motor encoders, calculate the speed
// in tick/second, or determine how many ticks/revolution your particular
// motors support. Ideally, after using these tasks you will know:
//   1) How many ticks per/revolution
//   2) That your motor encoders are not experiencing widespread faults
//      (though faults recorded when motors are switching directions is
//      acceptable)
//   3) The ticks/second when the motors are at full power.
// You can comment out code related to motor M1 or M0 if you want to work
// with a single motor for debugging.
//
// This sketch assumes there is a momentary push button connected to BUTTON_PIN
// and that it is by default connected HIGH (and will go LOW when pressed).
// Pressing the button will start/stop the tasks.

// Arduino includes
#include <Arduino.h>
#include <inttypes.h>

// My includes
#include <DebugMsgs.h>   // https://github.com/markwomack/ArduinoLogging
#include <TaskManager.h> // https://github.com/markwomack/TaskManager
#include <Task.h>

// Local includes
#include "pin_assignments.h"
#include "globals.h"
#include "interrupts.h"
#include "tasks.h"

ControlMotorTask controlM0Task;
ControlMotorTask controlM1Task;
PrintMotorTickCountsTask printM0TickCountsTask;
PrintMotorTickCountsTask printM1TickCountsTask;
CountRotationsTask countM0RotationsTask;
PrintTickSpeedTask printTickSpeedTask;

BlinkTask idleTask;

void setupPins() {
  // Setup PWM pins, set to 0
  pinMode(M0_SPEED_PIN, OUTPUT);
  analogWrite(M0_SPEED_PIN, 0);
  pinMode(M1_SPEED_PIN, OUTPUT);
  analogWrite(M1_SPEED_PIN, 0);

  // Teensy 4.1 specific
  // Values will be 0-8191
  analogWriteResolution(13);
  analogWriteFrequency(M0_SPEED_PIN, 18310.55);
  analogWriteFrequency(M1_SPEED_PIN, 18310.55);

  // Teensy 3.5 specific 
  // Values will be 0-4095
//  analogWriteResolution(12);
//  analogWriteFrequency(M0_SPEED_PIN, 14648.437);
//  analogWriteFrequency(M1_SPEED_PIN, 14648.437);

  // Setup brake pins, engage brake
  pinMode(M0_BRAKE_PIN, OUTPUT);
  digitalWrite(M0_BRAKE_PIN, HIGH);
  pinMode(M1_BRAKE_PIN, OUTPUT);
  digitalWrite(M1_BRAKE_PIN, HIGH);

  // Setup direction pins, set for forward
  pinMode(M0_DIR_PIN, OUTPUT);
  digitalWrite(M0_DIR_PIN, LOW);
  pinMode(M1_MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(M1_MOTOR_DIR_PIN, LOW);

  // Set interrupt pins for M0
  pinMode(M0_U_SIGNAL_PIN, INPUT_PULLDOWN);
  motorM0.prevUVal = digitalRead(M0_U_SIGNAL_PIN);
  attachInterrupt(M0_U_SIGNAL_PIN, countM0UTick, CHANGE);
  
  pinMode(M0_V_SIGNAL_PIN, INPUT_PULLDOWN);
  motorM0.prevVVal = digitalRead(M0_V_SIGNAL_PIN);
  attachInterrupt(M0_V_SIGNAL_PIN, countM0VTick, CHANGE);
  
  pinMode(M0_W_SIGNAL_PIN, INPUT_PULLDOWN);
  motorM0.prevWVal = digitalRead(M0_W_SIGNAL_PIN);
  attachInterrupt(M0_W_SIGNAL_PIN, countM0WTick, CHANGE);

  // Set interrupt pins for M1
  pinMode(M1_U_SIGNAL_PIN, INPUT_PULLDOWN);
  motorM1.prevUVal = digitalRead(M1_U_SIGNAL_PIN);
  attachInterrupt(M1_U_SIGNAL_PIN, countM1UTick, CHANGE);
  
  pinMode(M1_V_SIGNAL_PIN, INPUT_PULLDOWN);
  motorM1.prevVVal = digitalRead(M1_V_SIGNAL_PIN);
  attachInterrupt(M1_V_SIGNAL_PIN, countM1VTick, CHANGE);
  
  pinMode(M1_W_SIGNAL_PIN, INPUT_PULLDOWN);
  motorM1.prevWVal = digitalRead(M1_W_SIGNAL_PIN);
  attachInterrupt(M1_W_SIGNAL_PIN, countM1WTick, CHANGE);
}

void setup() {
  Serial.begin(9600);
  // give serial some time to catch up
  delay(1000);

  // Comment out to disable all debug messages
  DebugMsgs.enableLevel(DEBUG);

  setupPins();

  taskManager.addIdleTask(&idleTask, 100);
  taskManager.addBlinkTask(500);

  // If you only have one motor, you can comment out
  // the motor tasks for the other motor.
  
  // Motor M0 control and count tasks
  controlM0Task.setup(&motorM0, "M0", M0_SPEED_PIN, M0_DIR_PIN, M0_BRAKE_PIN);
  taskManager.addTask(&controlM0Task, 500);
  printM0TickCountsTask.setup(&motorM0, "M0");
  taskManager.addTask(&printM0TickCountsTask, 1000);
  
  // Motor M1 control and count tasks
  controlM1Task.setup(&motorM1, "M1", M1_SPEED_PIN, M1_MOTOR_DIR_PIN, M1_BRAKE_PIN);
  taskManager.addTask(&controlM1Task, 500);
  printM1TickCountsTask.setup(&motorM1, "M1");
  taskManager.addTask(&printM1TickCountsTask, 1000);

  // Task to help estimate ticks/rotation
  // countM0RotationsTask.setRotations(5);
  // taskManager.addTask(&countM0RotationsTask, 1);

  // Task to print motor speeds
  taskManager.addTask(&printTickSpeedTask, 1000);

  // Wait for the user to press the button to start
  taskManager.startMonitoringButton(BUTTON_PIN, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  taskManager.update();
}
