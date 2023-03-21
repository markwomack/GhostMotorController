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

// Control and print stats for motor M0
ControlMotorTask controlM0Task;
PrintMotorTickCountsTask printM0TickCountsTask;

// Control and print stats for motor M1
ControlMotorTask controlM1Task;
PrintMotorTickCountsTask printM1TickCountsTask;

// Rotate motor M0 for n rotations
CountRotationsTask countM0RotationsTask;

// Print speeds for both motors
PrintTickSpeedTask printTickSpeedTask;

// BlinkTask to indicate idle state (ie waiting for button
// to be pressed).
BlinkTask idleTask;

void setupPins() {
  // Setup PWM pins, set to 0
  pinMode(M0_SPEED_PIN, OUTPUT);
  analogWrite(M0_SPEED_PIN, 0);
  pinMode(M1_SPEED_PIN, OUTPUT);
  analogWrite(M1_SPEED_PIN, 0);

  // Teensy 4.1 specific
  // PWM values will be 0-8191
  analogWriteResolution(13);
  analogWriteFrequency(M0_SPEED_PIN, 18310.55);
  analogWriteFrequency(M1_SPEED_PIN, 18310.55);

   // Teensy 3.5 specific
   // PWM values will be 0-4095
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
  pinMode(M1_DIR_PIN, OUTPUT);
  digitalWrite(M1_DIR_PIN, LOW);

  // Set interrupt pins for M0, attach interrupts
  pinMode(M0_U_ENCODER_SIGNAL_PIN, INPUT_PULLDOWN);
  motorEncoderM0.prevUVal = digitalRead(M0_U_ENCODER_SIGNAL_PIN);
  attachInterrupt(M0_U_ENCODER_SIGNAL_PIN, countM0UTick, CHANGE);
  
  pinMode(M0_V_ENCODER_SIGNAL_PIN, INPUT_PULLDOWN);
  motorEncoderM0.prevVVal = digitalRead(M0_V_ENCODER_SIGNAL_PIN);
  attachInterrupt(M0_V_ENCODER_SIGNAL_PIN, countM0VTick, CHANGE);
  
  pinMode(M0_W_ENCODER_SIGNAL_PIN, INPUT_PULLDOWN);
  motorEncoderM0.prevWVal = digitalRead(M0_W_ENCODER_SIGNAL_PIN);
  attachInterrupt(M0_W_ENCODER_SIGNAL_PIN, countM0WTick, CHANGE);

  // Set interrupt pins for M1, attach interrupts
  pinMode(M1_U_ENCODER_SIGNAL_PIN, INPUT_PULLDOWN);
  motorEncoderM1.prevUVal = digitalRead(M1_U_ENCODER_SIGNAL_PIN);
  attachInterrupt(M1_U_ENCODER_SIGNAL_PIN, countM1UTick, CHANGE);
  
  pinMode(M1_V_ENCODER_SIGNAL_PIN, INPUT_PULLDOWN);
  motorEncoderM1.prevVVal = digitalRead(M1_V_ENCODER_SIGNAL_PIN);
  attachInterrupt(M1_V_ENCODER_SIGNAL_PIN, countM1VTick, CHANGE);
  
  pinMode(M1_W_ENCODER_SIGNAL_PIN, INPUT_PULLDOWN);
  motorEncoderM1.prevWVal = digitalRead(M1_W_ENCODER_SIGNAL_PIN);
  attachInterrupt(M1_W_ENCODER_SIGNAL_PIN, countM1WTick, CHANGE);
}

void setup() {
  Serial.begin(9600);
  // give serial some time to catch up
  delay(1000);

  // Comment out to disable debug messages
  DebugMsgs.enableLevel(DEBUG);

  setupPins();

  taskManager.addIdleTask(&idleTask, 100);
  taskManager.addBlinkTask(500);

  // If you only have one motor, you can comment out
  // the motor tasks for the other motor.
  
  // Motor M0
  // If you don't add any tasks to control the motor,
  // you can manually turn the motor to see how many
  // ticks/rotation. Just be sure to turn the brake off
  // first.
  controlM0Task.setup(&motorEncoderM0, "M0", M0_SPEED_PIN, M0_DIR_PIN, M0_BRAKE_PIN);
  taskManager.addTask(&controlM0Task, 500);
  printM0TickCountsTask.setup(&motorEncoderM0, "M0");
  taskManager.addTask(&printM0TickCountsTask, 1000);
  
  // Motor M1
  controlM1Task.setup(&motorEncoderM1, "M1", M1_SPEED_PIN, M1_DIR_PIN, M1_BRAKE_PIN);
  taskManager.addTask(&controlM1Task, 500);
  printM1TickCountsTask.setup(&motorEncoderM1, "M1");
  taskManager.addTask(&printM1TickCountsTask, 1000);

  // Uncomment if you want to test rotations/tick
  // countM0RotationsTask.setRotations(5);
  // taskManager.addTask(&countM0RotationsTask, 1);
  
  taskManager.addTask(&printTickSpeedTask, 1000);

  // Wait for the user to press the button to start
  taskManager.startMonitoringButton(BUTTON_PIN, HIGH);
}

void loop() {
  taskManager.update();
}
