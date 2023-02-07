//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

// Arduino includes
#include <Arduino.h>

// My includes
#include <TaskManager.h>

// Local includes
#include "pin_assignments.h"
#include "globals.h"
#include "interrupts.h"
#include "tasks.h"

ReadRCChannelTask readRCChannel1Task;
ReadRCChannelTask readRCChannel2Task;
AdjustMotorSpeedsTask adjustMotorSpeedsTask;

void setupPins() {
  // RC Channel inputs
  pinMode(RC_CH1_PIN, INPUT);
  pinMode(RC_CH2_PIN, INPUT);
      
  // Setup PWM pins, set to 0
  pinMode(M1_PWM_SPEED_PIN, OUTPUT);
  analogWrite(M1_PWM_SPEED_PIN, 0);
  pinMode(M2_PWM_SPEED_PIN, OUTPUT);
  analogWrite(M2_PWM_SPEED_PIN, 0);

  // TODO: put these under some kind of IFDEF
//  // Teensy 4.1 TODO: are these valid for Teensy 4.0?
//  // values will be 0-8191
  analogWriteResolution(13);
  analogWriteFrequency(M1_PWM_SPEED_PIN, 18310.55);
  analogWriteFrequency(M2_PWM_SPEED_PIN, 18310.55);

  //Teensy 3.5
  // values will be 0-4095
//  analogWriteResolution(12);
//  analogWriteFrequency(M1_PWM_SPEED_PIN, 14648.437);
//  analogWriteFrequency(M2_PWM_SPEED_PIN, 14648.437);

  // Setup brake pins, engage brake
  pinMode(M1_BRAKE_PIN, OUTPUT);
  digitalWrite(M1_BRAKE_PIN, HIGH);
  pinMode(M2_BRAKE_PIN, OUTPUT);
  digitalWrite(M2_BRAKE_PIN, HIGH);

  // Setup direction pins, set for forward
  pinMode(M1_MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(M1_MOTOR_DIR_PIN, LOW);
  pinMode(M2_MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(M2_MOTOR_DIR_PIN, LOW);

  // Set interrupt pins for M1
  pinMode(M1_U_ENCODER_SIGNAL_PIN, INPUT_PULLDOWN);
  motor1.prevUVal = digitalRead(M1_U_ENCODER_SIGNAL_PIN);
  attachInterrupt(M1_U_ENCODER_SIGNAL_PIN, countM1UTick, CHANGE);
  
  pinMode(M1_V_ENCODER_SIGNAL_PIN, INPUT_PULLDOWN);
  motor1.prevVVal = digitalRead(M1_V_ENCODER_SIGNAL_PIN);
  attachInterrupt(M1_V_ENCODER_SIGNAL_PIN, countM1VTick, CHANGE);
  
  pinMode(M1_W_ENCODER_SIGNAL_PIN, INPUT_PULLDOWN);
  motor1.prevWVal = digitalRead(M1_W_ENCODER_SIGNAL_PIN);
  attachInterrupt(M1_W_ENCODER_SIGNAL_PIN, countM1WTick, CHANGE);

  // Set interrupt pins for M2
  pinMode(M2_U_ENCODER_SIGNAL_PIN, INPUT_PULLDOWN);
  motor2.prevUVal = digitalRead(M2_U_ENCODER_SIGNAL_PIN);
  attachInterrupt(M2_U_ENCODER_SIGNAL_PIN, countM2UTick, CHANGE);
  
  pinMode(M2_V_ENCODER_SIGNAL_PIN, INPUT_PULLDOWN);
  motor2.prevVVal = digitalRead(M2_V_ENCODER_SIGNAL_PIN);
  attachInterrupt(M2_V_ENCODER_SIGNAL_PIN, countM2VTick, CHANGE);
  
  pinMode(M2_W_ENCODER_SIGNAL_PIN, INPUT_PULLDOWN);
  motor2.prevWVal = digitalRead(M2_W_ENCODER_SIGNAL_PIN);
  attachInterrupt(M2_W_ENCODER_SIGNAL_PIN, countM2WTick, CHANGE);
}

void setup() {
  readRCChannel1Task.setChannelPin(RC_CH1_PIN);
  readRCChannel2Task.setChannelPin(RC_CH2_PIN);

  adjustMotorSpeedsTask.setRCChannels(&readRCChannel1Task, &readRCChannel2Task);

  taskManager.addTask(&readRCChannel1Task, 50);
  taskManager.addTask(&readRCChannel2Task, 50);
  taskManager.addTask(&adjustMotorSpeedsTask, 100);
  
  taskManager.addBlinkTask(500);
  taskManager.startMonitoringButton(BUTTON_PIN, HIGH);
}

void loop() {
  taskManager.update();
}
