

// Arduino includes
#include <Arduino.h>
#include <inttypes.h>

// My includes
#include <DebugMsgs.h>
#include <TaskManager.h>
#include <Task.h>

// Local includes
#include "globals.h"
#include "pin_assignments.h"
#include "interrupts.h"

// TODO: are these processor model specific?
const int speedIncrement(200); //200
const int maxSpeed(5000); //5000

void stopMotors() {
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
    DebugMsgs.debug().println("...motor stopped.");
}

void setupPins() {
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
  attachInterrupt(M1_V_ENCODER_SIGNAL_PIN, countM2VTick, CHANGE);
  
  pinMode(M1_W_ENCODER_SIGNAL_PIN, INPUT_PULLDOWN);
  motor1.prevWVal = digitalRead(M1_W_ENCODER_SIGNAL_PIN);
  attachInterrupt(M1_W_ENCODER_SIGNAL_PIN, countM2WTick, CHANGE);

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

class ControlM1Task : public Task {
  public:
    ControlM1Task() {};

    void start(void) {
      motor1.incrementDirection = true; // false == slow it down, true == speed it up
      motor1.speed = 0;
      motor1.motorDirection = false;  // false == forward, true == reverse

      // Set up the speed (0) and set the direction
      analogWrite(M1_PWM_SPEED_PIN, motor1.speed);
      digitalWrite(M1_MOTOR_DIR_PIN, motor1.motorDirection ? HIGH : LOW);
      
      // Allow the motor to spin
      digitalWrite(M1_BRAKE_PIN, LOW);
    };

    void update(void) {
      // Motor is accelerating  
      if (motor1.incrementDirection) {
        motor1.speed = min(motor1.speed + speedIncrement, maxSpeed);

        // When we match the max speed, then start decelerating
        if (motor1.speed == maxSpeed) {
          motor1.incrementDirection = !motor1.incrementDirection;
        }
      // Motor is decelerating   
      } else {
        motor1.speed = max(motor1.speed - speedIncrement, 0);
        
        // When we reach 0, then switch the direction of the motor
        if (motor1.speed == 0) {
          motor1.incrementDirection = !motor1.incrementDirection;
          motor1.motorDirection = !motor1.motorDirection;
          digitalWrite(M1_MOTOR_DIR_PIN, motor1.motorDirection ? HIGH : LOW);
        }
      }
      //DebugMsgs.debug().print("Speed: ").println(speed);
      // Update the speed
      analogWrite(M1_PWM_SPEED_PIN, motor1.speed);
    };

    void stop(void) {
      stopMotors();
    };
};
ControlM1Task controlM1Task;

class ControlM2Task : public Task {
  public:
    ControlM2Task() {};

    void start(void) {
      motor2.incrementDirection = true; // false == slow it down, true == speed it up
      motor2.speed = 0;
      motor2.motorDirection = false;  // false == forward, true == reverse

      // Set up the speed (0) and set the direction
      analogWrite(M2_PWM_SPEED_PIN, motor2.speed);
      digitalWrite(M2_MOTOR_DIR_PIN, motor2.motorDirection ? HIGH : LOW);
      
      // Allow the motor to spin
      digitalWrite(M2_BRAKE_PIN, LOW);
    };

    void update(void) {
      // Motor is accelerating  
      if (motor2.incrementDirection) {
        motor2.speed = min(motor2.speed + speedIncrement, maxSpeed);

        // When we match the max speed, then start decelerating
        if (motor2.speed == maxSpeed) {
          motor2.incrementDirection = !motor2.incrementDirection;
        }
      // Motor is decelerating   
      } else {
        motor2.speed = max(motor2.speed - speedIncrement, 0);
        
        // When we reach 0, then switch the direction of the motor
        if (motor2.speed == 0) {
          motor2.incrementDirection = !motor2.incrementDirection;
          motor2.motorDirection = !motor2.motorDirection;
          digitalWrite(M2_MOTOR_DIR_PIN, motor2.motorDirection ? HIGH : LOW);
        }
      }

      // Update the speed
      analogWrite(M2_PWM_SPEED_PIN, motor2.speed);
    };

    void stop(void) {
      stopMotors();
    };
};
ControlM2Task controlM2Task;

class PrintTickCountsTask : public Task {
  public:
    void start(void) {
      motor1.tickCount = motor1.totalInterrupts = 0;
      motor1.uTickCount = motor1.vTickCount = motor1.wTickCount = 0;
      motor1.uFaultCount = motor1.vFaultCount = motor1.wFaultCount = 0;
    };

    void update(void) {
      DebugMsgs.debug()
              .print("M1: ")
              .print("U: ").print(motor1.uTickCount)
              .print(" V: ").print(motor1.vTickCount)
              .print(" W: ").print(motor1.wTickCount)
              .print(" tick: ").print(motor1.tickCount)
              .print(" totalInterrupts: ").println(motor1.totalInterrupts);
      DebugMsgs.debug().print("M1: Faults- U: ").print(motor1.uFaultCount).print(" V: ").print(motor1.vFaultCount)
        .print(" W: ").println(motor1.wFaultCount);
      DebugMsgs.debug()
              .print("M2: ")
              .print("U: ").print(motor2.uTickCount)
              .print(" V: ").print(motor2.vTickCount)
              .print(" W: ").print(motor2.wTickCount)
              .print(" tick: ").print(motor2.tickCount)
              .print(" totalInterrupts: ").println(motor2.totalInterrupts);
      DebugMsgs.debug().print("M1: Faults- U: ").print(motor2.uFaultCount).print(" V: ").print(motor2.vFaultCount)
        .print(" W: ").println(motor2.wFaultCount);
    };
};
PrintTickCountsTask printTickCountsTask;

class PrintTickSpeedTask : public Task {
  public:
    void start(void) {
      lastM1TickCount = 0;
      lastM2TickCount = 0;
      lastTickCountTime = 0;      
    };

    void update(void) {
      int32_t curM1TickCount = motor1.tickCount;
      int32_t curM2TickCount = motor2.tickCount;
      int32_t curTickTime = millis();
      
      if (lastTickCountTime != 0) {
        int32_t diffTime = curTickTime - lastTickCountTime;
        int32_t diffM1Ticks = abs(curM1TickCount - lastM1TickCount);
        int32_t diffM2Ticks = abs(curM2TickCount - lastM2TickCount);

        DebugMsgs.debug().print("M1 ticks/second: ").println(diffM1Ticks * (1000/diffTime));
        DebugMsgs.debug().print("M2 ticks/second: ").println(diffM2Ticks * (1000/diffTime));
      }

      lastM1TickCount = curM1TickCount;
      lastM2TickCount = curM2TickCount;
      lastTickCountTime = curTickTime;
    };
    
  private:
    int32_t lastM1TickCount;
    int32_t lastM2TickCount;
    int32_t lastTickCountTime;
};
PrintTickSpeedTask printTickSpeedTask;

BlinkTask idleTask;

void setup() {
  Serial.begin(9600);
  // give serial some time to catch up
  delay(1000);

  DebugMsgs.enableLevel(DEBUG);
  
  // Uncomment for more detailed debug messages
  //DebugMsgs.enableLevel(NOTIFICATION);

  setupPins();

  taskManager.addIdleTask(&idleTask, 100);
  taskManager.addBlinkTask(500);
  taskManager.addTask(&controlM1Task, 500);
  taskManager.addTask(&controlM2Task, 500);
  taskManager.addTask(&printTickCountsTask, 1000);
  //taskManager.addTask(&printTickSpeedTask, 1000);
  taskManager.startMonitoringButton(BUTTON_PIN, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  taskManager.update();
}
