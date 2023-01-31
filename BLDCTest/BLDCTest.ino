#include <Arduino.h>
#include <inttypes.h>
#include <DebugMsgs.h>
#include <TaskManager.h>
#include <Task.h>

//// pin assignments
// Teensy 3.5/4.0/4.1 - These assignments work across all versions
const uint8_t PWM_SPEED_1_PIN(2);
const uint8_t BRAKE_1_PIN(3);
const uint8_t MOTOR_DIR_1_PIN(4);
const uint8_t PWM_SPEED_2_PIN(10);
const uint8_t BRAKE_2_PIN(11);
const uint8_t MOTOR_DIR_2_PIN(12);
const uint8_t LED_BUILTIN_PIN(13);
const uint8_t U1_ENCODER_SIGNAL_PIN(14); // yellow - Ha
const uint8_t V1_ENCODER_SIGNAL_PIN(15); // blue   - Hb
const uint8_t W1_ENCODER_SIGNAL_PIN(16); // green  - Hc
const uint8_t U2_ENCODER_SIGNAL_PIN(17); // yellow - Ha
const uint8_t V2_ENCODER_SIGNAL_PIN(18); // blue   - Hb
const uint8_t W2_ENCODER_SIGNAL_PIN(19); // green  - Hc
const uint8_t BUTTON_PIN(23);

struct MotorInfo {
  volatile int32_t totalInterrupts;
  volatile int32_t tickCount;
  volatile int32_t uCount;
  volatile int lastUVal;
  volatile int uFaultCount;
  volatile int32_t vCount;
  volatile int lastVVal;
  volatile int vFaultCount;
  volatile int32_t wCount;
  volatile int lastWVal;
  volatile int wFaultCount;
  volatile char lastEncoder;
  volatile bool motorDirection;
};
MotorInfo motor1;
MotorInfo motor2;

// TODO: are these processor model specific?
const int speedIncrement(200); //200
const int maxSpeed(5000); //5000

class ControlMotorTask : public Task {
  public:
    ControlMotorTask() {};

    void setup(void) {
      pinMode(PWM_SPEED_1_PIN, OUTPUT);
      analogWrite(PWM_SPEED_1_PIN, 0);
      pinMode(PWM_SPEED_2_PIN, OUTPUT);
      analogWrite(PWM_SPEED_2_PIN, 0);

      // TODO: put these under some kind of IFDEF
    //  // Teensy 4.1
    //  // values will be 0-8191
      analogWriteResolution(13);
      analogWriteFrequency(PWM_SPEED_1_PIN, 18310.55);
      analogWriteFrequency(PWM_SPEED_2_PIN, 18310.55);

      //Teensy 3.5
      // values will be 0-4095
    //  analogWriteResolution(12);
    //  analogWriteFrequency(PWM_SPEED_1_PIN, 14648.437);
    //  analogWriteFrequency(PWM_SPEED_2_PIN, 14648.437);
      
      pinMode(BRAKE_1_PIN, OUTPUT);
      digitalWrite(BRAKE_1_PIN, HIGH);
      pinMode(BRAKE_2_PIN, OUTPUT);
      digitalWrite(BRAKE_2_PIN, HIGH);

      pinMode(MOTOR_DIR_1_PIN, OUTPUT);
      digitalWrite(MOTOR_DIR_1_PIN, LOW);
      pinMode(MOTOR_DIR_2_PIN, OUTPUT);
      digitalWrite(MOTOR_DIR_2_PIN, LOW);
    };

    void start(void) {
      incrementDirection = true; // false == slow it down, true == speed it up
      speed = 0;
      motor1.motorDirection = false;  // false == forward, true == reverse
      motor2.motorDirection = false;  // false == forward, true == reverse

      // Set up the speed (0) and set the direction
      analogWrite(PWM_SPEED_1_PIN, speed);
      digitalWrite(MOTOR_DIR_1_PIN, motor1.motorDirection ? HIGH : LOW);
      analogWrite(PWM_SPEED_2_PIN, speed);
      digitalWrite(MOTOR_DIR_2_PIN, motor2.motorDirection ? HIGH : LOW);
      
      // Allow the motor to spin
      digitalWrite(BRAKE_1_PIN, LOW);
      digitalWrite(BRAKE_2_PIN, LOW);
    };

    void update(void) {
      // Motor is accelerating  
      if (incrementDirection) {
        speed = min(speed + speedIncrement, maxSpeed);

        // When we match the max speed, then start decelerating
        if (speed == maxSpeed) {
          incrementDirection = !incrementDirection;
        }
      // Motor is decelerating   
      } else {
        speed = max(speed - speedIncrement, 0);
        
        // When we reach 0, then switch the direction of the motor
        if (speed == 0) {
          incrementDirection = !incrementDirection;
          motor1.motorDirection = !motor1.motorDirection;
          motor2.motorDirection = !motor2.motorDirection;
          digitalWrite(MOTOR_DIR_1_PIN, motor1.motorDirection ? HIGH : LOW);
          digitalWrite(MOTOR_DIR_2_PIN, motor2.motorDirection ? HIGH : LOW);
        }
      }
      //DebugMsgs.debug().print("Speed: ").println(speed);
      // Update the speed
      analogWrite(PWM_SPEED_1_PIN, speed);
      analogWrite(PWM_SPEED_2_PIN, speed);
    };

    void stop(void) {
      DebugMsgs.debug().println("Stopping motor...");
      // Spin down the motor in controlled manner
      while (speed > 0) {
        speed = max(speed - 500, 0);
        analogWrite(PWM_SPEED_1_PIN, speed);
        analogWrite(PWM_SPEED_2_PIN, speed);
        delay(500);
      }
      // Don't allow the motor to spin
      digitalWrite(BRAKE_1_PIN, HIGH);
      digitalWrite(BRAKE_2_PIN, HIGH);
      DebugMsgs.debug().println("...motor stopped.");
    };
    
  protected:
    bool incrementDirection;
    int speed;
};
ControlMotorTask controlMotorTask;

class PrintTickCountsTask : public Task {
  public:
    void start(void) {
      motor1.tickCount = motor1.totalInterrupts = 0;
      motor1.uCount = motor1.vCount = motor1.wCount = 0;
      motor1.uFaultCount = motor1.vFaultCount = motor1.wFaultCount = 0;
    };

    void update(void) {
      DebugMsgs.debug()
              .print("U: ").print(motor1.uCount)
              .print(" V: ").print(motor1.vCount)
              .print(" W: ").print(motor1.wCount)
              .print(" tick: ").print(motor1.tickCount)
              .print(" totalInterrupts: ").println(motor1.totalInterrupts);
      DebugMsgs.debug().print("Faults- U: ").print(motor1.uFaultCount).print(" V: ").print(motor1.vFaultCount)
        .print(" W: ").println(motor1.wFaultCount);
    };
};
PrintTickCountsTask printTickCountsTask;

class PrintTickSpeedTask : public Task {
  public:
    void start(void) {
      lastTickCount = 0;
      lastTickCountTime = 0;      
    };

    void update(void) {
      int32_t curTickCount = motor1.tickCount;
      int32_t curTickTime = millis();
      
      if (lastTickCountTime != 0) {
        int32_t diffTime = curTickTime - lastTickCountTime;
        int32_t diffTicks = abs(curTickCount - lastTickCount);

        DebugMsgs.debug().print("ticks/second: ").println(diffTicks * (1000/diffTime));
      }

      lastTickCount = curTickCount;
      lastTickCountTime = curTickTime;
    };
    
  private:
    int32_t lastTickCount;
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
  DebugMsgs.enableLevel(NOTIFICATION);
  
  pinMode(U1_ENCODER_SIGNAL_PIN, INPUT_PULLDOWN);
  motor1.lastUVal = digitalRead(U1_ENCODER_SIGNAL_PIN);
  attachInterrupt(U1_ENCODER_SIGNAL_PIN, countUTick, CHANGE);
  
  pinMode(V1_ENCODER_SIGNAL_PIN, INPUT_PULLDOWN);
  motor1.lastVVal = digitalRead(V1_ENCODER_SIGNAL_PIN);
  attachInterrupt(V1_ENCODER_SIGNAL_PIN, countVTick, CHANGE);
  
  pinMode(W1_ENCODER_SIGNAL_PIN, INPUT_PULLDOWN);
  motor1.lastWVal = digitalRead(W1_ENCODER_SIGNAL_PIN);
  attachInterrupt(W1_ENCODER_SIGNAL_PIN, countWTick, CHANGE);
  
  taskManager.addIdleTask(&idleTask, 100);
  taskManager.addBlinkTask(500);
  taskManager.addTask(&controlMotorTask, 500);
  taskManager.addTask(&printTickCountsTask, 1000);
  //taskManager.addTask(&printTickSpeedTask, 1000);
  taskManager.startMonitoringButton(BUTTON_PIN, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  taskManager.update();
}

/**
  Interrupt handler for the U encoder signal.
**/
void countUTick() {
  // Read the current signal value
  int val = digitalRead(U1_ENCODER_SIGNAL_PIN);
  
  motor1.totalInterrupts++;

  // Check for an encoder fault  
  bool encoderFault = (motor1.lastEncoder == 'U') || // Last encoder was this one
                      (motor1.lastEncoder == 'W' && motor1.motorDirection) || // Last encoder was W, but going forward
                      (motor1.lastEncoder == 'V' && !motor1.motorDirection);  // Last encoder was V, but going reverse
  
  if (encoderFault) {
    motor1.uFaultCount++;
  }

  // DebugMsgs.notification().print("U ").print(motor1.lastUVal).print(' ').print(val).print(' ')
  //   .print(motor1.lastEncoder).println(encoderFault ? " *" : "");
      
  // If there is a fault, don't record a tick.
  if (encoderFault) { return; }
  
  int increment = (motor1.lastEncoder == 'W') ? 1 : -1;
  motor1.uCount += increment;
  motor1.tickCount += increment;
  motor1.lastEncoder = 'U';
  motor1.lastUVal = val;
}

void countVTick() {
  // Read the current signal value
  int val = digitalRead(V1_ENCODER_SIGNAL_PIN);
  
  motor1.totalInterrupts++;

  // Check for an encoder fault  
  bool encoderFault = (motor1.lastEncoder == 'V') || // Last encoder was this one
                      (motor1.lastEncoder == 'U' && motor1.motorDirection) || // Last encoder was U, but going forward
                      (motor1.lastEncoder == 'W' && !motor1.motorDirection);  // Last encoder was W, but going reverse
  
  if (encoderFault) {
    motor1.vFaultCount++;
  }

  // DebugMsgs.notification().print("V ").print(motor1.lastVVal).print(' ').print(val).print(' ')
  //   .print(motor1.lastEncoder).println(encoderFault ? " *" : "");

  // If there is a fault, don't record a tick.
  if (encoderFault) { return; }
  
  int increment = (motor1.lastEncoder == 'U') ? 1 : -1;
  motor1.vCount += increment;
  motor1.tickCount += increment;
  motor1.lastEncoder = 'V';
  motor1.lastVVal = val;
}

void countWTick() {
  // Read the current signal value
  int val = digitalRead(W1_ENCODER_SIGNAL_PIN);

  motor1.totalInterrupts++;
  
  // Check for an encoder fault  
  bool encoderFault = (motor1.lastEncoder == 'W') || // Last encoder was this one
                      (motor1.lastEncoder == 'V' && motor1.motorDirection) || // Last encoder was V, but going forward
                      (motor1.lastEncoder == 'U' && !motor1.motorDirection);  // Last encoder was U, but going reverse

  if (encoderFault) {
    motor1.wFaultCount++;
  }
  
  // DebugMsgs.notification().print("W ").print(motor1.lastWVal).print(' ').print(val).print(' ')
  //   .print(motor1.lastEncoder).println(encoderFault ? " *" : "");

  // If there is a fault, don't record a tick.
  if (encoderFault) { return; }
  
  int increment = (motor1.lastEncoder == 'V') ? 1 : -1;         
  motor1.wCount += increment;
  motor1.tickCount += increment;
  motor1.lastEncoder = 'W';
  motor1.lastWVal = val;
}