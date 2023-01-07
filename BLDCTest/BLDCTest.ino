#include <Arduino.h>
#include <inttypes.h>
#include <DebugMsgs.h>
#include <TaskManager.h>

//// pin assignments
// Teensy 3.5/4.0/4.1 - These assignments work across all versions
const uint8_t PWM_SPEED_PIN(2);
const uint8_t BRAKE_PIN(3);
const uint8_t MOTOR_DIR_PIN(4);
const uint8_t LED_BUILTIN_PIN(13);
const uint8_t U_ENCODER_SIGNAL_PIN(14); // yellow - Ha
const uint8_t V_ENCODER_SIGNAL_PIN(15); // blue   - Hb
const uint8_t W_ENCODER_SIGNAL_PIN(16); // green  - Hc
const uint8_t BUTTON_PIN(23);

TaskManager taskManager;

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

struct LEDContext {
  int ledState;
};
LEDContext ledContext;

struct MotorContext {
  bool incrementDirection;
  bool motorDirection;
  int speed;
};
MotorContext motorContext;
 
void setup() {
  Serial.begin(9600);
  // give serial some time to catch up
  delay(1000);

  DebugMsgs.enableLevel(DEBUG);
    
  // Uncomment for more detailed debug messages
  //DebugMsgs.enableLevel(NOTIFICATION);
  
  // Setup the task manager with the button pin and callbacks
  taskManager.setup(BUTTON_PIN, HIGH, setupCallback, startCallback, stopCallback, idleCallback);
}

void loop() {
  // put your main code here, to run repeatedly:
  taskManager.loop();
}

/**
  Interrupt handler for the U encoder signal.
**/
void countUTick() {
  // Read the current signal value
  int val = digitalRead(U_ENCODER_SIGNAL_PIN);

  // Check for an encoder fault  
  bool encoderFault = (lastEncoder == 'U') || // Last encoder was this one
                      (lastEncoder == 'W' && motorContext.motorDirection) || // Last encoder was W, but going forward
                      (lastEncoder == 'V' && !motorContext.motorDirection);  // Last encoder was V, but going reverse
  
  if (encoderFault) {
    uFaultCount++;
    // DebugMsgs.notification().print("U ").print(lastUVal).print(' ').print(val).print(' ')
    //   .print(lastEncoder).println(encoderFault ? " *" : "");
  }

  // If there is a fault, don't record a tick.
  if (encoderFault) { return; }
  
  int increment = (lastEncoder == 'W') ? 1 : -1;
  uCount += increment;
  tickCount += increment;
  lastEncoder = 'U';
  lastUVal = val;
}

void countVTick() {
  // Read the current signal value
  int val = digitalRead(V_ENCODER_SIGNAL_PIN);

  // Check for an encoder fault  
  bool encoderFault = (lastEncoder == 'V') || // Last encoder was this one
                      (lastEncoder == 'U' && motorContext.motorDirection) || // Last encoder was U, but going forward
                      (lastEncoder == 'W' && !motorContext.motorDirection);  // Last encoder was W, but going reverse
  
  if (encoderFault) {
    vFaultCount++;
    // DebugMsgs.notification().print("V ").print(lastVVal).print(' ').print(val).print(' ')
    //   .print(lastEncoder).println(encoderFault ? " *" : "");
  }

  // If there is a fault, don't record a tick.
  if (encoderFault) { return; }
  
  int increment = (lastEncoder == 'U') ? 1 : -1;
  vCount += increment;
  tickCount += increment;
  lastEncoder = 'V';
  lastVVal = val;
}

void countWTick() {
  // Read the current signal value
  int val = digitalRead(W_ENCODER_SIGNAL_PIN);

  // Check for an encoder fault  
  bool encoderFault = (lastEncoder == 'W') || // Last encoder was this one
                      (lastEncoder == 'V' && motorContext.motorDirection) || // Last encoder was V, but going forward
                      (lastEncoder == 'U' && !motorContext.motorDirection);  // Last encoder was U, but going reverse

  if (encoderFault) {
    wFaultCount++;
    // DebugMsgs.notification().print("W ").print(lastWVal).print(' ').print(val).print(' ')
    //   .print(lastEncoder).println(encoderFault ? " *" : "");
  }

  // If there is a fault, don't record a tick.
  if (encoderFault) { return; }
  
  int increment = (lastEncoder == 'V') ? 1 : -1;         
  wCount += increment;
  tickCount += increment;
  lastEncoder = 'W';
  lastWVal = val;
}

void setupCallback(void) {
  tickCount = 0;
  uCount = 0;
  uFaultCount = 0;
  vCount = 0;
  vFaultCount = 0;
  wCount = 0;
  vFaultCount = 0;

  // Initialize our contexts

  pinMode(LED_BUILTIN_PIN, OUTPUT);
  digitalWrite(LED_BUILTIN_PIN, LOW);
  ledContext.ledState = LOW;

  pinMode(PWM_SPEED_PIN, OUTPUT);
  analogWrite(PWM_SPEED_PIN, 0);

  // TODO: put these under some kind of IFDEF
//  // Teensy 4.1
//  // values will be 0-8191
  analogWriteResolution(13);
  analogWriteFrequency(PWM_SPEED_PIN, 18310.55);

  //Teensy 3.5
  // values will be 0-4095
//  analogWriteResolution(12);
//  analogWriteFrequency(PWM_SPEED_PIN, 14648.437);
  
  pinMode(BRAKE_PIN, OUTPUT);
  digitalWrite(BRAKE_PIN, HIGH);

  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN, LOW);
  
  pinMode(U_ENCODER_SIGNAL_PIN, INPUT);
  lastUVal = digitalRead(U_ENCODER_SIGNAL_PIN);
  attachInterrupt(U_ENCODER_SIGNAL_PIN, countUTick, CHANGE);
  
  pinMode(V_ENCODER_SIGNAL_PIN, INPUT);
  lastVVal = digitalRead(V_ENCODER_SIGNAL_PIN);
  attachInterrupt(V_ENCODER_SIGNAL_PIN, countVTick, CHANGE);
  
  pinMode(W_ENCODER_SIGNAL_PIN, INPUT);
  lastWVal = digitalRead(W_ENCODER_SIGNAL_PIN);
  attachInterrupt(W_ENCODER_SIGNAL_PIN, countWTick, CHANGE);
}

void startCallback(void) {
  tickCount = 0;
  uCount = 0;
  uFaultCount = 0;
  vCount = 0;
  vFaultCount = 0;
  wCount = 0;
  vFaultCount = 0;
  motorContext.incrementDirection = true; // false == slow it down, true == speed it up
  motorContext.motorDirection = false;  // false == forward, true == reverse
  motorContext.speed = 0;

  // Set up the speed (0) and set the direction
  analogWrite(PWM_SPEED_PIN, motorContext.speed);
  digitalWrite(MOTOR_DIR_PIN, motorContext.motorDirection ? HIGH : LOW);
  
  // Allow the motor to spin
  digitalWrite(BRAKE_PIN, LOW);
  
  // Set up the best effort callbacks
  taskManager.callbackEvery(500, ledCallback, (void*)&ledContext);
  taskManager.callbackEvery(1000, printCounts, (void*)0);
  taskManager.callbackEvery(500, setSpeed, (void*)&motorContext);
}

void stopCallback(void) {
  DebugMsgs.debug().println("Stopping motor...");
  // Spin down the motor in controlled manner
  while (motorContext.speed > 0) {
    motorContext.speed = max(motorContext.speed - 500, 0);
    analogWrite(PWM_SPEED_PIN, motorContext.speed);
    delay(500);
  }
  // Don't allow the motor to spin
  digitalWrite(BRAKE_PIN, HIGH);
  DebugMsgs.debug().println("...motor stopped.");
}

// TODO: are these processor model specific?
const int speedIncrement(100); //200
const int maxSpeed(5000); //5000

void setSpeed(void* context) {
  MotorContext* motorContext = (MotorContext*)context;
  // Motor is accelerating  
  if (motorContext->incrementDirection) {
    motorContext->speed = min(motorContext->speed + speedIncrement, maxSpeed);

    // When we match the max speed, then start decelerating
    if (motorContext->speed == maxSpeed) {
      motorContext->incrementDirection = !motorContext->incrementDirection;
    }
  // Motor is decelerating   
  } else {
    motorContext->speed = max(motorContext->speed - speedIncrement, 0);
    
    // When we reach 0, then switch the direction of the motor
    if (motorContext->speed == 0) {
      motorContext->incrementDirection = !motorContext->incrementDirection;
      motorContext->motorDirection = !motorContext->motorDirection;
      digitalWrite(MOTOR_DIR_PIN, motorContext->motorDirection ? HIGH : LOW);
    }
  }
  //DebugMsgs.debug().print("Speed: ").println(motorContext->speed);
  // Update the speed
  analogWrite(PWM_SPEED_PIN, motorContext->speed);
}

void printCounts(void* context) {
  DebugMsgs.debug()
           .print("U: ").print(uCount)
           .print(" V: ").print(vCount)
           .print(" W: ").print(wCount)
           .print(" tick: ").println(tickCount);
  if (uFaultCount != 0 || vFaultCount != 0 || wFaultCount != 0) {
    DebugMsgs.debug().print("Faults- U: ").print(uFaultCount).print(" V: ").print(vFaultCount)
      .print(" W: ").println(wFaultCount);
    uFaultCount = vFaultCount = wFaultCount = 0;
  }
}

// This method is registered above with the task manager.
// You should see the built-in led flash on and off at
// the frequency to callback was registered with.
void ledCallback(void* context) {
  // change led state
  char newLedState = ((LEDContext*)context)->ledState == LOW ? HIGH : LOW;
  digitalWrite(LED_BUILTIN_PIN, newLedState);

  // store state for next callback
  ((LEDContext*)context)->ledState = newLedState;
}

void idleCallback(void) {
  // change led state
  char newLedState = ledContext.ledState == LOW ? HIGH : LOW;
  digitalWrite(LED_BUILTIN_PIN, newLedState);

  // store state for next callback
  ledContext.ledState = newLedState;
}
