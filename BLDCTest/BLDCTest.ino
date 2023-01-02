#include <Arduino.h>
#include <inttypes.h>
#include <DebugMsgs.h>
#include <TaskManager.h>

//// pin assignments
//// Teensy 4.1
const uint8_t PWM_SPEED_PIN(1);
const uint8_t BRAKE_PIN(2);
const uint8_t MOTOR_DIR_PIN(3);
const uint8_t LED_BUILTIN_PIN(13);
const uint8_t U_ENCODER_SIGNAL_PIN(14); // yellow - Ha
const uint8_t V_ENCODER_SIGNAL_PIN(15); // green  - Hb
const uint8_t W_ENCODER_SIGNAL_PIN(16); // blue   - Hc
const uint8_t BUTTON_PIN(23);

// Teensy 3.5
//const uint8_t PWM_SPEED_PIN(2);
//const uint8_t BRAKE_PIN(3);
//const uint8_t MOTOR_DIR_PIN(4);
//const uint8_t LED_BUILTIN_PIN(13);
//const uint8_t U_ENCODER_SIGNAL_PIN(14); // yellow - Ha
//const uint8_t V_ENCODER_SIGNAL_PIN(15); // blue   - Hb
//const uint8_t W_ENCODER_SIGNAL_PIN(16); // green  - Hc
//const uint8_t BUTTON_PIN(23);

TaskManager taskManager;

volatile int32_t tickCount;
volatile int32_t uCount;
volatile int lastUVal;
volatile int32_t vCount;
volatile int lastVVal;
volatile int32_t wCount;
volatile int lastWVal;
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

void countUTick() {
  int val = digitalRead(U_ENCODER_SIGNAL_PIN);
  if (val == lastUVal) { return; }
  if (lastEncoder == 'U') { return; }
  DebugMsgs.notification().print("U ").print(lastUVal).print(' ').print(val).print(' ').print(lastEncoder).println(lastEncoder != 'W' ? " *" : "");
  uCount += lastEncoder == 'W' ? 1 : -1;
  tickCount += lastEncoder == 'W' ? 1 : -1;
  lastEncoder = 'U';
  lastUVal = val;
}

void countVTick() {
  int val = digitalRead(V_ENCODER_SIGNAL_PIN);
  if (val == lastVVal) { return; }
  if (lastEncoder == 'V') { return; }
  DebugMsgs.notification().print("V ").print(lastVVal).print(' ').print(val).print(' ').print(lastEncoder).println(lastEncoder != 'U' ? " *" : "");
  vCount += lastEncoder == 'U' ? 1 : -1;
  tickCount += lastEncoder == 'U' ? 1 : -1;
  lastEncoder = 'V';
  lastVVal = val;
}

void countWTick() {
  int val = digitalRead(W_ENCODER_SIGNAL_PIN);
  if (val == lastWVal) { return; }
  if (lastEncoder == 'W') { return; }
  DebugMsgs.notification().print("W ").print(lastWVal).print(' ').print(val).print(' ').print(lastEncoder).println(lastEncoder != 'V' ? " *" : "");
  wCount += lastEncoder == 'V' ? 1 : -1;
  tickCount += lastEncoder == 'V' ? 1 : -1;
  lastEncoder = 'W';
  lastWVal = val;
}


void setupCallback(void) {
  tickCount = 0;
  uCount = 0;
  vCount = 0;
  wCount = 0;

  // Initialize our contexts

  pinMode(LED_BUILTIN_PIN, OUTPUT);
  digitalWrite(LED_BUILTIN_PIN, LOW);
  ledContext.ledState = LOW;

  pinMode(PWM_SPEED_PIN, OUTPUT);
  analogWrite(PWM_SPEED_PIN, 0);

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
  vCount = 0;
  wCount = 0;
  motorContext.incrementDirection = true;
  motorContext.motorDirection = false;
  motorContext.speed = 0;

  analogWrite(PWM_SPEED_PIN, motorContext.speed);
  digitalWrite(MOTOR_DIR_PIN, motorContext.motorDirection ? HIGH : LOW);
  
  // Allow the motor to spin
  digitalWrite(BRAKE_PIN, LOW);
  
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

const int speedIncrement(100); //200
const int maxSpeed(5000); //5000

void setSpeed(void* context) {
  MotorContext* motorContext = (MotorContext*)context;
  if (motorContext->incrementDirection) {
    motorContext->speed = min(motorContext->speed + speedIncrement, maxSpeed);
    if (motorContext->speed == maxSpeed) {
      motorContext->incrementDirection = !motorContext->incrementDirection;
    }
  } else {
    motorContext->speed = max(motorContext->speed - speedIncrement, 0);
    if (motorContext->speed == 0) {
      motorContext->incrementDirection = !motorContext->incrementDirection;

      // change direction of the motor
      motorContext->motorDirection = !motorContext->motorDirection;
      digitalWrite(MOTOR_DIR_PIN, motorContext->motorDirection ? HIGH : LOW);
    }
  }
  DebugMsgs.debug().print("Speed: ").println(motorContext->speed);
  analogWrite(PWM_SPEED_PIN, motorContext->speed);
}

void printCounts(void* context) {
  DebugMsgs.notification().print("W state: ").println(digitalRead(W_ENCODER_SIGNAL_PIN));
  DebugMsgs.notification().print("U state: ").println(digitalRead(U_ENCODER_SIGNAL_PIN));
  DebugMsgs.notification().print("V state: ").println(digitalRead(V_ENCODER_SIGNAL_PIN));
  DebugMsgs.debug()
           .print("U: ").print(uCount)
           .print(" V: ").print(vCount)
           .print(" W: ").print(wCount)
           .print(" tick: ").println(tickCount);
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
