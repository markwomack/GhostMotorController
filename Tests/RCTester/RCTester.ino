//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

// This is a simple program to test the RC inputs
// to the motor controller hardware.

// Arduino includes
#include <Arduino.h>

// Third party includes
#include <DebugMsgs.h>   // https://github.com/markwomack/ArduinoLogging
#include <TaskManager.h> // https://github.com/markwomack/TaskManager

const uint8_t RC_CH1_PIN(23);
const uint8_t RC_CH2_PIN(22);

class ReadRCTask : public Task {
  public:
    void setRCChannelPin(uint8_t pin) {
      _pin = pin;
    }

    void setup(void) {
      pinMode(_pin, INPUT);
    };

    void update(void) {
      int val = pulseIn(_pin, HIGH, 30000);
      DebugMsgs.debug().print("Pin ").print(_pin).print(": ").println(val);
    };

  private:
    uint8_t _pin;
};
ReadRCTask readRCTask1;
ReadRCTask readRCTask2;

void setup() {
  Serial.begin(9600);

  DebugMsgs.enableLevel(DEBUG);
  
  readRCTask1.setRCChannelPin(RC_CH1_PIN);
  readRCTask2.setRCChannelPin(RC_CH2_PIN);
  
  taskManager.addBlinkTask(500);
  taskManager.addTask(&readRCTask1, 100);
  taskManager.addTask(&readRCTask2, 100);
  taskManager.start();
}

void loop() {
  taskManager.update();
}
