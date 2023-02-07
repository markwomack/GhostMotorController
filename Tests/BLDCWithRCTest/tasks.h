//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

#ifndef TASKS_H
#define TASKS_H

// Arduino includes
#include <Arduino.h>

// My includes
#include <Task.h>

// Local includes
#include "globals.h"

class ReadRCChannelTask : public Task {
  public:
    ReadRCChannelTask() {};
    void setChannelPin(int8_t channelPin);
    int getValue(void);
    void start(void);
    void update(void);

  private:
    uint8_t _channelPin;
    int _value;
};

class AdjustMotorSpeedsTask : public Task {
  public:
    AdjustMotorSpeedsTask() {};
    void setRCChannels(ReadRCChannelTask* rcChannel1, ReadRCChannelTask* rcChannel2);
    void setMotor1Info(String label, MotorEncoderInfo* motorEncoderInfo, uint8_t speedPin, uint8_t dirPin, uint8_t brakePin);
    void setMotor2Info(String label, MotorEncoderInfo* motorEncoderInfo, uint8_t speedPin, uint8_t dirPin, uint8_t brakePin);
    void start(void);
    void update(void);
    void stop(void);

  private:
    ReadRCChannelTask* _rcChannel1;
    ReadRCChannelTask* _rcChannel2;
    MotorInfo _motor1;
    MotorInfo _motor2;
};

#endif // TASKS_H
