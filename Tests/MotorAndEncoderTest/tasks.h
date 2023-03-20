//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

#ifndef TASKS_H
#define TASKS_H

// Arduino includes
#include <inttypes.h>

// My includes
#include <Task.h>

// Local includes
#include "globals.h"

// Task to control a motor
class ControlMotorTask : public Task {
  public:
    void setup(MotorEncoderInfo* motor, String label, uint8_t pwmSpeedPin, uint8_t motorDirPin, uint8_t brakePin);
    void start(void);
    void update(void);
    void stop(void);
    
  protected:
    MotorEncoderInfo* _motor;
    String _label;    
    uint8_t _pwmSpeedPin;
    uint8_t _brakePin;
    uint8_t _motorDirPin;
};

// A task to count rotations and then stop when count detected.
class CountRotationsTask :public Task {
  public:
    void setRotations(int numRotations);
    void update(void);

  private:
    int _numRotations;
};

// A task to print the tick and fault counts.
class PrintMotorTickCountsTask : public Task {
  public:
    void setup(MotorEncoderInfo* motor, String label);
    void update(void);

    protected:
      MotorEncoderInfo* _motor;
      String _label;
};

// A task to print the motor speeds
class PrintTickSpeedTask : public Task {
  public:
    void start(void);
    void update(void);
    
  private:
    int32_t lastM0TickCount;
    int32_t lastM1TickCount;
    int32_t lastTickCountTime;
};

#endif // TASK_H