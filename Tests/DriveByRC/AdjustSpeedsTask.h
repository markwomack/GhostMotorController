//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

#ifndef ADJUSTSPEEDSTASK_H
#define ADJUSTSPEEDSTASK_H

#include <MotorAndEncoderManager.h>
#include <MotorController.h>

#include "ReadFromRCTask.h"

// Task to allow the motor controller to periodically adjust
// the motor speeds to match the current desired speeds
class AdjustSpeedsTask : public Task {
  public:
    AdjustSpeedsTask();
    
    void useMotorController(bool useMotorController);
    void setMotorManagerAndController(MotorAndEncoderManager* motorManager, MotorController* motorController);
    void setReadFromRCTask(ReadFromRCTask* readFromRCTask);
    void start(void);
    
    void stop(void);

    // Performs a controlled stop on both motors
    void performControlledStop(void);
    
    // Called periodically to set the desired motor speeds from the
    // the RC signals.
    void update(void);

  private:
    bool _useMotorController;
    bool _stopped;
    MotorAndEncoderManager* _motorManager;
    MotorController* _motorController;
    ReadFromRCTask* _readFromRCTask;
    double _targetM0Speed;
    double _targetM1Speed;
    double _linearVelocity;
    double _angularVelocity;
};

#endif // ADJUSTSPEEDSTASK_H
