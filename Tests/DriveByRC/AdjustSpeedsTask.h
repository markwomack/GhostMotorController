//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

#ifndef ADJUSTSPEEDSTASK_H
#define ADJUSTSPEEDSTASK_H

#include <MotorAndEncoderManager.h>
#include <MotorController.h>

#include "SpeedFromRCTask.h"

// Task to allow the motor controller to periodically adjust
// the motor speeds to match the current desired speeds
class AdjustSpeedsTask : public Task {
  public:
    AdjustSpeedsTask();
    
    void useMotorController(bool useMotorController);
    void setMotorManagerAndController(MotorAndEncoderManager* motorManager, MotorController* motorController);
    void setSpeedFromRCTask(SpeedFromRCTask* speedFromRCTask);
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
    SpeedFromRCTask* _speedFromRCTask;
    double _targetM0Speed;
    double _targetM1Speed;
};

#endif // ADJUSTSPEEDSTASK_H
