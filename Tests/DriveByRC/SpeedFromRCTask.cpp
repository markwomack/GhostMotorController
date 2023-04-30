//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

#include <Arduino.h>

#include <DebugMsgs.h>
#include <TaskManager.h>

#include "pin_assignments.h"
#include "globals.h"
#include "SpeedFromRCTask.h"

SpeedFromRCTask::SpeedFromRCTask() {
  // Nothing here!
}
    
void SpeedFromRCTask::setRCPins(uint8_t chan1Pin, uint8_t chan2Pin) {
  _chan1Pin = chan1Pin;
  _chan2Pin = chan2Pin;
}

// Reads the current RC signal on the given pin, returns it
// as a value between minLimit and maxLimit. Returns NO_RC_SIGNAL
// if there is no RC signal.
int SpeedFromRCTask::readChannel(int channelPin, int minLimit, int maxLimit) {
  int ch = pulseIn(channelPin, HIGH, 30000);
  if (ch < 100) return NO_RC_SIGNAL;
  int value = map(ch, 1020, 1980, minLimit, maxLimit);
  return min(max(minLimit, value), maxLimit);
}

void SpeedFromRCTask::start(void) {
  _m0Speed = 0;
  _m1Speed = 0;
  _linearVelocity = 0;
  _angularVelocity = 0;

}

// Called periodically to set the desired motor speeds from the
// the RC signals.
void SpeedFromRCTask::update(void) {

  // Read the current RC signals on both channels
  int ch1Val = readChannel(_chan1Pin, -100, 100);
  int ch2Val = readChannel(_chan2Pin, -100, 100);

  //DebugMsgs.debug().print("RC Channels: ").print(ch1Val).print(" : ").println(ch2Val);

  // If either channel has been lost, then stop the robot, stop all the tasks.
  if (ch1Val == NO_RC_SIGNAL || ch2Val == NO_RC_SIGNAL) {
    DebugMsgs.debug().println("RC channel lost, stopping...");
    taskManager.stop();
    return;
  }

  // maintain a value of zero if close to zero
  // avoids shimming
  if (abs(ch1Val) <= 5) {
    ch1Val = 0;
  }
  if (abs(ch2Val) <= 5) {
    ch2Val = 0;
  }

  // Channel 2 is for linear (forward/reverse) velocity
  _linearVelocity = ((double)ch2Val/100.0) * MAX_LINEAR_VELOCITY;

  // Channel 1 is for angular (left/right) velocity
  _angularVelocity = ((double)ch1Val/100.0) * MAX_ANGULAR_VELOCITY;

  // Calculate the motor speeds to match
  _m0Speed = _linearVelocity - _angularVelocity;
  _m1Speed = _linearVelocity + _angularVelocity;
  
  //DebugMsgs.debug().print("Setting speeds: ").print(_m0Speed).print(" : ").println(_m1Speed);
}

double SpeedFromRCTask::getM0Speed() {
  return _m0Speed;
}

double SpeedFromRCTask::getM1Speed() {
  return _m1Speed;
}
