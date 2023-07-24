//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

#include <Arduino.h>

#include <DebugMsgs.h>
#include <TaskManager.h>

#include "pin_assignments.h"
#include "globals.h"
#include "ReadFromRCTask.h"

ReadFromRCTask::ReadFromRCTask() {
  // Nothing here!
}
    
void ReadFromRCTask::setRCPins(int chan1Pin, int chan2Pin, int chan3Pin) {
  _chan1Pin = chan1Pin;
  _chan2Pin = chan2Pin;
  _chan3Pin = chan3Pin;
}

// Reads the current RC signal on the given pin, returns it
// as a value between minLimit and maxLimit. Returns NO_RC_SIGNAL
// if there is no RC signal.
int ReadFromRCTask::readChannel(int channelPin, int minLimit, int maxLimit) {
  int ch = pulseIn(channelPin, HIGH, 30000);
  if (ch < 100) return NO_RC_SIGNAL;
  int value = map(ch, 1020, 1980, minLimit, maxLimit);
  return min(max(minLimit, value), maxLimit);
}

void ReadFromRCTask::start(void) {
  _signal1Value = 0;
  _signal2Value = 0;
  _signal3Value = 0;
}

// Called periodically to set the desired motor speeds from the
// the RC signals.
void ReadFromRCTask::update(void) {

  // Read the current RC signals on both channels
  int ch1Val = _chan1Pin != -1 ? readChannel(_chan1Pin, -100, 100) : 0;
  int ch2Val = _chan2Pin != -1 ? readChannel(_chan2Pin, -100, 100) : 0;
  int ch3Val = _chan3Pin != -1 ? readChannel(_chan3Pin, -100, 100) : 0;

  //DebugMsgs.debug().printfln("RC Channels: %d : %d : %d", ch1Val, ch2Val, ch3Val);

  // If either channel has been lost, then stop the robot, stop all the tasks.
  if ((ch1Val == NO_RC_SIGNAL) || (ch2Val == NO_RC_SIGNAL) || (ch3Val == NO_RC_SIGNAL)) {
    DebugMsgs.debug().println("RC channel lost, stopping...");
    taskManager.stop();
    return;
  }

  if (abs(ch1Val) <= 5) {
    ch1Val = 0;
  }
  _signal1Value = ((double)ch1Val/100.0);

  if (abs(ch2Val) <= 5) {
    ch2Val = 0;
  }
  _signal2Value = ((double)ch2Val/100.0);

  if (abs(ch3Val) <= 5) {
    ch3Val = 0;
  }
  _signal3Value = ((double)ch3Val/100.0);
}

double ReadFromRCTask::getSignal1Value() {
  return _signal1Value;
}

double ReadFromRCTask::getSignal2Value() {
  return _signal2Value;
}

double ReadFromRCTask::getSignal3Value() {
  return _signal3Value;
}
