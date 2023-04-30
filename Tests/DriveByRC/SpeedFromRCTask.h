//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

#ifndef SPEEDFROMRCTASK_H
#define SPEEDFROMRCTASK_H

// A task to get the desired speeds from two RC
// input signals.
class SpeedFromRCTask : public Task {
  public:
    SpeedFromRCTask();
    
    void setRCPins(uint8_t chan1Pin, uint8_t chan2Pin);

    // Reads the current RC signal on the given pin, returns it
    // as a value between minLimit and maxLimit. Returns NO_RC_SIGNAL
    // if there is no RC signal.
    int readChannel(int channelPin, int minLimit, int maxLimit);

    void start(void);
    
    // Called periodically to set the desired motor speeds from the
    // the RC signals.
    void update(void);

    double getM0Speed();
    double getM1Speed();

  private:
    uint8_t _chan1Pin;
    uint8_t _chan2Pin;
    double _m0Speed;
    double _m1Speed;
    double _linearVelocity;
    double _angularVelocity;
};

#endif // SPEEDFROMRCTASK_H
