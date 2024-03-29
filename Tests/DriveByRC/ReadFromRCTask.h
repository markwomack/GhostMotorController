//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

#ifndef READFROMRCTASK_H
#define READFROMRCTASK_H

// A task to read the RC signals and convert
// to a value between -1 and 1.
class ReadFromRCTask : public Task {
  public:
    ReadFromRCTask();
    
    void setRCPins(int chan1Pin, int chan2Pin, int chan3Pin);

    void start(void);
    
    // Called periodically to set the desired motor speeds from the
    // the RC signals.
    void update(void);

    double getSignal1Value();
    double getSignal2Value();
    double getSignal3Value();

  private:
    int _chan1Pin;
    int _chan2Pin;
    int _chan3Pin;
    double _signal1Value;
    double _signal2Value;
    double _signal3Value;

    // Reads the current RC signal on the given pin, returns it
    // as a value between minLimit and maxLimit. Returns NO_RC_SIGNAL
    // if there is no RC signal.
    int readChannel(int channelPin, int minLimit, int maxLimit);
};

#endif // READFROMRCTASK_H
