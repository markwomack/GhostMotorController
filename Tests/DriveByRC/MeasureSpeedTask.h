
#ifndef MEASURESPEEDTASK_H
#define MEASURESPEEDTASK_H

#include <DebugMsgs.h>
#include <Task.h>
#include <MotorAndEncoderManager.h>

#include "pin_assignments.h"

class MeasureSpeedTask : public Task {
  public:

    void setMotorAndEncoderManager(MotorAndEncoderManager* motorAndEncoderManager) {
      _motorAndEncoderManager = motorAndEncoderManager;
    };

    void start(void) {
      _count = 6;
      
      _m0Speed = 0;
      _m1Speed = 0;
      
      _m0LastEncoderVal = 0;
      _m1LastEncoderVal = 0;

      _m0LastReadTime = millis();
      _m1LastReadTime = millis();

      _motorAndEncoderManager->readAndResetEncoder(M0);
      _motorAndEncoderManager->readAndResetEncoder(M1);
      
      digitalWrite(M0_BRAKE_PIN, LOW);
      digitalWrite(M1_BRAKE_PIN, LOW);
    };
    
    void update(void) {
      uint32_t m0CurReadTime = millis();
      uint32_t m0CurEncoderVal = _motorAndEncoderManager->readEncoder(M0);
      uint32_t m1CurReadTime = millis();
      uint32_t m1CurEncoderVal = _motorAndEncoderManager->readEncoder(M1);

      if (_count % 6 == 0) {
        _count = 1;
        DebugMsgs.debug().printfln("Speeds: %.02f, %.02f; Ticks/sec: %d, %d", _m0Speed, _m1Speed,
          (1000*(m0CurEncoderVal - _m0LastEncoderVal))/(m0CurReadTime - _m0LastReadTime),
          (1000*(m1CurEncoderVal - _m1LastEncoderVal))/(m1CurReadTime - _m1LastReadTime));
        
        _m0Speed = min(_m0Speed + .05, 1.0);
        _m1Speed = min(_m1Speed + .05, 1.0);
        _motorAndEncoderManager->setMotorSpeeds(_m0Speed, _m1Speed);
      } else {
        _count++;
      }

      _m0LastReadTime = m0CurReadTime;
      _m0LastEncoderVal = m0CurEncoderVal;
      _m1LastReadTime = m1CurReadTime;
      _m1LastEncoderVal = m1CurEncoderVal;
    };

    void stop(void) {
      performControlledStop();
      
      digitalWrite(M0_BRAKE_PIN, HIGH);
      digitalWrite(M1_BRAKE_PIN, HIGH);

      DebugMsgs.debug().println("All motors stopped.");
    };

    // Performs a controlled stop on both motors
  void performControlledStop(void) {
    DebugMsgs.debug().println("Performing controlled stop");
  
    uint32_t checkTime = 0;
    while(true) {
      if (millis() >= checkTime) {
        int32_t m0CurEncoderVal = _motorAndEncoderManager->readEncoder(M0);
        int32_t m1CurEncoderVal = _motorAndEncoderManager->readEncoder(M1);
        if (m0CurEncoderVal - _m0LastEncoderVal == 0 && m1CurEncoderVal - _m1LastEncoderVal == 0) {
          break;
        }
        _m0LastEncoderVal = m0CurEncoderVal;
        _m1LastEncoderVal = m1CurEncoderVal;
  
        // Do a controlled slow down to 0 for both motors
        if (_m0Speed > 0) {
          _m0Speed -= 0.1;
          if (_m0Speed < 0) {
            _m0Speed = 0;
          }
        } else if (_m0Speed < 0) {
          _m0Speed += 0.1;
          if (_m0Speed > 0) {
            _m0Speed = 0;
          }
        }
        if (_m1Speed > 0) {
          _m1Speed -= 0.1;
          if (_m1Speed < 0) {
            _m1Speed = 0;
          }
        } else if (_m1Speed < 0) {
          _m1Speed += 0.1;
          if (_m1Speed > 0) {
            _m1Speed = 0;
          }
        }
        _motorAndEncoderManager->setMotorSpeeds(_m0Speed, _m1Speed);
        checkTime = millis() + 50;
      }
    }
  };
    
  private:
    uint8_t _count;
    MotorAndEncoderManager* _motorAndEncoderManager;
    double _m0Speed;
    double _m1Speed;
    uint32_t _m0LastEncoderVal;
    uint32_t _m0LastReadTime;
    uint32_t _m1LastEncoderVal;
    uint32_t _m1LastReadTime;
};

#endif // MEASURESPEEDTASK_H
