//
// Licensed under the MIT license.
// See accompanying LICENSE file for details.
//

#ifndef GPSREADERTASK_H
#define GPSREADERTASK_H

#include <Arduino.h>
#include <DebugMsgs.h>
#include <Task.h>

class GPSReaderTask : public Task {
  public:
    void setSrcSerial(HardwareSerial* srcSerial) {
      _srcSerial = srcSerial;
    };

    void start(void) {
      offset = 0;
    };
    
    void update(void) {
      while (_srcSerial->available()) {
        if (offset >= sizeof(buffer)) {
          offset = 0;
          DebugMsgs.debug().println("Reset buffer offset!");
        }
        buffer[offset++] = (char)_srcSerial->read();
        if (offset > 1 && buffer[offset-2] == '\r' && buffer[offset-1] == '\n') {
          buffer[offset-2] = 0;
          DebugMsgs.debug().print(buffer).printfln(" ---- %d", strlen(buffer));
          offset = 0;
        }
      }
    };
    
  private:
    HardwareSerial* _srcSerial;
    uint32_t offset;
    char buffer[128];
  
};

#endif // GPSREADERTASK_H
