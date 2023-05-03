//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

#ifndef CHECKFOROTATASK_H
#define CHECKFOROTATASK_H

#include <Arduino.h>
#include "FXUtil.h"     // https://github.com/joepasquariello/FlasherX
extern "C" {
  #include "FlashTxx.h"    // TLC/T3x/T4x/TMM flash primitives
}

// This is a task that checks a serial port for avialable
// data, and raises a flag to indicate that an update
// is available.
class CheckForOTATask : public Task {
  public:
    void setSerial(HardwareSerial* serial) {
      _serial = serial;
    }
    
    void start(void) { 
      if (_serial == 0) {
        DebugMsgs.debug().println("Serial port not specified!");
      }
      
      _otaIsAvailable = false;
    }
    
    void update(void) {
      _otaIsAvailable = _serial->available();
    }
    
    boolean otaIsAvailable(void) {
      return _otaIsAvailable;
    }

    // Performs the actual firmware update using the FlasherX
    // methods.
    void performUpdate() {
      if (!_otaIsAvailable) {
        DebugMsgs.debug().println( "no ota update available" );
        return;
      }

      DebugMsgs.debug().println("Starting OTA update");
      uint32_t buffer_addr;
      uint32_t buffer_size;
    
      // create flash buffer to hold new firmware
      if (firmware_buffer_init( &buffer_addr, &buffer_size ) == 0) {
        DebugMsgs.debug().println( "unable to create buffer" );
        DebugMsgs.flush();
        return;
      }
      
      Serial.printf( "created buffer = %1luK %s (%08lX - %08lX)\n",
        buffer_size/1024, IN_FLASH(buffer_addr) ? "FLASH" : "RAM",
        buffer_addr, buffer_addr + buffer_size );
    
      update_firmware((Stream*)_serial, (Stream*)&Serial, buffer_addr, buffer_size);
      
      // return from update_firmware() means error, so clean up and
      // reboot to ensure that static vars get boot-up initialized before retry
      DebugMsgs.debug().println( "erase FLASH buffer / free RAM buffer..." );
      firmware_buffer_free( buffer_addr, buffer_size );
      Serial.flush();
      REBOOT;
    }

  private:
    HardwareSerial* _serial = 0;
    boolean _otaIsAvailable;
};

#endif
