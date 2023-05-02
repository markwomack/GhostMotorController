

#ifndef CHECKFOROTATASK_H
#define CHECKFOROTATASK_H

#include <Arduino.h>

#include <Task.h>

#include "FXUtil.h"     // https://github.com/joepasquariello/FlasherX
extern "C" {
  #include "FlashTxx.h"    // TLC/T3x/T4x/TMM flash primitives
}

// This is a task that checks a TCP port for avialable
// data, and raises a flag to indicate that an update
// is available.
class CheckForOTATask : public Task {
  public:
    void setTCPServer(WiFiServer* tcpServer) {
      _tcpServer = tcpServer;
    }
    
    void start(void) { 
      if (_tcpServer == 0) {
        DebugMsgs.debug().println("TCP Server not specified!");
      }
      
      _otaIsAvailable = false;
    }
    
    void update(void) {
      WiFiClient tcpClient = _tcpServer->available();

      // If a client was returned, store it for future use
      // and indicate an OTA is now available
      if (tcpClient) {
        _tcpClient = tcpClient;
        _otaIsAvailable = true;
      }
    }
    
    boolean otaIsAvailable(void) {
      return _otaIsAvailable;
    }

    // Performs the actual firmware update using the FlasherX
    // methods.
    void performUpdate() {
      if (!_otaIsAvailable) {
        DebugMsgs.debug().println("No update available");
        return;
      }
      
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
    
      update_firmware((Stream*)&_tcpClient, (Stream*)&Serial, buffer_addr, buffer_size);
      
      // return from update_firmware() means error, so clean up and
      // reboot to ensure that static vars get boot-up initialized before retry
      DebugMsgs.debug().println( "erase FLASH buffer / free RAM buffer..." );
      firmware_buffer_free( buffer_addr, buffer_size );
      Serial.flush();
      REBOOT;
    }

  private:
    WiFiServer* _tcpServer = 0;
    WiFiClient _tcpClient = 0;
    boolean _otaIsAvailable;
};

#endif // CHECKFOROTATASK_H
