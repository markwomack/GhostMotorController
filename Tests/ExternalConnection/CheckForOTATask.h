

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

      CRCStream* updateStream = getCRCStream(&_tcpClient);
      if (!updateStream) {
        return;
      }
      
      uint32_t buffer_addr;
      uint32_t buffer_size;
    
      // create flash buffer to hold new firmware
      if (firmware_buffer_init( &buffer_addr, &buffer_size ) == 0) {
        DebugMsgs.debug().println( "unable to create buffer" );
        DebugMsgs.flush();
        free(updateStream);
        return;
      }
      
      DebugMsgs.debug().printfln( "created buffer = %1luK %s (%08lX - %08lX)",
       buffer_size/1024, IN_FLASH(buffer_addr) ? "FLASH" : "RAM",
        buffer_addr, buffer_addr + buffer_size );
    
      update_firmware(updateStream, (Stream*)&Serial, buffer_addr, buffer_size);
      
      // return from update_firmware() means error, so clean up and
      // reboot to ensure that static vars get boot-up initialized before retry
      DebugMsgs.debug().println( "erase FLASH buffer / free RAM buffer...restarting in 5 seconds" );
      firmware_buffer_free( buffer_addr, buffer_size );
      DebugMsgs.flush();
      delay(5000);
      REBOOT;
    };

    CRCStream* getCRCStream(WiFiClient* tcpClient) {    
      uint32_t expectedSize = 0;
      uint32_t expectedCRC = 0;
      
      uint8_t buffer[25];
      int count = 0;
      while (tcpClient->available()) {
        int data = tcpClient->read();
        if (data != -1) {
          if (data == '!') {
            buffer[count] = 0;
            expectedSize = strtoul((const char*)buffer, 0, 10);
            break;
          } else {
            buffer[count++] = (uint8_t)data;
          }
        }
      }
    
      if (expectedSize == 0) {
        DebugMsgs.debug().println("Error reading the expected size, aborted");
        return 0;
      }
    
      count = 0;
      while (tcpClient->available()) {
        count += tcpClient->readBytes(buffer + count, 9 - count);
        if (count == 9) {
          if (buffer[8] == '!') {
            buffer[8] = 0;
            expectedCRC = strtoul((const char*)buffer, 0, 16);
          }
          break;
        }
      }
    
      if (expectedCRC == 0) {
        DebugMsgs.error().println("Error reading the expected crc, aborted");
        return 0;
      }
    
      DebugMsgs.debug().printfln("Expected update size: %d, expected update CRC: %x", expectedSize, expectedCRC);
      
      return new CRCStream(tcpClient, expectedSize, expectedCRC);
    };

  private:
    WiFiServer* _tcpServer = 0;
    WiFiClient _tcpClient = 0;
    boolean _otaIsAvailable;
};

#endif // CHECKFOROTATASK_H
