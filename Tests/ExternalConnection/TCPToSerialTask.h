
#ifndef TCPTOSERIAL_H
#define TCPTOSERIAL_H

#include <Arduino.h>

#include <WiFiServer.h>
#include <WiFiClient.h>

#include <Task.h>

// This is a task that checks a TCP port for avialable
// data, and then pushes the data to its serial port.
class TCPToSerialTask : public Task {
  public:
    void setTCPServer(WiFiServer* tcpServer) {
      _tcpServer = tcpServer;
    }

    void setSerial(HardwareSerial* serial) {
      _serial = serial;
    }
    
    void start(void) { 
      if (_tcpServer == 0) {
        DebugMsgs.debug().println("TCP Server not specified!");
      }
    }
    
    void update(void) {
      WiFiClient tcpClient = _tcpServer->available();
      
      if (tcpClient) {
        DebugMsgs.debug().println("New tcp data available");
        uint32_t totalSize = 0;
        uint32_t size;
        uint8_t buffer[512];
        while (tcpClient.available()) {
          size = tcpClient.readBytes(buffer, 512);
          totalSize += _serial->write(buffer, size);
        }
        DebugMsgs.debug().printfln("All data sent: %d", totalSize);
        tcpClient.stop();
      }
    }

  private:
    WiFiServer* _tcpServer = 0;
    HardwareSerial* _serial;
};

#endif // TCPTOSERIAL_H
