//
// Licensed under the MIT license.
// See accompanying LICENSE file for details.
//

// Arduino includes
#include <Arduino.h>
#include <WiFiServer.h>
#include <WiFiClient.h>

// My includes
#include <DebugMsgs.h>   // https://github.com/markwomack/ArduinoLogging
#include <CascadePrinter.h>
#include <UDPPrintWrapper.h>
#include <TaskManager.h> // https://github.com/markwomack/TaskManager

// Local includes
#include "pin_assignments.h"
#include "constants.h"
#include "MyNetworkHub.h"
#include "CheckForOTATask.h"
#include "TCPToSerialTask.h"

MyNetworkHub networkHub;

CascadePrinter xferPrinter;

class RemoteSerialReaderTask : public Task {
  public:
    void setSrcSerial(HardwareSerial* srcSerial) {
      _srcSerial = srcSerial;
    };

    void update(void) {
      if (_srcSerial->available()) {
        uint32_t size = _srcSerial->readBytesUntil('\r', buffer, 1023);
        if (size == 0) {
          return;
        }

        buffer[size] = 0;
        xferPrinter.print(buffer);
        
        int nextByte = _srcSerial->read();
        if (nextByte == -1) {
          return;
        } else if (nextByte == '\n') {
          xferPrinter.println();
          xferPrinter.flush();
        } else {
          xferPrinter.print((char)nextByte);
        }
      }
    };

  private:
    HardwareSerial* _srcSerial;
    char buffer[1024];
};

#define SERIAL_BUFFER_SIZE 8192
uint8_t incomingBuffer[SERIAL_BUFFER_SIZE];
uint8_t outgoingBuffer[SERIAL_BUFFER_SIZE];

TCPToSerialTask tcpToSerialTask;
CheckForOTATask checkForLocalOTATask;
RemoteSerialReaderTask remoteSerialReaderTask;

void setup() {
  Serial.begin(115200);
  Serial5.begin(115200);
  Serial5.addMemoryForRead(incomingBuffer, SERIAL_BUFFER_SIZE);
  Serial5.addMemoryForWrite(outgoingBuffer, SERIAL_BUFFER_SIZE);
  delay(500);
  
  DebugMsgs.enableLevel(DEBUG);

  pinMode(LED_STATUS_PIN, OUTPUT);

  // Connect to WiFi network, create a TCP port to monitor
  if (networkHub.start() == 0) {
    
    DebugMsgs.print("Switching to UDP for debug messages: ").print(TARGET_IP_ADDRESS).print(":").println(UDP_TARGET_PORT1);
    UDPPrintWrapper* udpPrint =
        new UDPPrintWrapper(networkHub.getUdpPort(DEBUG_UDP_PORT1), TARGET_IP_ADDRESS, UDP_TARGET_PORT1);
      
    DebugMsgs.setPrint(udpPrint);
    DebugMsgs.println("Starting debug messages through remote udp");

    // set up the remote xfer printer
    UDPPrintWrapper* xferUdpPrint =
        new UDPPrintWrapper(networkHub.getUdpPort(DEBUG_UDP_PORT2), TARGET_IP_ADDRESS, UDP_TARGET_PORT2);
    xferPrinter.setPrint(xferUdpPrint);
        
    WiFiServer* tcpServer1 = networkHub.getTCPServer(TCP_SERVER_PORT_1);
    tcpToSerialTask.setTCPServer(tcpServer1);

    WiFiServer* tcpServer2 = networkHub.getTCPServer(TCP_SERVER_PORT_2);
    checkForLocalOTATask.setTCPServer(tcpServer2);
  } else {
    DebugMsgs.debug().println("Unable to connect to WiFi!");
    while (true) {;}
  }

  networkHub.printWifiStatus();

  tcpToSerialTask.setSerial(&Serial5);
  
  remoteSerialReaderTask.setSrcSerial(&Serial5);
  
  // This task will check the TCP port for the remote OTA every second
  taskManager.addTask(&tcpToSerialTask, 1000);

  // This task will check the TCP port for a local OTA every half second
  taskManager.addTask(&checkForLocalOTATask, 500);

  // This task will read from the 'remote' serial and xfer to udp
  taskManager.addTask(&remoteSerialReaderTask, 5);

  // This LED will blink (half second) during normal operations of the sketch
  taskManager.addBlinkTask(LED_STATUS_PIN, 500);

  // Start normal operations
  taskManager.start();
}

void loop() {
  // Normal operation
  taskManager.update();

  // If there is an OTA, stop everything and process
  if (checkForLocalOTATask.otaIsAvailable()) {
    DebugMsgs.debug().println("OTA update available, processing...");

    // stop normal operation
    taskManager.stop();

    // perform the OTA update
    checkForLocalOTATask.performUpdate();

    // update aborted, so restart task manager
    taskManager.start();
  }
}
