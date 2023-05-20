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
#include <WiFiNetworkHub.h>
#include <CheckForTCPUpdateTask.h>
#include <FlasherXUpdater.h>

// Local includes
#include "pin_assignments.h"
#include "constants.h"
#include "secrets.h"
#include "TCPToSerialTask.h"

WiFiNetworkHub networkHub;

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
CheckForTCPUpdateTask checkForTCPUpdateTask;
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
  networkHub.setPins(WIFI_SPI_MOSI0_PIN, WIFI_SPI_MISO0_PIN, WIFI_SPI_SCK0_PIN, WIFI_SPI_CS0_PIN, WIFI_RESET_PIN, WIFI_BUSY_PIN);
  networkHub.setHostIPAddress(HOST_IP_ADDRESS);
  if (networkHub.start(SECRET_SSID, SECRET_PASS, DebugMsgs.getPrint())) {
    
    DebugMsgs.print("Switching to UDP for debug messages: ").print(TARGET_IP_ADDRESS).print(":").println(UDP_TARGET_PORT1);
    UDPPrintWrapper* udpPrint =
        new UDPPrintWrapper(networkHub.getUdpPort(DEBUG_UDP_PORT1), TARGET_IP_ADDRESS, UDP_TARGET_PORT1);
      
    DebugMsgs.setPrintWrapper(udpPrint);
    DebugMsgs.println("Starting debug messages through remote udp");

    // set up the remote xfer printer
    UDPPrintWrapper* xferUdpPrint =
        new UDPPrintWrapper(networkHub.getUdpPort(DEBUG_UDP_PORT2), TARGET_IP_ADDRESS, UDP_TARGET_PORT2);
    xferPrinter.setPrintWrapper(xferUdpPrint);
        
    tcpToSerialTask.setTCPServer(networkHub.getTCPServer(TCP_SERVER_PORT_1));

    checkForTCPUpdateTask.setTCPServer(networkHub.getTCPServer(TCP_SERVER_PORT_2));
  } else {
    DebugMsgs.debug().println("Unable to connect to WiFi!");
    while (true) {;}
  }

  networkHub.printWiFiStatus(DebugMsgs.getPrint());

  tcpToSerialTask.setSerial(&Serial5);
  
  remoteSerialReaderTask.setSrcSerial(&Serial5);
  
  // This task will check the TCP port for the remote OTA every second
  taskManager.addTask(&tcpToSerialTask, 1000);

  // This task will check the TCP port for a local OTA every half second
  taskManager.addTask(&checkForTCPUpdateTask, 500);

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
  if (checkForTCPUpdateTask.updateIsAvailable()) {
    DebugMsgs.debug().println("OTA update available, processing...");

    // stop normal operation
    taskManager.stop();

    // perform the OTA update
    FlasherXUpdater::performUpdate(checkForTCPUpdateTask.getTCPClient());

    // update aborted before restart requires, so restart task manager
    taskManager.start();
  }
}
