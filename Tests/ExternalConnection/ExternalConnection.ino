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
        uint32_t size = _srcSerial->readBytes(buffer, sizeof(buffer));
        if (size == 0) {
          return;
        }

        //DebugMsgs.debug().printfln("Starting transfer for %d bytes", size);
        uint8_t lineBuffer[4096];
        for (size_t totalSize = 0; totalSize < size;) {
          bool readLF = false;
          size_t sendSize = readUntilLF(buffer+totalSize, size - totalSize, lineBuffer, sizeof(lineBuffer), &readLF);
          //DebugMsgs.debug().printfln("totalSize %d, sendSize %d, lineBuffer '%s'", totalSize, sendSize, lineBuffer);
          totalSize += sendSize;
          if (readLF) {
            xferPrinter.println((char*)lineBuffer);
          } else {
            xferPrinter.print((char*)lineBuffer);
            xferPrinter.flush();
          }
        }
        //DebugMsgs.debug().println("Transfer complete");
      }
    };

  private:
    HardwareSerial* _srcSerial;
    uint8_t buffer[16384];

    size_t readUntilLF(uint8_t* src, size_t sizeSrc, uint8_t* dst, size_t sizeDst, bool* readLF) {
      size_t x = 0;
      *readLF = false;
      for (x = 0; x < sizeSrc && x < sizeDst-1; x++) {
        if (src[x] == '\n') {
          *readLF = true;
          break;
        } else {
          dst[x] = src[x];
        }
      }

      dst[x] = 0;
      return x + (*readLF ? 1 : 0);
    };
};

#define SERIAL_BUFFER_SIZE 16384
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
  
  // This task will check the TCP port for the remote firmware update every 5ms
  taskManager.addTask(&tcpToSerialTask, 1);

  // This task will check the TCP port for a local OTA every second
  taskManager.addTask(&checkForTCPUpdateTask, 1000);

  // This task will read from the 'remote' serial and xfer to udp
  taskManager.addTask(&remoteSerialReaderTask, 1);

  // This LED will blink (half second) during normal operations of the sketch
  taskManager.addBlinkTask(LED_STATUS_PIN, 500);

  // Start normal operations
  taskManager.start();
}

void loop() {
  // Normal operation
  taskManager.update();

  // If there is a firmware update, stop everything and process
  if (checkForTCPUpdateTask.updateIsAvailable()) {
    DebugMsgs.debug().println("Firmware update available, processing...");

    // stop normal operation
    taskManager.stop();

    FlasherXUpdater::setTimeout(100);
    
    // perform the OTA update
    FlasherXUpdater::performUpdate(checkForTCPUpdateTask.getUpdateStream());

    // bleed any remaining update data
    DebugMsgs.debug().println("Firmware update aborted, clearing remaining update data");
    Stream* updateStream = checkForTCPUpdateTask.getUpdateStream();
    uint32_t lastRead = millis();
    while (millis() < lastRead + 500) {
      if (updateStream->available()) {
        uint8_t buffer[4096];
        updateStream->readBytes(buffer, sizeof(buffer));
        lastRead = millis();
      }
    }

    // update aborted, restart task manager
    DebugMsgs.debug().println("Firmware update aborted, restarting normal operations");
    taskManager.start();
  }
}
