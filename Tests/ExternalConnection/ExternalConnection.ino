//
// Licensed under the MIT license.
// See accompanying LICENSE file for details.
//

// Arduino includes
#include <Arduino.h>

// My includes
#include <DebugMsgs.h>              // https://github.com/markwomack/ArduinoLogging
#include <CascadePrinter.h>
#include <UDPPrintWrapper.h>
#include <TaskManager.h>           // https://github.com/markwomack/TaskManager
#include <WiFiNetworkHub.h>        // https://github.com/markwomack/TeensyNetworkHub
#include <NetworkServer.h>
#include <NetworkUDP.h>
#include <FlasherXUpdater.h>       // https://github.com/markwomack/FlasherXUpdater
#include <CheckForTCPUpdateTask.h>

// Local includes
#include "pin_assignments.h"
#include "constants.h"
#include "secrets.h"
#include "TCPToSerialTask.h"
#include "RemoteSerialReaderTask.h"
#include "GPSReaderTask.h"

WiFiNetworkHub networkHub = WiFiNetworkHub::getInstance();

CascadePrinter xferPrinter;

#define SERIAL_BUFFER_SIZE 16384
uint8_t incomingBuffer[SERIAL_BUFFER_SIZE];
uint8_t outgoingBuffer[SERIAL_BUFFER_SIZE];

TCPToSerialTask tcpToSerialTask;
CheckForTCPUpdateTask checkForTCPUpdateTask;
RemoteSerialReaderTask remoteSerialReaderTask;
GPSReaderTask gpsReaderTask;

void setup() {
  Serial.begin(115200);
  Serial5.begin(115200);
  Serial5.addMemoryForRead(incomingBuffer, SERIAL_BUFFER_SIZE);
  Serial5.addMemoryForWrite(outgoingBuffer, SERIAL_BUFFER_SIZE);
  Serial7.begin(9600);
  delay(500);
  
  DebugMsgs.enableLevel(DEBUG);

  pinMode(LED_STATUS_PIN, OUTPUT);

  // Connect to WiFi network, create a TCP port to monitor
  networkHub.setPins(WIFI_SPI_MOSI0_PIN, WIFI_SPI_MISO0_PIN, WIFI_SPI_SCK0_PIN, WIFI_SPI_CS0_PIN, WIFI_RESET_PIN, WIFI_BUSY_PIN);
  networkHub.setLocalIPAddress(HOST_IP_ADDRESS);
  if (networkHub.begin(SECRET_SSID, SECRET_PASS, DebugMsgs.getPrint())) {
    
    DebugMsgs.print("Switching to UDP for debug messages: ").print(TARGET_IP_ADDRESS).print(":").println(UDP_TARGET_PORT1);
    NetworkUDP* udpPort1 = networkHub.getUDP();
    udpPort1->begin(DEBUG_UDP_PORT1);
    UDPPrintWrapper* udpPrint = new UDPPrintWrapper(udpPort1, TARGET_IP_ADDRESS, UDP_TARGET_PORT1);
      
    DebugMsgs.setPrintWrapper(udpPrint);
    DebugMsgs.println("Starting debug messages through remote udp");

    // set up the remote xfer printer
    NetworkUDP* udpPort2 = networkHub.getUDP();
    udpPort2->begin(DEBUG_UDP_PORT2);
    UDPPrintWrapper* xferUdpPrint = new UDPPrintWrapper(udpPort2, TARGET_IP_ADDRESS, UDP_TARGET_PORT2);
    xferPrinter.setPrintWrapper(xferUdpPrint);

    NetworkServer* tcpServer1 = networkHub.getServer(TCP_SERVER_PORT_1);
    tcpServer1->begin();
    tcpToSerialTask.setTCPServer(tcpServer1);

    NetworkServer* tcpServer2 = networkHub.getServer(TCP_SERVER_PORT_2);
    tcpServer2->begin();
    checkForTCPUpdateTask.setTCPServer(tcpServer2);
  } else {
    DebugMsgs.debug().println("Unable to connect to WiFi!");
    while (true) {;}
  }

  networkHub.printStatus(DebugMsgs.getPrint());

  tcpToSerialTask.setSerial(&Serial5);
  
  remoteSerialReaderTask.setSrcSerial(&Serial5);
  remoteSerialReaderTask.setDstPrinter(&xferPrinter);

  gpsReaderTask.setSrcSerial(&Serial7);
  
  // This task will check the TCP port for the remote firmware update every 5ms
  taskManager.addTask(&tcpToSerialTask, 1);

  // This task will check the TCP port for a local OTA every second
  taskManager.addTask(&checkForTCPUpdateTask, 1000);

  // This task will check the gps data every 10th of a second
  taskManager.addTask(&gpsReaderTask, 5);

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
    DebugMsgs.debug().println("Firmware update available, processing...").flush();

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
