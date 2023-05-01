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
#include <UDPPrintWrapper.h>
#include <TaskManager.h> // https://github.com/markwomack/TaskManager

// Local includes
#include "pin_assignments.h"
#include "constants.h"
#include "MyNetworkHub.h"
#include "CheckForOTATask.h"
#include "TCPToSerialTask.h"

MyNetworkHub networkHub;

TCPToSerialTask tcpToSerialTask;
CheckForOTATask checkForLocalOTATask;

void setup() {
  Serial.begin(115200);
  Serial5.begin(115200);
  delay(500);
  
  DebugMsgs.enableLevel(DEBUG);

  pinMode(LED_STATUS_PIN, OUTPUT);

  // Connect to WiFi network, create a TCP port to monitor
  if (networkHub.start() == 0) {

    DebugMsgs.print("Switching to UDP for debug messages: ").print(UDP_TARGET_ADDRESS).print(":").println(UDP_TARGET_PORT);
    UDPPrintWrapper* udpPrint =
        new UDPPrintWrapper(networkHub.getUdpPort(DEBUG_UDP_PORT), UDP_TARGET_ADDRESS, UDP_TARGET_PORT);
      
    DebugMsgs.setPrint(udpPrint);
    DebugMsgs.println("Starting debug messages through remote udp");
    
    WiFiServer* tcpServer1 = networkHub.getTCPServer(TCP_SERVER_PORT_1);
    tcpToSerialTask.setTCPServer(tcpServer1);

    WiFiServer* tcpServer2 = networkHub.getTCPServer(TCP_SERVER_PORT_2);
    checkForLocalOTATask.setTCPServer(tcpServer2);
  } else {
    DebugMsgs.debug().println("Unable to connect to WiFi!");
    while (true) {;}
  }

  tcpToSerialTask.setSerial(&Serial5);
  
  // This task will check the TCP port for the remote OTA every second
  taskManager.addTask(&tcpToSerialTask, 1000);

  // This task will check the TCP port for a local OTA every half second
  taskManager.addTask(&checkForLocalOTATask, 500);

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
  }
}
