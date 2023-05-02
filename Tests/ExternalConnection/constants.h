//
// Licensed under the MIT license.
// See accompanying LICENSE file for details.
//

#ifndef CONSTANTS_H
#define CONSTANTS_H

const IPAddress HOST_IP_ADDRESS(IPAddress(192, 168, 86, 101));   // Our IP address (ExternalConnection)
const IPAddress TARGET_IP_ADDRESS(IPAddress(192, 168, 86, 100)); // IP address we send stuff to

// TCP server constants
const uint32_t TCP_SERVER_PORT_1(50005); // Receive updates for DriveByRC (and xmit to Serial5 TX)
const uint32_t TCP_SERVER_PORT_2(50006); // Receive updates for ExternalConnection (this process)

// UDP client constants (not used yet)
const uint32_t UDP_TARGET_PORT1(54321);  // Send messages from ExternalConnection (this process)
const uint32_t UDP_TARGET_PORT2(54322);  // Send messages from process sending data on Serial5 (received on Serial5 RX)
const uint32_t DEBUG_UDP_PORT1(1234);  
const uint32_t DEBUG_UDP_PORT2(1235);  

#endif // CONSTANTS_H
