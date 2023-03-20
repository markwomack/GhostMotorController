//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

#ifndef PIN_ASSIGNMENTS_H
#define PIN_ASSIGNMENTS_H

#include <inttypes.h>

// Teensy 4.0 - Would need to verify for other Teensy models
const uint8_t M0_DIR_PIN(2);
const uint8_t M0_BRAKE_PIN(3);
const uint8_t M0_SPEED_PIN(4);
const uint8_t M0_W_ENCODER_SIGNAL_PIN(5);  // green  - Hc
const uint8_t M0_V_ENCODER_SIGNAL_PIN(6);  // blue   - Hb
const uint8_t M0_U_ENCODER_SIGNAL_PIN(7);  // yellow - Ha
const uint8_t M1_W_ENCODER_SIGNAL_PIN(8);  // green  - Hc
const uint8_t M1_V_ENCODER_SIGNAL_PIN(9);  // blue   - Hb
const uint8_t M1_U_ENCODER_SIGNAL_PIN(10); // yellow - Ha
const uint8_t M1_DIR_PIN(11);
const uint8_t M1_BRAKE_PIN(12);
const uint8_t LED_BUILTIN_PIN(13); // Used by TaskManager blink task
const uint8_t M1_SPEED_PIN(14);
const uint8_t BUTTON_PIN(15);

// not used in this sketch, but here for documentation
const uint8_t I2C_SCL_PIN(16);
const uint8_t I2C_SDA_PIN(17);
const uint8_t RC_ENABLE_PIN(18);
const uint8_t SERIAL_TX_PIN(20);
const uint8_t SERIAL_RX_PIN(21);
const uint8_t RC_CH1_PIN(22);
const uint8_t RC_CH2_PIN(23);

#endif // PIN_ASSIGNMENTS_H
