//
// Licensed under Apache 2.0 license.
// See accompanying LICENSE file for details.
//

#ifndef PIN_ASSIGNMENTS_H
#define PIN_ASSIGNMENTS_H

#include <inttypes.h>

// Teensy 3.5/4.0/4.1 - These assignments work across all versions
const uint8_t M1_MOTOR_DIR_PIN(2);
const uint8_t M1_BRAKE_PIN(3);
const uint8_t M1_PWM_SPEED_PIN(4);
const uint8_t M1_W_ENCODER_SIGNAL_PIN(5);  // green  - Hc
const uint8_t M1_V_ENCODER_SIGNAL_PIN(6);  // blue   - Hb
const uint8_t M1_U_ENCODER_SIGNAL_PIN(7);  // yellow - Ha
const uint8_t M2_W_ENCODER_SIGNAL_PIN(8);  // green  - Hc
const uint8_t M2_V_ENCODER_SIGNAL_PIN(9);  // blue   - Hb
const uint8_t M2_U_ENCODER_SIGNAL_PIN(10); // yellow - Ha
const uint8_t M2_BRAKE_PIN(12);
const uint8_t M2_MOTOR_DIR_PIN(11);
const uint8_t LED_BUILTIN_PIN(13); // Used by TaskManager blink task
const uint8_t M2_PWM_SPEED_PIN(14);
const uint8_t I2C_SDA(18);
const uint8_t I2C_SCL(19);
const uint8_t SERIAL_TX(20);
const uint8_t SERIAL_RX(21);
const uint8_t BUTTON_PIN(23);

#endif // PIN_ASSIGNMENTS_H
