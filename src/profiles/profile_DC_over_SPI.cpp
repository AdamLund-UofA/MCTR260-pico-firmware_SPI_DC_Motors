
#include "project_config.h"

#if defined(ENABLE_DC_MOTOR_1_SPI) || defined(ENABLE_DC_MOTOR_2_SPI)

#include "profile_DC_over_SPI.h"
#include <Arduino.h>
#include <math.h>

// Define DC motor pins directly from PICO, following the 
// /resources/motor-control-pcbs-main/motor-control-pcbs-main/Datasheets/Inventor_2040_w_schematic_annotated.pdf
// for pin assignments see drivers/Custom_DC_motor_driver.h

#define HIGH 1
#define LOW 0

// Deadzone threshold - inputs below this magnitude are treated as "stop"
static const float AUX_DEADZONE = 15.0f;

// =========================================================================
// Apply aux channel inputs to auxiliary motors
// =========================================================================

// Convert input magnitude to PWM
static int inputToPwm(float input, float deadzone) {
  float mag = fabs(input);

  if (mag <= deadzone) {
    return 0;
  }

  // Map remaining range to 0..255. (8 bit)
  float scaled = (mag - deadzone) / (100.0f - deadzone); 
  if (scaled > 1.0f) scaled = 1.0f;
  if (scaled < 0.0f) scaled = 0.0f;

  return (int)(scaled * 255.0f);
}

static void driveMotorPWM(uint8_t pinP, uint8_t pinN, float input, float deadzone) {
  int pwm = inputToPwm(input, deadzone);

  if (input > deadzone) {
    analogWrite(pinP, pwm);
    digitalWrite(pinN, LOW);
  } else if (input < -deadzone) {
    digitalWrite(pinP, LOW);
    analogWrite(pinN, pwm);
  } else {
    digitalWrite(pinP, LOW);
    digitalWrite(pinN, LOW);
  }
}

void profile_DC_over_SPI_init() {
  // Configure the pins as outputs
  pinMode(IN_MOT_1P_BIT, OUTPUT);
  pinMode(IN_MOT_1N_BIT, OUTPUT);
  pinMode(IN_MOT_2P_BIT, OUTPUT);
  pinMode(IN_MOT_2N_BIT, OUTPUT);

  // Set them to LOW on init for safety.
  digitalWrite(IN_MOT_1P_BIT, LOW);
  digitalWrite(IN_MOT_1N_BIT, LOW);
  digitalWrite(IN_MOT_2P_BIT, LOW);
  digitalWrite(IN_MOT_2N_BIT, LOW);
}

void profile_DC_over_SPI_apply(const control_command_t *cmd) {

#ifdef ENABLE_DC_MOTOR_1_SPI
  {
    float input = cmd->aux[DC_MOTOR_1_AUX_CHANNEL] * DC_MOTOR_1_DIR_INVERT;
    driveMotorPWM(IN_MOT_1P_BIT, IN_MOT_1N_BIT, input, AUX_DEADZONE);
  }
#endif

#ifdef ENABLE_DC_MOTOR_2_SPI
  {
    float input = cmd->aux[DC_MOTOR_2_AUX_CHANNEL] * DC_MOTOR_2_DIR_INVERT;
    driveMotorPWM(IN_MOT_2P_BIT, IN_MOT_2N_BIT, input, AUX_DEADZONE);
  }
#endif
}

void profile_DC_SPI_motors_stop() {

#ifdef ENABLE_DC_MOTOR_1_SPI
  digitalWrite(IN_MOT_1P_BIT, LOW);
  digitalWrite(IN_MOT_1N_BIT, LOW);
#endif

#ifdef ENABLE_DC_MOTOR_2_SPI
  digitalWrite(IN_MOT_2P_BIT, LOW);
  digitalWrite(IN_MOT_2N_BIT, LOW);
#endif
}


#else
// Stub implementations when no DC SPI motors are enabled
#include "profile_DC_over_SPI.h"
void profile_DC_over_SPI_apply(const control_command_t *cmd) {
  (void)cmd;
}
void profile_DC_SPI_motors_stop() {}
#endif
