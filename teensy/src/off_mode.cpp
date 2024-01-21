#include "off_mode.h"
#include "Arduino.h"
#include "Servo.h"
#include "pwm_input.h"

extern Servo esc;
extern Servo str;
extern PwmInput rx_str;
extern PwmInput rx_esc;


OffMode::OffMode() {
  name = "off";
}

void OffMode::begin() {
}

void OffMode::end() {
    str.writeMicroseconds(1500);
    esc.writeMicroseconds(1500);
}

void OffMode::execute() {
  if(rx_str.pulse_us() > 0 && rx_esc.pulse_us() > 0) {
    str.writeMicroseconds(rx_str.pulse_us());
    esc.writeMicroseconds(1500);
  } else {
    esc.writeMicroseconds(1500);
    str.writeMicroseconds(1500);
  }
}

