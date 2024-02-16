#include "hand_mode.h"
#include "Arduino.h"
#include "Servo.h"
#include "pwm_input.h"

extern Servo esc;
extern Servo str;
extern PwmInput rx_str;
extern PwmInput rx_esc;


HandMode::HandMode() {
  name = "hand";
}

void HandMode::begin() {
  // nh.loginfo("begin of hand mode");
}

void HandMode::end() {
    str.writeMicroseconds(1500);
    esc.writeMicroseconds(1500);
}

void HandMode::execute() {
  if(rx_str.pulse_us() > 0 && rx_esc.pulse_us() > 0) {
    str.writeMicroseconds(rx_str.pulse_us());
    esc.writeMicroseconds(rx_esc.pulse_us());
  } else {
    esc.writeMicroseconds(1500);
    str.writeMicroseconds(1500);
  }
}

