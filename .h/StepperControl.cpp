#include "Arduino.h"
#include "Stepper.h"

StepperControl::StepperControl(int motor1_pin_1, int motor1_pin_2,
                               int motor2_pin_1, int motor2_pin_2,
                               int motor3_pin_1, int motor3_pin_2,
                               int motor4_pin_1, int motor4_pin_2) 
                               : motor1(STEPS, motor1_pin_1, motor1_pin_2),
    motor2(STEPS, motor2_pin_1, motor2_pin_2),
    motor3(STEPS, motor3_pin_1, motor3_pin_2),
    motor4(STEPS, motor4_pin_1, motor4_pin_2)
{
  this->motor1_pin_1 = motor1_pin_1;
  this->motor1_pin_2 = motor1_pin_2;
  this->motor2_pin_1 = motor2_pin_1;
  this->motor2_pin_2 = motor2_pin_2;
  this->motor3_pin_1 = motor3_pin_1;
  this->motor3_pin_2 = motor3_pin_2;
  this->motor4_pin_1 = motor4_pin_1;
  this->motor4_pin_2 = motor4_pin_2;
}

void StepperControl::setSpeed(long whatSpeed) {
  motor1.setSpeed(whatSpeed);
  motor2.setSpeed(whatSpeed);
  motor3.setSpeed(whatSpeed);
  motor4.setSpeed(whatSpeed);
}

void StepperControl::forward() {
  motor1.step(1);
    motor2.step(1);
    motor3.step(1);
    motor4.step(1);
}

void StepperControl::backwards() {
  motor1.step(-1);
    motor2.step(-1);
    motor3.step(-1);
    motor4.step(-1);
}

void StepperControl::rotate(bool clockwise) {
  if (clockwise) {
    motor1.step(-1);
    motor2.step(1);
    motor3.step(-1);
    motor4.step(1);
  } else {
    motor1.step(1);
    motor2.step(-1);
    motor3.step(1);
    motor4.step(-1);
  }
}

void StepperControl::left(uint8_t degree) {
  while (degree > 0) {
  motor1.step(-1);
    motor2.step(1);
    motor3.step(-1);
    motor4.step(1);
    degree--;
  }
}

void StepperControl::right(uint8_t degree) {
  while (degree > 0) {
        motor1.step(1);
    motor2.step(-1);
    motor3.step(1);
    motor4.step(-1);
    degree--;
  }
}

void StepperControl::stop() {
  // No action needed for stepper motors to stop
  return;
}

