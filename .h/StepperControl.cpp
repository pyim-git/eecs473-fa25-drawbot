#include "Arduino.h"
#include "Stepper.h"

StepperControl::StepperControl(int motor1_pin_1, int motor1_pin_2,
                               int motor2_pin_1, int motor2_pin_2,
                               int motor3_pin_1, int motor3_pin_2,
                               int motor4_pin_1, int motor4_pin_2) {
  this->motor1_pin_1 = motor1_pin_1;
  this->motor1_pin_2 = motor1_pin_2;
  this->motor2_pin_1 = motor2_pin_1;
  this->motor2_pin_2 = motor2_pin_2;
  this->motor3_pin_1 = motor3_pin_1;
  this->motor3_pin_2 = motor3_pin_2;
  this->motor4_pin_1 = motor4_pin_1;
  this->motor4_pin_2 = motor4_pin_2;

  motor1 = Stepper motor1(STEPS, motor1_pin_1, motor1_pin_2);
  motor2 = Stepper motor2(STEPS, motor2_pin_1, motor2_pin_2);
  motor3 = Stepper motor3(STEPS, motor3_pin_1, motor3_pin_2);
  motor4 = Stepper motor4(STEPS, motor4_pin_1, motor4_pin_2);
}

void StepperControl::setSpeed(long whatSpeed) {
  motor1.setSpeed(whatSpeed);
  motor2.setSpeed(whatSpeed);
  motor3.setSpeed(whatSpeed);
  motor4.setSpeed(whatSpeed);
}

void StepperControl::forward() {
  forward = true;
  backwards = false;
  left = false;
  right = false;
  rotateClockwise = false;
  rotateCounterClockwise = false;
}

void StepperControl::backwards() {
  forward = false;
  backwards = true;
  left = false;
  right = false;
  rotateClockwise = false;
  rotateCounterClockwise = false;
}

void StepperControl::rotate(bool clockwise) {
  forward = false;
  backwards = false;
  left = false;
  right = false;
  if (clockwise) {
    rotateClockwise = true;
  } else {
    rotateCounterClockwise = true;
  }
}

void StepperControl::left(uint8_t degree) {
  forward = false;
  backwards = false;
  left = true;
  right = false;
  rotateClockwise = false;
  rotateCounterClockwise = false;
}

void StepperControl::right(uint8_t degree) {
  forward = false;
  backwards = false;
  left = false;
  right = true;
  rotateClockwise = false;
  rotateCounterClockwise = false;
}

void StepperControl::stop() {
  forward = false;
  backwards = false;
  left = false;
  right = false;
  rotateClockwise = false;
  rotateCounterClockwise = false;
}

// Call this method repeatedly in your main loop to update motor steps
void StepperControl::update() {
  if (forward) {
    motor1.step(1);
    motor2.step(1);
    motor3.step(1);
    motor4.step(1);
  } else if (backwards) {
    motor1.step(-1);
    motor2.step(-1);
    motor3.step(-1);
    motor4.step(-1);
  } else if (left && degree > 0) {
    motor1.step(-1);
    motor2.step(1);
    motor3.step(-1);
    motor4.step(1);
    degree--;
  } else if (right && degree > 0) {
    motor1.step(1);
    motor2.step(-1);
    motor3.step(1);
    motor4.step(-1);
    degree--;
  } else if (rotateClockwise) {
    motor1.step(-1);
    motor2.step(1);
    motor3.step(-1);
    motor4.step(1);
  } else if (rotateCounterClockwise) {
    motor1.step(1);
    motor2.step(-1);
    motor3.step(1);
    motor4.step(-1);
  } else {
    // Motors are stopped; do nothing
  }
}
