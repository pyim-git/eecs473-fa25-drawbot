// ensure this library description is only included once
#ifndef StepperControl_h
#define StepperControl_h

#define STEPS 200  // Number of steps per revolution for the stepper motor
#include "Stepper.h"
#include <stdint.h>

// library interface description
class StepperControl {
  public:
    // constructors:
    StepperControl(int motor1_pin_1, int motor1_pin_2, int motor2_pin_1, int motor2_pin_2, int motor3_pin_1, int motor3_pin_2, int motor4_pin_1, int motor4_pin_2);

    // speed setter method:
    void setSpeed(long whatSpeed);

    void forward();

    void backwards();

    void left(uint8_t degree);

    void right(uint8_t degree);

    void stop();

    void disableMotors();

    void rotate(bool clockwise);

  private:
    // speed
    long speed;

    // degree
    long degree;

    // stepper motor objects
    Stepper motor1(STEPS, 0, 0); // Placeholder pins, will be set in constructor
    Stepper motor2(STEPS, 0, 0);
    Stepper motor3(STEPS, 0, 0);
    Stepper motor4(STEPS, 0, 0);

    // motor pin numbers:
    int motor1_pin_1;
    int motor1_pin_2;
    int motor2_pin_1;
    int motor2_pin_2;
    int motor3_pin_1;
    int motor3_pin_2;
    int motor4_pin_1;
    int motor4_pin_2;
};

#endif
