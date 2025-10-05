#include "Stepper.h"

#define DIR_PIN_1  2  // Direction pin for motor 1
#define STEP_PIN_1 3  // Step pin for motor 1
#define DIR_PIN_2  4  // Direction pin for motor 2
#define STEP_PIN_2 5  // Step pin for motor 2
#define DIR_PIN_3  6  // Direction pin for motor 3
#define STEP_PIN_3 7  // Step pin for motor 3
#define DIR_PIN_4  8  // Direction pin for motor 4
#define STEP_PIN_4 9  // Step pin for motor 4

/*
This is the arduino motor control script for controlling 4 DC motors with an ESP32 microcontroller.
It is designed to work with a motor driver that uses 8 GPIO pins to control 4 motors (each motor requires 2 pins for direction and speed control).
*/

setup()
{
    ESP32MotorControl motorControl;  // Create an instance of the motor control class
    motorControl.attachMotors(DIR_PIN_1, STEP_PIN_1, DIR_PIN_2, STEP_PIN_2, DIR_PIN_3, STEP_PIN_3, DIR_PIN_4, STEP_PIN_4);

}

loop()
{
    // Main code can go here if needed
}


