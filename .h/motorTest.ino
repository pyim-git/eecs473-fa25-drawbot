#include "StepperControl.h"

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
StepperControl motorControl(DIR_PIN_1, STEP_PIN_1, DIR_PIN_2, STEP_PIN_2, DIR_PIN_3, STEP_PIN_3, DIR_PIN_4, STEP_PIN_4);


void setup()
{
    motorControl.setSpeed(100); // Set speed to 100 RPM
}

void loop()
{
    // Move in a square: forward, right, forward, right, etc.
    static int step = 0;
    static unsigned long lastMoveTime = 0;
    const unsigned long moveDuration = 1000; // ms per side

    if (millis() - lastMoveTime > moveDuration) {
        switch (step % 4) {
            case 0: // Move forward
                motorControl.forward();
                break;
            case 1: // Turn right
                motorControl.right(90);
                break;
            case 2: // Move forward
                motorControl.forward();
                break;
            case 3: // Turn right
                motorControl.right(90);
                break;
        }
        step++;
        lastMoveTime = millis();
    }
}


