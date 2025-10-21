#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

/* Header for ESP32MotorControl
 *
 * Copyright (C) 2018  Joao Lopes https://github.com/JoaoLopesF/ESP32MotorControl
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * This header file describes the public API for SerialDebug.
 *
 */

// LIBRARIES
#include "Arduino.h"        // download this please :)

class ESP32MotorControl
{
public:

	// *** VARIABLES: ***
	uint16_t mMotorSpeed[2] = {0, 0};                   // stores individual motor speed
	boolean mMotorForward[2] = {true, true};            // stores if motor is moving forward

	// *** FUNCTIONS: ***
    // attach 2 motors to specific ESP32 GPIO pins
    // In1 + In2 (left motor) - In3 + In4 (right motor)
	void attachMotors(uint8_t gpioIn1, uint8_t gpioIn2, uint8_t gpioIn3, uint8_t gpioIn4);

    // moves robot forward
        // LEFT - CCW : RIGHT - CW
	void motorForward(uint8_t motor, uint8_t speed);

    // moves robot backward
        // LEFT - CW : RIGHT - CCW
	void motorReverse(uint8_t motor, uint8_t speed);

    // turns off one motor on the robot
        // TURN LEFT - turn off left
        // TURN RIGHT - turn off right
	void motorStop(uint8_t motor);

    // turns off both motors on the robot
	void motorsStop();

    // came with the origianl library - unsure of what it does
    // void handle();

    // outputs current speed of specified motor
    // this data is stored in mMotorSpeed array
	uint8_t getMotorSpeed(uint8_t motor);

    // checks if specified motor is moving forward
    // returns true if motor is moving forward
    // returns false if motor is not moving or moving backwards
	boolean isMotorForward(uint8_t motor);

    // checks if specified motor is not moving
    // returns true if motor is not moving
    // returns false if motor is moving (forwards/backwards)
	boolean isMotorStopped(uint8_t motor);

    // NOTE: can check if motor is moving backward if it returns false on both FORWARD and STOPPED checks
        // isMotorBackward = !isMotorForward && !isMotorStopped

private:

	// *** VARIABLES: ***
    // tells if a motor is attached to left / right
	boolean mMotorAttached[2] = {false, false};

	// *** FUNCTIONS: ***
    // tells if given motor is a valid motor number and if it's attached or not
	boolean isMotorValid(uint8_t motor);
};

#endif // ESP32MotorControl_H

#endif