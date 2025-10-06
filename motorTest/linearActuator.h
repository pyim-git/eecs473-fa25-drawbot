#ifndef LINEAR_ACTUATOR_H
#define LINEAR_ACTUATOR_H

// LIBRARIES
#include "Arduino.h" 

// Header for LinearActuator interface for ESP32-based systems

class LinearActuator 
{
public:

    // *** VARIABLES: ***
	uint16_t lASpeed = 0;                   // stores individual motor speed
    
    // *** FUNCTIONS: ***
    // Attach and set up the actuator GPIO pins
    void attach(int pin1, int pin2, int enablePin) = 0;

    // Initialize the actuator hardware
    void initialize() = 0;

    // Move actuator to a specific position (in mm or encoder ticks)
    void moveToPosition(uint8_t position) = 0;

    // Get the current position of the actuator
    uint8_t getPosition() const = 0;

    // Stop the actuator movement
    void stop() = 0;

    // Set the speed of the actuator (units depend on implementation)
    void setSpeed(uint8_t speed) = 0;

    // Get the current speed setting
    uint8_t getSpeed() const = 0;

    // Check if the actuator is currently moving
    bool isMoving() const = 0;

    private:

	// *** VARIABLES: ***
    boolean mLAAttached = false;          // tells if linear actuator is attached
	// *** FUNCTIONS: ***
    // tells if given motor is a valid motor number and if it's attached or not
	boolean isLAValid(uint8_t linearActuator);
};

#endif // LINEAR_ACTUATOR_H