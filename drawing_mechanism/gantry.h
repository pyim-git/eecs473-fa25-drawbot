// *** LIBRARIES ***
// #include "DRV8825.h"  // DRV8825 library for stepper motor control
// #include <Arduino.h>
#include "ESP32Servo.h"    // servo library for ESP32
//#include "Servo.h"      // servo library for Arduino (prototype testing)
#include <math.h>

// ========================
// ***** GANTRY CLASS *****
// ========================
class GANTRY {
  private:
    // adjust as needed
    float BELT_LENGTH = 100.0;   // length of belt (max distance gantry can travel)
    int TOOL_RETRACT = 120;
    int TOOL_EXTEND = 0;
    int GRAB = 10;
    int RELEASE = 90;
    int UP = 90;
    int DOWN = 110;
  
    // CAN BE CHANGED
    int waitTime = 350;   // delay time between commands

    // NUMBERS FOR PRECISION
    int tot_num_steps = 200;
    int wait = 2000;
    double radius = 6.0;    // in mm
    double circumference = 2*PI*radius;   // in mm
    double step_dist = 2*PI*radius / 200;

    Servo z_axis;       // servo for z axis movement (underneath gripper)
    Servo gripper;      // servo for grippers
    Servo tool_change;  // servo for tool change rack
    // DRV8825 gantry;     // STEPPER MOTOR FOR GANTRY

    // private marker helper functions
    // puts marker back into tool changing rack
    // specify postion to put it back into
    void putMarkerBack(int numPos);

    // takes marker out of tool changing rack
    // specify postion to take it out of
    void takeMarkerFrom(int numPos);

    // move marker to the right (from robot's perspective)
    // function takes in a distance in mm and speed in mm/s
    void moveRight(float distance, float speed);

    // move marker to the left (from robot's perspective)
    // function takes in a distance in mm and speed in mm/s
    void moveLeft(float distance, float speed);

    // grab marker - change angle as needed
    void grab();

    // release marker - change angle as needed
    void release();

  public:
    // position of gantry belt - assuming no slipping
    float position = 0.0;   // position of gantry - update as needed

    // *** FUNCTIONS! ***
    // NOTE: with the current stepper motor resolution (1.8˚) and the radius of the pulley wheel (6mm), 
    // each step is equivalent to moving 0.188 mm
      // this can change with microstepping
    
    // initialize servos and driver
    // CALL THIS FUNCTION IN SETUP LOOP!
    void init();

    // move gripper fully to the top of the gantry
    // orientation == 0 means robot is facing to the right
    // orientation == 1 means robot is facing to the left
    void resetPos(int orientation);

    // gantry moves to an "absolute" position that it needs to move to
    void move(float pos, float speed);

    // ===== MARKER COMMANDS =====
    // move marker up - hasn't been tested on whiteboard yet
    void liftMarker();

    // move marker down - hasn't been tested on whiteboard yet
    void lowerMarker();

    // switch color marker
    void switchMarker(int colorPos1, int colorPos2);
};
// -- END OF FILE --