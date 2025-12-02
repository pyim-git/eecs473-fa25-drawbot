// *** LIBRARIES ***
#include "DRV8825.h"  // DRV8825 library for stepper motor control
#include "ESP32Servo.h"    // servo library for ESP32
// #include "Servo.h"      // servo library for Arduino (prototype testing)

// ========================
// ***** GANTRY CLASS *****
// ========================
class GANTRY {
  private:
    float position = 0.0;   // position of gantry - update as needed
    float BELT_LENGTH = 100.0;   // length of belt (max distance gantry can travel)
  
    // CAN BE CHANGED
    int waitTime = 500;   // delay time between commands

    // NUMBERS FOR PRECISION
    int tot_num_steps = 200;
    int wait = 1;
    double radius = 6.0;    // in mm
    double circumference = 2*PI*radius;   // in mm
    double step_dist = 2*PI*radius / 200;

    Servo z_axis;       // servo for z axis movement (underneath gripper)
    Servo gripper;      // servo for grippers
    Servo tool_change;  // servo for tool change rack
    DRV8825 gantry;     // STEPPER MOTOR FOR GANTRY

  public:
    // *** FUNCTIONS! ***
    // NOTE: with the current stepper motor resolution (1.8˚) and the radius of the pulley wheel (6mm), 
    // each step is equivalent to moving 0.188 mm
      // this can change with microstepping
    
    // initialize servos and driver
    void init();

    // move marker to the right (from robot's perspective)
    // function takes in a distance in mm and speed in mm/s
    void moveRight(float distance, float speed);

    // move marker to the left (from robot's perspective)
    // function takes in a distance in mm and speed in mm/s
    void moveLeft(float distance, float speed);

    // move gantry fully to the left and reset its position
    void resetPos(int orientation);

    // *** MARKER COMMANDS ***
    // grab marker - change angle as needed
    void grab();

    // release marker - change angle as needed
    void release();

    // move marker up - hasn't been tested on whiteboard yet
    void markerUp();

    // move marker down - hasn't been tested on whiteboard yet
    void markerDown();

    // put marker back into rack
    // MUST CALL MOVE GANTRY TO POSITION BEFORE CALLING THIS
    // NEED TO CHANGE TO AUTOMATE THIS
    void putMarkerBack(int numPos);

    // switch out marker with one in the tool rack
    // MUST CALL MOVE GANTRY TO POSITION BEFORE CALLING THIS
    // NEED TO CHANGE TO AUTOMATE THIS
    void takeMarkerFrom(int numPos);
};
// -- END OF FILE --
