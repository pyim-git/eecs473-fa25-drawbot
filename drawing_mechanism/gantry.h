// *** LIBRARIES ***
#include "DRV8825.h"  // DRV8825 library for stepper motor control
#include "ESP32Servo.h"    // servo library for ESP32

// ========================
// ***** GANTRY CLASS *****
// ========================
class GANTRY {
  private:
    // NUMBERS FOR PRECISION
    double radius = 6.0;    // in mm
    double circumference = 2*PI*radius;   // in mm
    int tot_num_steps = 200;
    double step_dist = 2*PI*radius / 200;
    int wait = 1;

    DRV8825 gantry;    // STEPPER MOTOR FOR GANTRY
    Servo z_axis;       // servo for z axis movement (underneath gripper)
    Servo gripper;      // servo for grippers

  public:
    // *** FUNCTIONS! ***
    // NOTE: with the current stepper motor resolution (1.8˚) and the radius of the pulley wheel (6mm), 
    // each step is equivalent to moving 0.188 mm
      // this can change with microstepping
    
    // initialize servos and driver
    void init();

    // move marker to the right (from robot's perspective)
    // function takes in a distance in mm and speed in mm/s
    void moveRight(double distance, double speed);

    // move marker to the left (from robot's perspective)
    // function takes in a distance in mm and speed in mm/s
    void moveLeft(double distance, double speed);

    // *** MARKER COMMANDS ***
    // grab marker - change angle as needed
    void grab();

    // release marker - change angle as needed
    void release();

    // move marker up - hasn't been tested yet
    void markerUp();

    // move marker down - hasn't been tested yet
    void markerDown();
};
// -- END OF FILE --
