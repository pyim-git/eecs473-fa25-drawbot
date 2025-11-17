// *** LIBRARIES ***
#include "gantry.h"

// NOTE: IF NOT USING .H FILE - COPY THESE OTHER .H FILES INTO TOP
// EVERYTHING SHOULD WORK AND BE USUABLE
// #include "DRV8825.h"  // DRV8825 library for stepper motor control
// #include "ESP32Servo.h"    // servo library for ESP32

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

    // initialize gantry + drawing arm
    // CALL THIS FUNCTION IN SETUP LOOP!
    void init() {
      // start gantry belt stepper motor
      gantry.begin(4, 5);
      // gantry.enable()     // enable gantry
      // gantry.sleep()      // set gantry to sleep - save power
      gantry.setDirection(DRV8825_CLOCK_WISE);

      // attach pins
      z_axis.attach(10);  // attach z_axis to pin 10
      gripper.attach(9);  // attach gripper to pin 9

      markerUp();  // reset marker to up position (not touching board)
      grab();     // tighten grippers
    }

    // move marker to the right (from robot's perspective)
    // function takes in a distance in mm and speed in mm/s
    void moveRight(double distance, double speed) {
      // wake up gantry - only allowed if sleep pin set
      // gantry.wakup();
      
      // set direction to be clockwise (moving right)
      gantry.setDirection(DRV8825_CLOCK_WISE);

      // Calculate number of steps
      int num_steps = distance/step_dist;

      // calculate delay needed for specific speed
      wait = 1000*step_dist / speed;

      Serial.println("Moving gantry arm right!");
      for (int i = 0; i < num_steps; i++) {
        gantry.step();
        delay(wait);
      } // for ..move number of required steps

      // put gantry to sleep - only allowed if sleep pin set
      // gantry.sleep();

      // regrip marker (could've gotten loose)
      grab();
    } // ..moveRight()

    // move marker to the left (from robot's perspective)
    // function takes in a distance in mm and speed in mm/s
    void moveLeft(double distance, double speed) {
      // wake up gantry - only allowed if sleep pin set
      // gantry.wakup();

      // set direction to be counterclockwise (moving left)
      gantry.setDirection(DRV8825_COUNTERCLOCK_WISE);

      // Calculate number of steps
      int num_steps = distance/step_dist;

      // calculate delay needed for specific speed
      wait = 1000*step_dist / speed;

      Serial.println("Moving gantry arm left!");
      for (int i = 0; i < num_steps; i++) {
        gantry.step();
        delay(wait);
      } // for ..move number of required steps
      
      // put gantry to sleep - only allowed if sleep pin set
      // gantry.sleep();
      
      // regrip marker (could've gotten loose)
      grab();
    } // ..moveLeft()

    // *** MARKER COMMANDS ***
    // grab marker - change angle as needed
    void grab() {
      // tighten gripper for servo
      gripper.write(180);
    } // ..grab()

    // release marker - change angle as needed
    void release() {
      // loosen gripper for servo
      gripper.write(40);
    }

    // move marker up - hasn't been tested yet
    void markerUp() {
      // move marker up off whiteboard
      z_axis.write(70);

      // regrip marker (could've gotten loose
      grab();
    } // ..markerUp

    // move marker down - hasn't been tested yet
    void markerDown() {
      // move marker down onto whiteboard
      z_axis.write(0);

      // regrip marker (could've gotten loose
      grab();
    } // ..markerDown
};
// -- END OF FILE --
