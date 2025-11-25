// *** LIBRARIES ***
#include "gantry.h"

// NOTE: IF NOT USING .H FILE - COPY THESE OTHER .H FILES INTO TOP
// EVERYTHING SHOULD WORK AND BE USUABLE
// #include "DRV8825.h"  // DRV8825 library for stepper motor control
// #include "ESP32Servo.h"    // servo library for ESP32

// ========================
// ***** GANTRY CLASS *****
// ========================
// *** FUNCTIONS! ***
// NOTE: with the current stepper motor resolution (1.8˚) and the radius of the pulley wheel (6mm), 
// each step is equivalent to moving 0.188 mm
  // this can change with microstepping

// initialize gantry + drawing arm
// CALL THIS FUNCTION IN SETUP LOOP!
void GANTRY::init() {
  // start gantry belt stepper motor
  gantry.begin(4, 5);
  // gantry.enable()     // enable gantry
  // gantry.sleep()      // set gantry to sleep - save power
  gantry.setDirection(DRV8825_CLOCK_WISE);

  // attach pins
  // change pins as needed
  /* FOR ESP32 PINOUTS */
  z_axis.attach(47);       // attach z_axis servo to pin 47 (PWM1)
  gripper.attach(48);      // attach gripper servo to pin 48 (PWM2)
  tool_change.attach(7);   // attach tool change servo to pin 7

  /* FOR PROTOTYPE PINOUTS
  z_axis.attach(11);       // attach z_axis servo to pin 9
  gripper.attach(9);     // attach gripper servo to pin 10
  tool_change.attach(10);  // attach tool change servo to pin 11
  */

  // initialize servos and steppers to initial positions
  resetPos();
  markerUp();  // reset marker to up position (not touching board)
  grab();     // tighten grippers
  tool_change.write(50);   // reset position of tool change rack
}

// move marker to the right (from robot's perspective)
// function takes in a distance in mm and speed in mm/s
void GANTRY::moveRight(double distance, double speed) {
  // wake up gantry - only allowed if sleep pin set
  // gantry.wakup();
  
  // set direction to be clockwise (moving right)
  gantry.setDirection(DRV8825_CLOCK_WISE);

  // Calculate number of steps
  int num_steps = distance/step_dist;

  // calculate delay needed for specific speed
  wait = 1000*step_dist / speed;

  // Serial.println("Moving gantry arm right!");
  for (int i = 0; i < num_steps; i++) {
    gantry.step();
    delay(wait);
  } // for ..move number of required steps

  // put gantry to sleep - only allowed if sleep pin set
  // gantry.sleep();

  // add a little bit of delay to separate commands
  delay(waitTime);

  // regrip marker (could've gotten loose)
  grab();
} // ..moveRight()

// move marker to the left (from robot's perspective)
// function takes in a distance in mm and speed in mm/s
void GANTRY::moveLeft(double distance, double speed) {
  // wake up gantry - only allowed if sleep pin set
  // gantry.wakup();

  // set direction to be counterclockwise (moving left)
  gantry.setDirection(DRV8825_COUNTERCLOCK_WISE);

  // Calculate number of steps
  int num_steps = distance/step_dist;

  // calculate delay needed for specific speed
  wait = 1000*step_dist / speed;

  // Serial.println("Moving gantry arm left!");
  for (int i = 0; i < num_steps; i++) {
    gantry.step();
    delay(wait);
  } // for ..move number of required steps
  
  // put gantry to sleep - only allowed if sleep pin set
  // gantry.sleep();
  
  // add a little bit of delay to separate commands
  delay(waitTime);

  // regrip marker (could've gotten loose)
  grab();
} // ..moveLeft()

// move gripper fully to the left side of the gantry
void resetPos() {
  // reset gantry position to left side
  moveLeft(100, 50);
  delay(waitTime);
}

// *** MARKER COMMANDS ***
// grab marker - change angle as needed
void GANTRY::grab() {
  // tighten gripper for servo
  gripper.write(180);
  delay(waitTime);
} // ..grab()

// release marker - change angle as needed
void GANTRY::release() {
  // loosen gripper for servo
  gripper.write(60);
  delay(waitTime);
}

// move marker up
void GANTRY::markerUp() {
  // move marker up off whiteboard
  z_axis.write(70);
  // add a little bit of delay to separate commands
  delay(waitTime);

  // regrip marker (could've gotten loose
  grab();
} // ..markerUp

// move marker down
void GANTRY::markerDown() {
  // move marker down onto whiteboard
  z_axis.write(0);
  // add a little bit of delay to separate commands
  delay(waitTime);

  // regrip marker (could've gotten loose
  grab();
} // ..markerDown

// puts marker back into tool changing rack
// specify postion to put it back into
void GANTRY::putMarkerBack(int position) {
  // reset gantry position
  resetPos();

  // move gripper to position you want
  // CHANGE THIS WHEN TESTING TO GET BEST POSITION
  if (position == 2) { moveRight(50, 50); }
  else if (position == 3) { moveRight(100, 50); }

  // marker currently in gripper
  // have tool rack grab marker
  tool_change.write(0);
  delay(200);

  // release marker
  release();
  delay(300);

  // move tool rack back to initial position
  tool_change.write(40);
  delay(200);
} // placeMarker()

// takes marker out of tool changing rack
// specify postion to take it out of
void GANTRY::takeMarkerFrom(int position) {
  // reset gantry position
  resetPos();

  // move gripper to position you want
  // CHANGE THIS WHEN TESTING TO GET BEST POSITION
  if (position == 2) { moveRight(50, 50); }
  else if (position == 3) { moveRight(100, 50); }

  // marker currently not in gripper
  // have tool rack give gripper a marker
  tool_change.write(0);
  delay(200);

  // grab marker
  grab();
  delay(300);

  // move tool rack back to initial position
  tool_change.write(40);
  delay(200);
} // takeMarker()

// -- END OF FILE --
