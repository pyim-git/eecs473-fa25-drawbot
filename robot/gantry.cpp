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

// ===== PRIVATE FUNCTIONS: =====
// move marker to the right (from robot's perspective)
// function takes in a distance in mm and speed in mm/s
void GANTRY::moveRight(float distance, float speed) {
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

  // add a little bit of delay to separate commands
  delay(waitTime);

  // regrip marker (could've gotten loose)
  grab();
  
  // update position value
  if (position + distance <= BELT_LENGTH) {
    position += distance;
  } // if ..position not hit zero
  
  else {
    position == BELT_LENGTH;
  } // else ..position hits leftmost
} // ..moveRight()

// move marker to the left (from robot's perspective)
// function takes in a distance in mm and speed in mm/s
void GANTRY::moveLeft(float distance, float speed) {
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

  // update position value
  if (position - distance >= 0) {
    position -= distance;
  } // if ..position not hit zero
  
  else {
    position == 0.0;
  } // else ..position hits leftmost
} // ..moveLeft()

// puts marker back into tool changing rack
// specify postion to put it back into
void GANTRY::putMarkerBack(int position) {
  // reset gantry position
  resetPos(0);

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

  // move tool rack back to initial position
  tool_change.write(90);
  delay(200);
} // putMarkerBack()

// takes marker out of tool changing rack
// specify postion to take it out of
void GANTRY::takeMarkerFrom(int position) {
  // reset gantry position
  resetPos(0);

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

  // move tool rack back to initial position
  tool_change.write(90);
  delay(200);
} // takeMarker()

// grab marker - change angle as needed
void GANTRY::grab() {
  // tighten gripper for servo
  gripper.write(10);
  delay(waitTime);
} // ..grab()

// release marker - change angle as needed
void GANTRY::release() {
  // loosen gripper for servo
  gripper.write(80);
  delay(waitTime);
} // ..release()

// ==== PUBLIC FUNCTIONS =====
// initialize gantry + drawing arm
// CALL THIS FUNCTION IN SETUP LOOP!
void GANTRY::init() {
  // start gantry belt stepper motor
  gantry.begin(11, 9); // change these!!!
  gantry.setDirection(DRV8825_CLOCK_WISE);

  // attach pins
  // change pins as needed
  /* FOR ESP32 PINOUT */
  z_axis.attach(47);       // attach z_axis servo to pin 47 (PWM1)
  gripper.attach(48);      // attach gripper servo to pin 48 (PWM2)
  tool_change.attach(7);   // attach tool change servo to pin 7

  /* FOR PROTOTYPE PINOUTS
  // CAN ONLY DO 2 PINS AT A TIME WITH ARDUINO
  z_axis.attach(10);     // attach z_axis servo to pin 9    
  gripper.attach(9);     // attach gripper servo to pin 10
  // tool_change.attach(10);  // attach tool change servo to pin 11
  */

  // initialize servos and steppers to initial positions
  resetPos(0);
  grab();    // clamp grippers - grabbing initial marker
  tool_change.write(90);   // reset position of tool change rack
}

// move gripper fully to the top of the gantry
  // orientation == 0 means robot is facing to the right
  // orientation == 1 means robot is facing to the left
void GANTRY::resetPos(int orientation) {
  // reset gantry position to left side
  if (orientation == 0) {   // normal orientation
    moveLeft(100, 50);
  } // if ..orientation is normal - facing right

  else {
    moveRight(100, 50);
  } // else ..orientation is reversed - facing left

  // wait a bit
  delay(waitTime);
}

// gantry moves to an "absolute" position that it needs to move to
void GANTRY::move(float pos, float speed) {
  // figure out where marker needs to move to
  float moving = position - pos;

  // if moving is positive, move to the left
  if (moving > 0) {
    moveLeft(moving, speed);
  } // if ..moveLeft

  // else moving is negative, move to the right
  else if (moving < 0) {
    moveRight(abs(moving), speed);
  } // else ..moveRight
} // ..move()

// ===== MARKER COMMANDS =====
// move marker up
void GANTRY::liftMarker() {
  // move marker up off whiteboard
  z_axis.write(90);
} // ..markerUp

// move marker down
void GANTRY::lowerMarker() {
  // move marker down onto whiteboard
  z_axis.write(110);
} // ..markerDown

// switches out the marker for the new color we want
void GANTRY::switchMarker(int colorPos1, int colorPos2) {
  // put back the marker currently in the gripper
  putMarkerBack(colorPos1);

  // take out the new marker we want
  takeMarkerFrom(colorPos2);
} // switchMarker()

// -- END OF FILE --

// ========================================================================
//                                  NOTES
// ========================================================================
/*
    * mess around with delay values for marker changing functions
*/