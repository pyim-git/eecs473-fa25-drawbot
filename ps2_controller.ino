#include "PS2X_lib.h"  //for v1.6

PS2X ps2x; // create PS2 Controller Class
int error = 0; 
byte type = 0;
byte vibrate = 0;

void setup(){
  // *** START CONTROLLER INIT ***
  Serial.begin(115200);
  
  error = ps2x.config_gamepad(10,12,11,13, false, false);   //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  
  if (error == 0) {
    Serial.println("Found Controller, configured successful");
    Serial.println("Press X to turn on motors. Press O to turn off motors.");
    Serial.println("Holding START will print out the motor controls that are read from the stick.");
  }
   
  else if (error == 1) {
    Serial.println("No controller found, check wiring, see readme.txt to enable debug.");
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug.");
  }

  else if (error == 3) {
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
  }

  type = ps2x.readType(); 
    
  switch(type) {
    case 0:
      Serial.println("Unknown Controller type");
      break;
    case 1:
      Serial.println("DualShock Controller Found");
      break;
    case 2:
      // Serial.println("GuitarHero Controller Found");
      break;
    default:
      break;
  }

  // *** END CONTROLLER INIT ***
}

void loop(){
  /* You must Read Gamepad to get new values
  Read GamePad and set vibration values
  ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
  if you don't enable the rumble, use ps2x.read_gamepad(); with no values
  
  you should call this at least once a second
  */
  
  ps2x.read_gamepad(false, vibrate);          //read controller and set large motor to spin at 'vibrate' speed

  /* STICK CONTROLS DEBUG!!! */
  if (ps2x.Button(PSB_START)) {     // Hold down START button to read left stick movements
    if (ps2x.Analog(PSS_LX) == 127 && ps2x.Analog(PSS_LY) == 128) {
      Serial.println("Nothing to see here...");
      // stop robot in place
      stop();
    } // if ..LEFT STICK in idle position

    else if (ps2x.Analog(PSS_LX) > 127 && ps2x.Analog(PSS_LY) == 128) {  // RIGHT
      Serial.println("Moving right!");
      // move robot right
      right();
    } // elif ..LEFT STICK pointing right

    else if (ps2x.Analog(PSS_LX) < 127 && ps2x.Analog(PSS_LY) == 128) { // LEFT
      Serial.println("Moving left!");
      // move robot left
      left();
    } // elif ..LEFT STICK pointing left

    else if (ps2x.Analog(PSS_LX) == 127 && ps2x.Analog(PSS_LY) < 128) { // FORWARD
      Serial.println("Moving forward!");
      // move robot forwards
      forward();
    } // elif ..LEFT STICK pointing up

    else if (ps2x.Analog(PSS_LX) == 127 && ps2x.Analog(PSS_LY) > 128) { // BACKWARD
      Serial.println("Moving backward!");
      // move robot backwards
      backwards();
    } // elif ..LEFT STICK pointing down
  } // if ..START button is being held

  /* STICK CONTROLS!!! */
  if (ps2x.Analog(PSS_LX) == 127 && ps2x.Analog(PSS_LY) == 128) {
    // stop robot in place
    stop();
  } // if ..LEFT STICK in idle position

  else if (ps2x.Analog(PSS_LX) > 127 && ps2x.Analog(PSS_LY) == 128) {  // RIGHT
    // move robot right
    right();
  } // elif ..LEFT STICK pointing right

  else if (ps2x.Analog(PSS_LX) < 127 && ps2x.Analog(PSS_LY) == 128) { // LEFT
    // move robot left
    left();
  } // elif ..LEFT STICK pointing left

  else if (ps2x.Analog(PSS_LX) == 127 && ps2x.Analog(PSS_LY) < 128) { // FORWARD
    // move robot forwards
    forward();
  } // elif ..LEFT STICK pointing up

  else if (ps2x.Analog(PSS_LX) == 127 && ps2x.Analog(PSS_LY) > 128) { // BACKWARD
    // move robot backwards
    backwards();
  } // elif ..LEFT STICK pointing down
  // END. STICK CONTROLS

  // MISC BUTTONS
  // stick controls debug (analog values)
  if (ps2x.Button(PSB_SELECT)) {
    // PRINT VALUES FOR DEBUGGING
    Serial.print(ps2x.Analog(PSS_LX));
    Serial.print(", ");
    Serial.println(ps2x.Analog((PSS_LY)));
  }

  /* BUTTON CONTROLS!!! */
  if (ps2x.ButtonPressed(PSB_RED)) {
    Serial.println("*** STOP! ***");
    // disable robot motors
    // disableMotors()
  } // if ..O button pressed

  if (ps2x.ButtonPressed(PSB_BLUE)) {
    Serial.println("*** GO! ***");
    // enable robot motors
    // enableMotors();
  } // if ..X pressed

  if (ps2x.ButtonPressed(PSB_L1)) {
    Serial.println("Spinning counter clockwise!");
    // Rotate robot counter clockwise
    // rotate(0);
  } // if ..LEFT trigger pressed

  if (ps2x.ButtonPressed(PSB_R1)) {
    Serial.println("Spinning clockwise!");
    // Rotate robot clockwise
    // rotate(1);
  } // if ..RIGHT trigger pressed

  // delayyyyy
  delay(50);
}