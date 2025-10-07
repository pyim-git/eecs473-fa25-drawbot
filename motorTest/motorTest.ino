// #include "StepperControl.h"
#include "PS2X_lib.h"  //for v1.6 - ps2 controller
#include "DRV8825.h"        // library for specific motor driver
#include "MultiDriver.h"    // controlling multiple stepping motors

#define DIR_PIN_1  3  // Direction pin for motor 1
#define STEP_PIN_1 2  // Step pin for motor 1
#define DIR_PIN_2  5  // Direction pin for motor 2
#define STEP_PIN_2 4  // Step pin for motor 2
#define DIR_PIN_3  7  // Direction pin for motor 3
#define STEP_PIN_3 6  // Step pin for motor 3
#define DIR_PIN_4  9  // Direction pin for motor 4
#define STEP_PIN_4 8  // Step pin for motor 4

#define MS0 0
#define MS1 1
#define MS2 A1

/*
This is the arduino motor control script for controlling 4 DC motors with an ESP32 microcontroller.
It is designed to work with a motor driver that uses 8 GPIO pins to control 4 motors (each motor requires 2 pins for direction and speed control).
*/
// StepperControl motorControl(DIR_PIN_1, STEP_PIN_1, DIR_PIN_2, STEP_PIN_2, DIR_PIN_3, STEP_PIN_3, DIR_PIN_4, STEP_PIN_4);
DRV8825 motor4(200,9,8,MS0,MS1,MS2);
DRV8825 motor3(200,7,6,MS0,MS1,MS2);
DRV8825 motor2(200,5,4,MS0,MS1,MS2);
DRV8825 motor1(200,3,2,MS0,MS1,MS2);

// motor driver object
MultiDriver drivers(motor1, motor2, motor3, motor4);

// driver enable pins
#define EN1 A2
#define EN2 A3
#define EN3 A4
#define EN4 A5

// PS2 CONTROLER VARIABLES
PS2X ps2x; // create PS2 Controller Class
int error = 0;
byte type = 0;
byte vibrate = 0;

int speed = 200;

void setup()
{
    drivers.begin(200, 1);
    // drivers.setMicrostep(1600);

    pinMode(EN1, OUTPUT);
    pinMode(EN2, OUTPUT);
    pinMode(EN3, OUTPUT);
    pinMode(EN4, OUTPUT);

    // *** START CONTROLLER INIT ***
    Serial.begin(115200);

    Serial.println("Start");
    
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
            Serial.println("GuitarHero Controller Found");
            break;
        default:
            break;
    }
    // *** END CONTROLLER INIT ***
}

void loop()
{
    // *** START CONTROLLER CONTROLS ***
    /* You must Read Gamepad to get new values
    Read GamePad and set vibration values
    ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
    if you don't enable the rumble, use ps2x.read_gamepad(); with no values
    
    you should call this at least once a second
    */
    /**/
    ps2x.read_gamepad(false, vibrate);          //read controller and set large motor to spin at 'vibrate' speed

    // when not reading anything, turn off drivers
    digitalWrite(EN1, HIGH);
    digitalWrite(EN2, HIGH);
    digitalWrite(EN3, HIGH);
    digitalWrite(EN4, HIGH);

    // STICK CONTROLS DEBUG!!!
    if (ps2x.Button(PSB_START)) {     // Hold down START button to read left stick movements
        if (ps2x.Analog(PSS_LX) == 127 && ps2x.Analog(PSS_LY) == 128) {
            Serial.println("Nothing to see here...");
        } // if ..LEFT STICK in idle position

        else if (ps2x.Analog(PSS_LX) > 127 && ps2x.Analog(PSS_LY) == 128) {  // RIGHT
            Serial.println("Moving right!");
        } // elif ..LEFT STICK pointing right

        else if (ps2x.Analog(PSS_LX) < 127 && ps2x.Analog(PSS_LY) == 128) { // LEFT
            Serial.println("Moving left!");
        } // elif ..LEFT STICK pointing left

        else if (ps2x.Analog(PSS_LX) == 127 && ps2x.Analog(PSS_LY) < 128) { // FORWARD
            Serial.println("Moving forward!");
        } // elif ..LEFT STICK pointing up

        else if (ps2x.Analog(PSS_LX) == 127 && ps2x.Analog(PSS_LY) > 128) { // BACKWARD
            Serial.println("Moving backward!");
        } // elif ..LEFT STICK pointing down
    } // if ..START button is being held

    // STICK CONTROLS!!!
    if (ps2x.Analog(PSS_LX) == 127 && ps2x.Analog(PSS_LY) == 128) {
        // stop robot in place
        // enables stay off
        drivers.stop();
    } // if ..LEFT STICK in idle position

    else if (ps2x.Analog(PSS_LX) > 127 && ps2x.Analog(PSS_LY) == 128) {  // RIGHT
        // move robot right
        digitalWrite(EN1, LOW);
        digitalWrite(EN3, LOW);
        drivers.move(-1, 0, -1, 0);
    } // elif ..LEFT STICK pointing right

    else if (ps2x.Analog(PSS_LX) < 127 && ps2x.Analog(PSS_LY) == 128) { // LEFT
        // move robot left
        digitalWrite(EN2, LOW);
        digitalWrite(EN4, LOW);
        drivers.move(0, 1, 0, 1);
    } // elif ..LEFT STICK pointing left

    else if (ps2x.Analog(PSS_LX) == 127 && ps2x.Analog(PSS_LY) < 128) { // FORWARD
        // move robot forwards
        digitalWrite(EN1, LOW);
        digitalWrite(EN2, LOW);
        digitalWrite(EN3, LOW);
        digitalWrite(EN4, LOW);
        drivers.move(-1, 1, -1, 1);
    } // elif ..LEFT STICK pointing up

    else if (ps2x.Analog(PSS_LX) == 127 && ps2x.Analog(PSS_LY) > 128) { // BACKWARD
        // move robot backwards
        digitalWrite(EN1, LOW);
        digitalWrite(EN2, LOW);
        digitalWrite(EN3, LOW);
        digitalWrite(EN4, LOW);
        drivers.move(1, -1, 1, -1);
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

    // BUTTON CONTROLS!!!
    if (ps2x.ButtonPressed(PSB_GREEN)) {
        Serial.println("Speed up!");
        Serial.println(speed);
        // increment rpm
        speed += 20;
        drivers.begin(speed, 1);
    } // if ..X pressed

    if (ps2x.ButtonPressed(PSB_BLUE)) {
        Serial.println("Slow down!");
        Serial.println(speed);
        // decrement rpm
        speed -= 20;
        drivers.begin(speed, 1);
    }

    // if (ps2x.ButtonPressed(PSB_L1)) {
    //     Serial.println("Spinning counter clockwise!");
    //     // TODO: Rotate robot counter clockwise
    // } // if ..LEFT trigger pressed

    // if (ps2x.ButtonPressed(PSB_R1)) {
    //     Serial.println("Spinning clockwise!");
    //     // TODO: Rotate robot clockwise
    // } // if ..RIGHT trigger pressed

    // delayyyyy
    delay(10);

    /**/
    // *** END CONTROLLER LOOP ***
}


