// *** LIBRARIES ***
#include "gantry.cpp"

GANTRY drawing;

void setup() {
    // open serial monitor
    Serial.begin(115200);

    // initialize gantry
    drawing.init();
}

void loop() {
    // draw a straight line to the right by 100 mm
    grab(); // grab marker
    delay(10);
    markerDown();
    moveRight(100, 100);
    delay(10);
    // reset position - don't draw now
    markerUp();
    moveLeft(100, 100);

    // wait one full second
    delay(1000);
}