// *** LIBRARIES ***
#include "gantry.h"

GANTRY drawing;

void setup() {
    // open serial monitor
    Serial.begin(115200);

    // initialize gantry
    drawing.init();
}

void loop() {
    // draw a straight line to the right by 100 mm
    delay(1000);
    drawing.takeMarkerFrom();
    delay(1000);
    drawing.putMarkerBack();
}