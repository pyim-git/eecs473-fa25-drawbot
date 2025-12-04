// *** LIBRARIES ***
#include "gantry.h"

GANTRY drawing;

void setup() {
    // open serial monitor
    Serial.begin(115200);

    // initialize gantry
    drawing.init();
    Serial.print("Initialization Done!");
    Serial.println("Grabbed marker!");
}

void loop() {
}