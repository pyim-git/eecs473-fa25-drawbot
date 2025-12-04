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
  // TESTING LOOP FOR STUFF YAY
  // drawing.lowerMarker();
  // drawing.move(0, 50);
  // delay(3100);
  // // drawing.liftMarker();
  // drawing.move(100, 50);
  // delay(3100);

  // switching marker test

  drawing.release();
  delay(2000);
  drawing.tool_change.write(0);
  delay(2000);
  drawing.grab();
  delay(2000);
  drawing.tool_change.write(140);
  delay(2000);
}