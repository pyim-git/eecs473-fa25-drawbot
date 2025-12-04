// ==== LIBRARIES =====
#include <Arduino.h>
#include <string>
#include <math.h>

// ====== MOTOR SETUP ======
void setupMotors(); // ..setupMotors()

// Remap PID output to skip deadzone
int remapPID(double pidOutput, bool rotatingInPlace);

void motorWrite(bool left, int pwmVal);

void stopMotors();