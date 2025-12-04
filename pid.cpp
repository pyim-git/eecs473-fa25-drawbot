// ==== LIBRARIES =====
#include "pid.h"

// ====== L298N MOTOR PINS ======
#define ENA_L 5
#define IN1_L 6
#define IN2_L 7
#define ENB_R 8
#define IN3_R 9
#define IN4_R 10

// ===== MOTOR SETUP =====
void setupMotors() {
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(IN3_R, OUTPUT);
  pinMode(IN4_R, OUTPUT);

  ledcAttach(ENA_L, 5000, 8);
  ledcAttach(ENB_R, 5000, 8);
} // ..setupMotors()

int remapPID(double pidOutput, bool rotatingInPlace) {
  const int MIN_PWM = 140;
  const int DEAD_PWM = 50;
  const int MAX_PWM = 255;
  const double S = 10.0;

  double scaledPID = pidOutput * 1;

  if (scaledPID < -atanh(MIN_PWM/255.0)*S || scaledPID > atanh(MIN_PWM/255.0)*S) {
    scaledPID = tanh(scaledPID/S)*255.0;
  } else if (scaledPID < -atanh(DEAD_PWM/255.0)*S) {
    scaledPID = -MIN_PWM;
  } else if (scaledPID > atanh(DEAD_PWM/255.0)*S) {
    scaledPID = MIN_PWM;
  } else {
    scaledPID = 0;
  }
  return (int)scaledPID;
} // ..remapPID()

void motorWrite(bool left, int pwmVal) {
  bool fwd = pwmVal >= 0;
  int pwm = constrain(abs(pwmVal), 0, 255);

  if (left) {
    digitalWrite(IN1_L, fwd ? HIGH : LOW);   
    digitalWrite(IN2_L, fwd ? LOW : HIGH);   
    ledcWrite(ENA_L, pwm);
  } else {
    digitalWrite(IN3_R, fwd ? HIGH : LOW);   
    digitalWrite(IN4_R, fwd ? LOW : HIGH);   
    ledcWrite(ENB_R, pwm);
  }
} // ..motorWrite()

void stopMotors() {
  ledcWrite(ENA_L, 0);
  ledcWrite(ENB_R, 0);
  digitalWrite(IN1_L, LOW);
  digitalWrite(IN2_L, LOW);
  digitalWrite(IN3_R, LOW);
  digitalWrite(IN4_R, LOW);
} // ..stopMotors()