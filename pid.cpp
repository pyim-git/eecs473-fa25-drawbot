// ==== LIBRARIES =====
#include "pid.h"

// ====== encoder PINS ======
#define ENC_R_A 18
#define ENC_R_B 8
#define ENC_L_A 40
#define ENC_L_B 39

// ====== L298N MOTOR PINS ======
#define IN1_R 16
#define IN2_R 17
#define IN3_L 41
#define IN4_L 42
// ====== MOTOR SETUP ======
void setupMotors() {
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  pinMode(IN3_L, OUTPUT);
  pinMode(IN4_L, OUTPUT);

  ledcAttach(IN1_R, 10000, 8);
  ledcAttach(IN2_R, 10000, 8);
  ledcAttach(IN3_L, 10000, 8);
  ledcAttach(IN4_L, 10000, 8);
}

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
    ledcWrite(IN3_L, fwd ? 0 : pwm);   // reverse
    ledcWrite(IN4_L, fwd ? pwm : 0);   // forward
  } else {
    ledcWrite(IN1_R, fwd ? pwm : 0);
    ledcWrite(IN2_R, fwd ? 0 : pwm);
  }
}

void stopMotors() {
    ledcWrite(IN1_R, 255);
    ledcWrite(IN2_R, 255);
    ledcWrite(IN3_L, 255);
    ledcWrite(IN4_L, 255);
}