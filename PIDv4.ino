#include <Arduino.h>
#include <ESP32Encoder.h>
#include <math.h>
#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"

// ====== L298N MOTOR PINS ======
#define ENA_L 5
#define IN1_L 6
#define IN2_L 7
#define ENB_R 8
#define IN3_R 9
#define IN4_R 10

// ====== ENCODER PINS ======
#define ENC_L_A 18
#define ENC_L_B 12
#define ENC_R_A 13
#define ENC_R_B 14

// ====== I2C PINS ======
#define CUSTOM_SDA 15  
#define CUSTOM_SCL 16  

// ====== ROBOT PARAMETERS ======
#define WHEEL_RADIUS 0.0358  // 3.58 cm
#define WHEEL_BASE   0.246063    // 24.6 cm
#define TICKS_PER_REV 8400.0
#define LOOP_DT_MS 20

// ====== ENCODER DIRECTION ======
#define ENC_L_DIR 1   // Try -1 if left encoder backwards
#define ENC_R_DIR 1   // Try -1 if right encoder backwards

// ====== TEST MODE ======
#define ENCODER_TEST_MODE false  // SET TO false WHEN ENCODERS WORK
#define MOTOR_DIRECTION_TEST false  // SET TO true TO TEST MOTOR DIRECTIONS

// ====== CONTROL VARIABLES ======
ESP32Encoder encLeft, encRight;
QwiicOTOS myOtos;

// PID Variables - NO LIBRARY NEEDED
double set_L = 0.0, input_L = 0.0, output_L = 0.0;
double set_R = 0.0, input_R = 0.0, output_R = 0.0;

// Odometry
volatile double x = 0, y = 0, theta = M_PI / 2.0;

// Target position & orientation
int c = 0;
double setpointx[4] = {0.1, 0.2, 0.2, 0.0};
double setpointy[4] = {0.0, 0.2, 0.0, 0.0};
double setpointtheta[4] = {0.0, 0.0, -M_PI/2, M_PI};

volatile double x_s = setpointx[c];
volatile double y_s = setpointy[c];
volatile double theta_s = setpointtheta[c];

// Velocity filtering
static double filteredL = 0, filteredR = 0;

// Global rotation flag
volatile bool isRotatingInPlace = false;

// ====== UTILITY FUNCTIONS ======
double wrapAngle(double angle) {
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

// ====== MOTOR DIRECTION TEST ======
void testMotorDirections() {
  Serial.println("\n╔════════════════════════════════╗");
  Serial.println("║   MOTOR DIRECTION TEST         ║");
  Serial.println("╚════════════════════════════════╝\n");
  
  setupMotors();
  
  Serial.println("Test 1: Left motor FORWARD (+150 PWM)");
  Serial.println("Expected: Left wheel rotates forward");
  motorWrite(true, 150);
  delay(2000);
  stopMotors();
  delay(1000);
  
  Serial.println("\nTest 2: Left motor BACKWARD (-150 PWM)");
  Serial.println("Expected: Left wheel rotates backward");
  motorWrite(true, -150);
  delay(2000);
  stopMotors();
  delay(1000);
  
  Serial.println("\nTest 3: Right motor FORWARD (+150 PWM)");
  Serial.println("Expected: Right wheel rotates forward");
  motorWrite(false, 150);
  delay(2000);
  stopMotors();
  delay(1000);
  
  Serial.println("\nTest 4: Right motor BACKWARD (-150 PWM)");
  Serial.println("Expected: Right wheel rotates backward");
  motorWrite(false, -150);
  delay(2000);
  stopMotors();
  delay(1000);
  
  Serial.println("\nTest 5: CCW Rotation (Left=-150, Right=+150)");
  Serial.println("Expected: Robot rotates counterclockwise (left)");
  motorWrite(true, -150);
  motorWrite(false, 150);
  delay(2000);
  stopMotors();
  delay(1000);
  
  Serial.println("\nTest 6: CW Rotation (Left=+150, Right=-150)");
  Serial.println("Expected: Robot rotates clockwise (right)");
  motorWrite(true, 150);
  motorWrite(false, -150);
  delay(2000);
  stopMotors();
  
  Serial.println("\n╔════════════════════════════════╗");
  Serial.println("║   TEST COMPLETE                ║");
  Serial.println("╚════════════════════════════════╝");
  Serial.println("\nIf any test failed, adjust motorWrite() function.");
  Serial.println("System halted.");
  while(1) delay(1000);
}

// ====== MOTOR SETUP ======
void setupMotors() {
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(IN3_R, OUTPUT);
  pinMode(IN4_R, OUTPUT);

  ledcAttach(ENA_L, 5000, 8);
  ledcAttach(ENB_R, 5000, 8);
}

// Remap PID output to skip deadzone
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
}

void SerialCommandTask(void *pv) {
  while (true) {
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();

      if (cmd.length() > 0 && (cmd.charAt(0) == 'S' || cmd.charAt(0) == 's')) {
        double newX, newY, thetaDeg;
        int count = sscanf(cmd.c_str() + 1, "%lf %lf %lf", &newX, &newY, &thetaDeg);

        if (count == 3) {
          x_s = newX;
          y_s = newY;
          theta_s = wrapAngle(thetaDeg * M_PI / 180.0);
          
          Serial.printf("✅ New setpoint: X=%.3f m, Y=%.3f m, Theta=%.2f° (%.3f rad)\n",
                        x_s, y_s, thetaDeg, theta_s);
        } else {
          Serial.println("❌ Format error! Use: S <x> <y> <thetaDeg>");
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

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
}

void stopMotors() {
  ledcWrite(ENA_L, 0);
  ledcWrite(ENB_R, 0);
  digitalWrite(IN1_L, LOW);
  digitalWrite(IN2_L, LOW);
  digitalWrite(IN3_R, LOW);
  digitalWrite(IN4_R, LOW);
}

// ====== ENCODER TEST ======
void testEncoders() {
  Serial.println("\n╔════════════════════════════════╗");
  Serial.println("║    ENCODER DIAGNOSTIC TEST     ║");
  Serial.println("╚════════════════════════════════╝\n");
  
  setupMotors();
  stopMotors();
  
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encLeft.attachHalfQuad(ENC_L_A, ENC_L_B);
  encRight.attachHalfQuad(ENC_R_A, ENC_R_B);
  
  encLeft.clearCount();
  encRight.clearCount();
  
  delay(500);
  
  Serial.println("📌 Pin Configuration:");
  Serial.printf("   Left Motor:  ENA=%d, IN1=%d, IN2=%d\n", ENA_L, IN1_L, IN2_L);
  Serial.printf("   Right Motor: ENB=%d, IN3=%d, IN4=%d\n", ENB_R, IN3_R, IN4_R);
  Serial.printf("   Left Encoder:  A=%d, B=%d (DIR=%+d)\n", ENC_L_A, ENC_L_B, ENC_L_DIR);
  Serial.printf("   Right Encoder: A=%d, B=%d (DIR=%+d)\n\n", ENC_R_A, ENC_R_B, ENC_R_DIR);
  
  // Test 1: LEFT MOTOR
  Serial.println("🔄 TEST 1: LEFT MOTOR (3 seconds)");
  Serial.println("   Expected: Encoder count should increase\n");
  
  digitalWrite(IN1_L, HIGH);
  digitalWrite(IN2_L, LOW);
  ledcWrite(ENA_L, 180);
  
  long lastCountL = 0;
  for (int i = 0; i < 30; i++) {
    long rawCount = encLeft.getCount();
    long currCount = rawCount * ENC_L_DIR;
    long delta = currCount - lastCountL;
    Serial.printf("   t=%0.1fs | Raw: %6ld | Corrected: %6ld | Delta: %+4ld ", 
                  i*0.1, rawCount, currCount, delta);
    
    if (currCount != 0) {
      Serial.println("✓");
    } else if (i > 5) {
      Serial.println("✗ PROBLEM!");
    } else {
      Serial.println("");
    }
    
    lastCountL = currCount;
    delay(100);
  }
  
  stopMotors();
  long finalL = encLeft.getCount() * ENC_L_DIR;
  
  Serial.println("\n   Result:");
  if (finalL > 100) {
    Serial.printf("   ✅ LEFT ENCODER WORKING (count: %ld)\n", finalL);
    if (finalL < 0) {
      Serial.println("   ⚠️  Count is negative - set ENC_L_DIR = -1");
    }
  } else {
    Serial.printf("   ❌ LEFT ENCODER FAILED (count: %ld)\n", finalL);
    Serial.println("   Possible issues:");
    Serial.println("      • Encoder not connected to GPIO pins");
    Serial.println("      • Wrong pin numbers in code");
    Serial.println("      • Encoder needs 5V power (not 3.3V)");
    Serial.println("      • Wiring: Check A, B, VCC, GND");
  }
  Serial.println();
  
  delay(1000);
  encRight.clearCount();
  
  // Test 2: RIGHT MOTOR
  Serial.println("🔄 TEST 2: RIGHT MOTOR (3 seconds)");
  Serial.println("   Expected: Encoder count should increase\n");
  
  digitalWrite(IN3_R, HIGH);
  digitalWrite(IN4_R, LOW);
  ledcWrite(ENB_R, 180);
  
  long lastCountR = 0;
  for (int i = 0; i < 30; i++) {
    long rawCount = encRight.getCount();
    long currCount = rawCount * ENC_R_DIR;
    long delta = currCount - lastCountR;
    Serial.printf("   t=%0.1fs | Raw: %6ld | Corrected: %6ld | Delta: %+4ld ", 
                  i*0.1, rawCount, currCount, delta);
    
    if (currCount != 0) {
      Serial.println("✓");
    } else if (i > 5) {
      Serial.println("✗ PROBLEM!");
    } else {
      Serial.println("");
    }
    
    lastCountR = currCount;
    delay(100);
  }
  
  stopMotors();
  long finalR = encRight.getCount() * ENC_R_DIR;
  
  Serial.println("\n   Result:");
  if (finalR > 100) {
    Serial.printf("   ✅ RIGHT ENCODER WORKING (count: %ld)\n", finalR);
    if (finalR < 0) {
      Serial.println("   ⚠️  Count is negative - set ENC_R_DIR = -1");
    }
  } else {
    Serial.printf("   ❌ RIGHT ENCODER FAILED (count: %ld)\n", finalR);
    Serial.println("   Possible issues:");
    Serial.println("      • Encoder not connected to GPIO pins");
    Serial.println("      • Wrong pin numbers in code");
    Serial.println("      • Encoder needs 5V power (not 3.3V)");
    Serial.println("      • Wiring: Check A, B, VCC, GND");
  }
  Serial.println();
  
  // Final Summary
  Serial.println("╔════════════════════════════════╗");
  Serial.println("║         TEST SUMMARY           ║");
  Serial.println("╚════════════════════════════════╝");
  Serial.printf("Left Encoder:  %s (count: %ld)\n", finalL > 100 ? "✅ PASS" : "❌ FAIL", finalL);
  Serial.printf("Right Encoder: %s (count: %ld)\n", finalR > 100 ? "✅ PASS" : "❌ FAIL", finalR);
  
  if (finalL > 100 && finalR > 100) {
    Serial.println("\n🎉 Both encoders working!");
    if (finalL < 0 || finalR < 0) {
      Serial.println("⚠️  Check encoder directions and adjust ENC_L_DIR/ENC_R_DIR");
    }
    Serial.println("   Set ENCODER_TEST_MODE = false to enable navigation\n");
  } else {
    Serial.println("\n⚠️  Fix encoder issues before running navigation\n");
  }
  
  Serial.println("Test complete. System halted.");
  while(1) {
    delay(1000);
  }
}

// ====== ENCODER + ODOMETRY ======
void encoderTask(void *pv) {
  static long prevL = 0, prevR = 0;
  const double alpha_filter = 0.1;
  const double dt = LOOP_DT_MS / 1000.0;
  
  // Drift compensation variables for theta
  static double lastTheta = 0;
  static unsigned long stillStartTime = 0;
  static bool wasStill = false;
  const double ANGULAR_DRIFT_THRESHOLD = 0.005;  // rad/s (~0.03 deg/s)
  const unsigned long STILL_TIME_MS = 100;  // Time to confirm robot is still
  
  for (;;) {
    long currL = encLeft.getCount() * ENC_L_DIR;
    long currR = encRight.getCount() * ENC_R_DIR;
    long dL = currL - prevL;
    long dR = currR - prevR;
    prevL = currL;
    prevR = currR;

    // Calculate distances traveled by each wheel
    double distL = 2 * M_PI * WHEEL_RADIUS * (dL / TICKS_PER_REV);
    double distR = 2 * M_PI * WHEEL_RADIUS * (dR / TICKS_PER_REV);

    // Calculate center distance and change in angle from encoders
    double distCenter = (distL + distR) / 2.0;
    double deltaTheta = (distR - distL) / WHEEL_BASE;

    // Get theta from OTOS sensor only
    sfe_otos_pose2d_t myPosition;
    myOtos.getPosition(myPosition);
    double rawTheta = myPosition.h * 1000 / 57296;
    
    // Calculate angular velocity for drift detection
    double angularVel = fabs((rawTheta - lastTheta) / dt);
    
    // Detect if robot is stationary
    bool isStill = (fabs(filteredL) < 0.01 && fabs(filteredR) < 0.01 && 
                    angularVel < ANGULAR_DRIFT_THRESHOLD);
    
    if (isStill) {
      if (!wasStill) {
        stillStartTime = millis();
        wasStill = true;
      } else if (millis() - stillStartTime > STILL_TIME_MS) {
        // Robot confirmed still - freeze angle to prevent drift
        // (Don't update theta, keep last known good value)
      } else {
        // Transitioning to still - update theta normally
        theta = rawTheta;
      }
    } else {
      // Robot is moving - update theta normally from OTOS
      theta = rawTheta;
      wasStill = false;
    }
    
    lastTheta = rawTheta;

    // Update x and y using encoder odometry with current theta from OTOS
    x += distCenter * cos(theta);
    y += distCenter * sin(theta);

    // Calculate velocities with low-pass filter
    double rawL = (dL / TICKS_PER_REV) / dt;
    double rawR = (dR / TICKS_PER_REV) / dt;
    
    filteredL = alpha_filter * rawL + (1.0 - alpha_filter) * filteredL;
    filteredR = alpha_filter * rawR + (1.0 - alpha_filter) * filteredR;
    
    input_L = filteredL;
    input_R = filteredR;

    vTaskDelay(pdMS_TO_TICKS(LOOP_DT_MS));
  }
}

// ====== MANUAL PID CONTROLLER ======
void pidTask(void *pv) {
  // Wait for encoder task to start
  vTaskDelay(pdMS_TO_TICKS(200));
  
  // === Manual PID State Variables ===
  double integralL = 0, integralR = 0;
  double lastErrorL = 0, lastErrorR = 0;
  
  // === PID Gains ===
  const double Kp = 100.0;
  const double Ki = 1.0;
  const double Kd = 5.0;
  const double dt = LOOP_DT_MS / 1000.0;
  
  // === Anti-windup limits ===
  const double maxIntegral = 25.5;
  
  static bool wasAtGoal = false;
  
  Serial.println("✅ Manual PID controller started");
  Serial.printf("   Kp=%.1f, Ki=%.1f, Kd=%.1f, dt=%.3fs\n", Kp, Ki, Kd, dt);

  for (;;) {
    // Reset integral if we reach goal (anti-windup)
    bool atGoal = (fabs(set_L) < 0.001 && fabs(set_R) < 0.001);
    if (atGoal && !wasAtGoal) {
      integralL = 0;
      integralR = 0;
      lastErrorL = 0;
      lastErrorR = 0;
      output_L = 0;
      output_R = 0;
      Serial.println("🛑 PID RESET - Clearing integral windup");
    }
    wasAtGoal = atGoal;
    
    // === LEFT WHEEL PID ===
    double errorL = set_L - input_L;
    integralL += errorL * dt;
    integralL = constrain(integralL, -maxIntegral, maxIntegral);
    double derivativeL = (errorL - lastErrorL) / dt;
    output_L = Kp * errorL + Ki * integralL + Kd * derivativeL;
    output_L = constrain(output_L, -255.0, 255.0);
    lastErrorL = errorL;
    
    // === RIGHT WHEEL PID ===
    double errorR = set_R - input_R;
    integralR += errorR * dt;
    integralR = constrain(integralR, -maxIntegral, maxIntegral);
    double derivativeR = (errorR - lastErrorR) / dt;
    output_R = Kp * errorR + Ki * integralR + Kd * derivativeR;
    output_R = constrain(output_R, -255.0, 255.0);
    lastErrorR = errorR;
    
    // === Debug Output ===
    Serial.printf("PID: L[set=%.3f cur=%.3f err=%.3f out=%.1f] R[set=%.3f cur=%.3f err=%.3f out=%.1f]\n",
                  set_L, input_L, errorL, output_L,
                  set_R, input_R, errorR, output_R);

    // === Apply PWM ===
    int pwmL = remapPID(output_L, isRotatingInPlace);
    int pwmR = remapPID(output_R, isRotatingInPlace);
    
    Serial.printf("PWM: LEFT=%d RIGHT=%d\n\n", pwmL, pwmR);

    motorWrite(true, pwmL);
    motorWrite(false, pwmR);

    vTaskDelay(pdMS_TO_TICKS(LOOP_DT_MS));
  }
}

// ====== NAVIGATION TASK (WITH BACKWARD MOTION) ======
void navigationTask(void *pv) {
    enum State {
        DECIDE_MOTION,      // NEW: Decide whether to go forward or backward
        DRIVE_TO_GOAL,      // Drive forward or backward to goal
        ROTATE_TO_FINAL,    // Rotate to final orientation
        GOAL_REACHED        // Hold position
    };
    
    State currentState = DECIDE_MOTION;

    // Control gains
    const double Kp_rho   = 0.7;
    const double Kp_alpha = 5.0;
    const double Kp_theta = 1.0;

    // Tolerances
    const double goal_tolerance  = 0.01;
    const double angle_tolerance = 0.01;

    // Velocity limits
    const double MAX_LINEAR_VEL  = 0.1;
    const double MAX_ANGULAR_VEL = 0.7;
    const double MAX_WHEEL_SPEED = 0.5;

    // State management
    unsigned long stateChangeTime = millis();
    const unsigned long SETTLE_TIME = 300;
    
    // Motion direction flag
    bool drivingBackward = false;

    for (;;) {
        double dx = x_s - x;
        double dy = y_s - y;
        double rho = sqrt(dx*dx + dy*dy);

        // Angle from robot to goal
        double alpha = wrapAngle(atan2(dy, dx) - theta);
        
        // Angle error for final orientation
        double dtheta = wrapAngle(theta_s - theta);

        double v = 0;
        double w = 0;

        switch (currentState) {
            case DECIDE_MOTION:
                Serial.printf("[STATE 1: DECIDE_MOTION] α=%.3f (%.1f°), ρ=%.4f\n", 
                             alpha, alpha * 180.0 / M_PI, rho);
                
                // If target is behind us (|alpha| > 90°), drive backward
                if (fabs(alpha) > M_PI / 2.0) {
                    drivingBackward = true;
                    Serial.println("   → Decision: DRIVE BACKWARD");
                } else {
                    drivingBackward = false;
                    Serial.println("   → Decision: DRIVE FORWARD");
                }
                
                currentState = DRIVE_TO_GOAL;
                stateChangeTime = millis();
                break;

            case DRIVE_TO_GOAL:
                if (drivingBackward) {
                    Serial.printf("[STATE 2: DRIVE_TO_GOAL - BACKWARD] ρ=%.4f m, α=%.3f\n", rho, alpha);
                    
                    // Drive backward with inverted steering
                    v = -Kp_rho * rho;
                    
                    // Adjust alpha for backward motion
                    // When driving backward, we want to minimize the angle to 180° from goal
                    double alpha_backward = wrapAngle(alpha + (alpha > 0 ? -M_PI : M_PI));
                    w = Kp_alpha * alpha_backward;
                    
                } else {
                    Serial.printf("[STATE 2: DRIVE_TO_GOAL - FORWARD] ρ=%.4f m, α=%.3f\n", rho, alpha);
                    
                    // Drive forward normally
                    v = Kp_rho * rho;
                    w = Kp_alpha * alpha;
                }
                
                // Check if we've reached position
                if (rho < goal_tolerance) {
                    if (millis() - stateChangeTime > SETTLE_TIME) {
                        currentState = ROTATE_TO_FINAL;
                        stateChangeTime = millis();
                        Serial.println("✓ Position reached! Rotating to final angle...");
                    }
                } else {
                    stateChangeTime = millis();
                }
                break;

            case ROTATE_TO_FINAL:
                Serial.printf("[STATE 3: ROTATE_TO_FINAL] dθ=%.3f (%.1f°)\n", 
                             dtheta, dtheta * 180.0 / M_PI);
                
                v = 0;
                w = Kp_theta * dtheta;
                
                if (fabs(dtheta) < angle_tolerance) {
                    if (millis() - stateChangeTime > SETTLE_TIME) {
                        currentState = GOAL_REACHED;
                        stateChangeTime = millis();
                        Serial.println("✓ Final orientation reached!");
                    }
                } else {
                    stateChangeTime = millis();
                }
                break;

            case GOAL_REACHED:
                v = 0;
                w = 0;
                
                Serial.printf("[STATE 4: GOAL_REACHED] ρ=%.4f (tol %.4f) dθ=%.4f (tol %.4f) | Holding...\n", 
                             rho, goal_tolerance * 3.0, dtheta, angle_tolerance * 3.0);
                
                // After holding for a while, move to next setpoint
                if (millis() - stateChangeTime > SETTLE_TIME * 5) {
                    if (c < 3) {
                        c++;
                        x_s = setpointx[c];
                        y_s = setpointy[c];
                        theta_s = setpointtheta[c];
                        currentState = DECIDE_MOTION;
                        stateChangeTime = millis();
                        Serial.printf("\n🎯 Moving to setpoint %d: (%.2f, %.2f, %.1f°)\n\n", 
                                     c, x_s, y_s, theta_s * 180.0 / M_PI);
                    }
                }

                // Check for drift - return to motion if needed
                if (rho > goal_tolerance * 3.0) {
                    Serial.printf("⚠️ Position drift detected! ρ=%.4f > %.4f\n", rho, goal_tolerance * 3.0);
                    currentState = DECIDE_MOTION;
                    stateChangeTime = millis();
                } else if (fabs(dtheta) > angle_tolerance * 3.0) {
                    Serial.printf("⚠️ Angle drift detected! |dθ|=%.4f > %.4f (%.1f°)\n", 
                                 fabs(dtheta), angle_tolerance * 3.0, fabs(dtheta) * 180.0 / M_PI);
                    currentState = ROTATE_TO_FINAL;
                    stateChangeTime = millis();
                }
                break;
        }

        // Apply velocity limits
        v = constrain(v, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
        w = constrain(w, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);

        // Convert to wheel velocities (rev/s)
        double vL = (v - w * WHEEL_BASE / 2.0) / (2 * M_PI * WHEEL_RADIUS);
        double vR = (v + w * WHEEL_BASE / 2.0) / (2 * M_PI * WHEEL_RADIUS);

        vL = constrain(vL, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
        vR = constrain(vR, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);

        set_L = vL;
        set_R = vR;
        
        // Force stop at goal
        if (currentState == GOAL_REACHED) {
            set_L = 0.0;
            set_R = 0.0;
            vL = 0.0;
            vR = 0.0;
            v = 0.0;
            w = 0.0;
        }
        
        isRotatingInPlace = (set_L * set_R < 0);

        // Motion description for debug output
        double linear_speed_cm_s = v * 100.0;
        double angular_speed_deg_s = w * 180.0 / M_PI;
        double left_rpm = vL * 60.0;
        double right_rpm = vR * 60.0;
        
        String motion_desc = "";
        if (fabs(v) < 0.001 && fabs(w) < 0.001) {
            motion_desc = "⏸️  STOPPED";
        } else if (fabs(v) < 0.001) {
            motion_desc = (w > 0) ? "🔄 ROTATING CCW" : "🔃 ROTATING CW";
        } else if (fabs(w) < 0.01) {
            motion_desc = (v > 0) ? "⬆️  DRIVING FORWARD" : "⬇️  DRIVING BACKWARD";
        } else {
            if (v > 0 && w > 0) motion_desc = "↖️  FORWARD + LEFT";
            else if (v > 0 && w < 0) motion_desc = "↗️  FORWARD + RIGHT";
            else if (v < 0 && w > 0) motion_desc = "↙️  BACKWARD + LEFT";
            else motion_desc = "↘️  BACKWARD + RIGHT";
        }
        
        Serial.println("┌─────────────────────────────────────────────────────────────┐");
        Serial.printf("│ Position: x=%.3fm y=%.3fm θ=%.2f° (%.3frad)\n", 
                      x, y, theta * 180.0 / M_PI, theta);
        Serial.printf("│ Target:   x=%.3fm y=%.3fm θ=%.2f° (%.3frad)\n", 
                      x_s, y_s, theta_s * 180.0 / M_PI, theta_s);
        Serial.printf("│ Errors:   dist=%.3fm (%.1fcm)  angle=%.2f° (%.3frad)\n",
                      rho, rho * 100.0, dtheta * 180.0 / M_PI, dtheta);
        Serial.println("├─────────────────────────────────────────────────────────────┤");
        Serial.printf("│ Motion:   %s\n", motion_desc.c_str());
        Serial.printf("│ Linear:   v = %.3f m/s  (%.1f cm/s)\n", v, linear_speed_cm_s);
        Serial.printf("│ Angular:  w = %.3f rad/s  (%.1f deg/s)\n", w, angular_speed_deg_s);
        Serial.println("├─────────────────────────────────────────────────────────────┤");
        Serial.printf("│ Left wheel:  vL = %.3f rev/s  (%.1f RPM)  ", vL, left_rpm);
        if (vL > 0.01) Serial.println("⬆️ FORWARD");
        else if (vL < -0.01) Serial.println("⬇️ BACKWARD");
        else Serial.println("⏸️  STOPPED");
        Serial.printf("│ Right wheel: vR = %.3f rev/s  (%.1f RPM)  ", vR, right_rpm);
        if (vR > 0.01) Serial.println("⬆️ FORWARD");
        else if (vR < -0.01) Serial.println("⬇️ BACKWARD");
        else Serial.println("⏸️  STOPPED");
        Serial.println("└─────────────────────────────────────────────────────────────┘\n");

        vTaskDelay(pdMS_TO_TICKS(LOOP_DT_MS * 2));
    }
}

// ====== SETUP ======
void setup() {
  Serial.begin(115200);
  delay(100);

  Wire.begin(CUSTOM_SDA, CUSTOM_SCL);
  delay(1000);
  
  // Initialize OTOS sensor
  Serial.println("Initializing OTOS sensor...");
  if (!myOtos.begin()) {
    Serial.println("❌ OTOS sensor failed to initialize!");
    Serial.println("Check I2C wiring: SDA=" + String(CUSTOM_SDA) + " SCL=" + String(CUSTOM_SCL));
    Serial.println("System halted.");
    while(1) delay(1000);
  }
  Serial.println("✅ OTOS sensor initialized");

  myOtos.calibrateImu();
  myOtos.resetTracking();
  myOtos.setAngularScalar(0.9985437903);
  
  Wire.beginTransmission(0x0A);
  Wire.write(0x3A);
  Wire.write(0x5A);
  Wire.endTransmission();
  
  Serial.println("Robot Navigation Starting...");

  // Run motor direction test if enabled
  if (MOTOR_DIRECTION_TEST) {
    testMotorDirections();
  }
  
  // Run encoder test if enabled
  if (ENCODER_TEST_MODE) {
    testEncoders();
  }
  
  setupMotors();

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encLeft.attachHalfQuad(ENC_L_A, ENC_L_B);
  encRight.attachHalfQuad(ENC_R_A, ENC_R_B);
  
  encLeft.clearCount();
  encRight.clearCount();

  Serial.printf("Target: x=%.2f, y=%.2f, theta=%.2f\n", x_s, y_s, theta_s);
  Serial.println("Starting tasks in 2 seconds...");
  delay(2000);

  // Start tasks
  xTaskCreatePinnedToCore(encoderTask, "Encoders", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(pidTask, "PID", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(navigationTask, "Nav", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(SerialCommandTask, "Serial", 4096, NULL, 1, NULL, 1);

  Serial.println("✅ All tasks started!");
}

void loop() {
  // All work done in FreeRTOS tasks
}