#include <Arduino.h>
#include <PID_v1.h>
#include <ESP32Encoder.h>
#include <math.h>

// ====== L298N MOTOR PINS ======
#define ENA_L 5
#define IN1_L 6
#define IN2_L 7
#define ENB_R 8
#define IN3_R 9
#define IN4_R 10

// ====== ENCODER PINS ======
#define ENC_L_A 11
#define ENC_L_B 12
#define ENC_R_A 13
#define ENC_R_B 14

// ====== ROBOT PARAMETERS ======
#define WHEEL_RADIUS 0.0358  // 3.5 cm
#define WHEEL_BASE   0.246063    // 12 cm
#define TICKS_PER_REV 8400.0
#define LOOP_DT_MS 20

// ====== ENCODER DIRECTION ======
// Set to -1 if encoder reads opposite of motor direction
#define ENC_L_DIR 1   // Try -1 if left encoder backwards
#define ENC_R_DIR 1   // Try -1 if right encoder backwards

// ====== TEST MODE ======
#define ENCODER_TEST_MODE false  // SET TO false WHEN ENCODERS WORK
#define MOTOR_DIRECTION_TEST false  // SET TO true TO TEST MOTOR DIRECTIONS

// ====== CONTROL PARAMS ======
ESP32Encoder encLeft, encRight;
double set_L = 0, input_L = 0, output_L = 0;
double set_R = 0, input_R = 0, output_R = 0;

PID pidL(&input_L, &output_L, &set_L, 10.0, 0.2, 0.2, DIRECT);
PID pidR(&input_R, &output_R, &set_R, 10.0, 0.2, 0.2, DIRECT);

// Odometry
volatile double x = 0, y = 0, theta = M_PI / 2.0;

// Target position & orientation


int c=0;
double setpointx[4]= {0.0, 0.2, 0.2, 0.0};
double setpointy[4]= {0.2, 0.2, 0.0, 0.0};
double setpointtheta[4]= {M_PI/2, 0.0, -M_PI/2, M_PI};

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
  
  Serial.println("Test 1: Left motor FORWARD (+100 PWM)");
  Serial.println("Expected: Left wheel rotates forward");
  motorWrite(true, 150);
  delay(2000);
  stopMotors();
  delay(1000);
  
  Serial.println("\nTest 2: Left motor BACKWARD (-100 PWM)");
  Serial.println("Expected: Left wheel rotates backward");
  motorWrite(true, -150);
  delay(2000);
  stopMotors();
  delay(1000);
  
  Serial.println("\nTest 3: Right motor FORWARD (+100 PWM)");
  Serial.println("Expected: Right wheel rotates forward");
  motorWrite(false, 150);
  delay(2000);
  stopMotors();
  delay(1000);
  
  Serial.println("\nTest 4: Right motor BACKWARD (-100 PWM)");
  Serial.println("Expected: Right wheel rotates backward");
  motorWrite(false, -150);
  delay(2000);
  stopMotors();
  delay(1000);
  
  Serial.println("\nTest 5: CCW Rotation (Left=-100, Right=+100)");
  Serial.println("Expected: Robot rotates counterclockwise (left)");
  motorWrite(true, -150);
  motorWrite(false, 150);
  delay(2000);
  stopMotors();
  delay(1000);
  
  Serial.println("\nTest 6: CW Rotation (Left=+100, Right=-100)");
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

  // ESP32 Arduino Core 3.x LEDC API
  ledcAttach(ENA_L, 5000, 8);
  ledcAttach(ENB_R, 5000, 8);
}

// Remap PID output to skip deadzone
int remapPID(double pidOutput, bool rotatingInPlace) {
    const int MIN_PWM = 50;
    const int MAX_PWM = 255;


    double scaledPID = pidOutput * 15;

    if (scaledPID < -50 || scaledPID > 50) {
      scaledPID = tanh(scaledPID/80.0)*255.0;
    }else if (scaledPID<-25 || scaledPID > 25){
      scaledPID=140;
    }else{
      scaledPID = 0;
      };

    return scaledPID;
    

}
void SerialCommandTask(void *pv) {
  Serial.println("Ready for commands: use  S x y thetaDeg");

  while (true) {
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();

      // Split into tokens
      if (cmd.length() > 0 && (cmd.charAt(0) == 'S' || cmd.charAt(0) == 's')) {
        double newX, newY, thetaDeg;
        int count = sscanf(cmd.c_str() + 1, "%lf %lf %lf", &newX, &newY, &thetaDeg);

        if (count == 3) {
          x_s = newX;
          y_s = newY;
          theta_s = wrapAngle(thetaDeg * M_PI / 180.0);  // convert deg → rad
          
          // ADD THIS: Reset PIDs for new command
          pidL.SetMode(MANUAL);
          pidR.SetMode(MANUAL);
          output_L = 0;
          output_R = 0;
          pidL.SetMode(AUTOMATIC);
          pidR.SetMode(AUTOMATIC);
          
          Serial.printf("✅ New setpoint: X=%.3f m, Y=%.3f m, Theta=%.2f° (%.3f rad)\n",
                        x_s, y_s, thetaDeg, theta_s);
          Serial.println("🔄 PID controllers reset for new target");
        } else {
          Serial.println("❌ Format error! Use: S <x> <y> <thetaDeg>");
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));  // check serial every 100 ms
  }
}

void motorWrite(bool left, int pwmVal) {
  bool fwd = pwmVal >= 0;
  int pwm = constrain(abs(pwmVal), 0, 255);

  if (left) {
    // LEFT MOTOR: If your left wheel goes forward when it should go backward,
    // swap the HIGH/LOW in these lines (flip fwd to !fwd)
    digitalWrite(IN1_L, fwd ? HIGH : LOW);   
    digitalWrite(IN2_L, fwd ? LOW : HIGH);   
    ledcWrite(ENA_L, pwm);
  } else {
    // RIGHT MOTOR: If your right wheel goes forward when it should go backward,
    // swap the HIGH/LOW in these lines (flip fwd to !fwd)
    digitalWrite(IN3_R, fwd ? HIGH : LOW);   
    digitalWrite(IN4_R, fwd ? LOW : HIGH);   
    ledcWrite(ENB_R, pwm);
  }
}

// ALTERNATIVE FIX if BOTH motors are inverted:
// void motorWrite(bool left, int pwmVal) {
//   bool fwd = pwmVal >= 0;
//   int pwm = constrain(abs(pwmVal), 0, 255);
//
//   if (left) {
//     digitalWrite(IN1_L, fwd ? LOW : HIGH);    // ← Swapped
//     digitalWrite(IN2_L, fwd ? HIGH : LOW);    // ← Swapped
//     ledcWrite(ENA_L, pwm);
//   } else {
//     digitalWrite(IN3_R, fwd ? LOW : HIGH);    // ← Swapped
//     digitalWrite(IN4_R, fwd ? HIGH : LOW);    // ← Swapped
//     ledcWrite(ENB_R, pwm);
//   }
// }

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
    long currCount = rawCount * ENC_L_DIR;  // Apply direction correction
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
    long currCount = rawCount * ENC_R_DIR;  // Apply direction correction
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
  const double alpha_filter = 0.1;  // Low-pass filter coefficient (0-1)
  const double dt = LOOP_DT_MS / 1000.0;  // Convert to seconds
  
  for (;;) {
    long currL = encLeft.getCount() * ENC_L_DIR;   // Apply direction correction
    long currR = encRight.getCount() * ENC_R_DIR;
    long dL = currL - prevL;
    long dR = currR - prevR;
    prevL = currL;
    prevR = currR;

    // Calculate distances
    double distL = 2 * M_PI * WHEEL_RADIUS * (dL / TICKS_PER_REV);
    double distR = 2 * M_PI * WHEEL_RADIUS * (dR / TICKS_PER_REV);

    // Update odometry
    double dCenter = (distR + distL) / 2.0;
    double dTheta = (distR - distL) / WHEEL_BASE;

    theta = wrapAngle(theta + dTheta);

    x += dCenter * cos(theta);
    y += dCenter * sin(theta);

    // Calculate velocities with low-pass filter to reduce noise
    double rawL = (dL / TICKS_PER_REV) / dt;
    double rawR = (dR / TICKS_PER_REV) / dt;
    
    filteredL = alpha_filter * rawL + (1.0 - alpha_filter) * filteredL;
    filteredR = alpha_filter * rawR + (1.0 - alpha_filter) * filteredR;
    
    input_L = filteredL ;
    input_R = filteredR ;

    vTaskDelay(pdMS_TO_TICKS(LOOP_DT_MS));
  }
}

void pidTask(void *pv) {
  pidL.SetMode(AUTOMATIC);
  pidR.SetMode(AUTOMATIC);
  pidL.SetOutputLimits(-255, 255);
  pidR.SetOutputLimits(-255, 255);

  // === Tunable PID Coefficients ===
  double KpL = 100.0, KiL = 1, KdL = 5;
  double KpR = 100.0, KiR = 1, KdR = 5;

  pidL.SetTunings(KpL, KiL, KdL);
  pidR.SetTunings(KpR, KiR, KdR);

  const double lowKi = 0.1;
  const unsigned long windupTimeout = 150; // ms
  static unsigned long satStartL = 0, satStartR = 0;
  static bool wasInSaturationL = false, wasInSaturationR = false;
  unsigned long lastPrint = 0;
  static bool wasAtGoal = false;  // ADD THIS


  for (;;) {
    // === Compute PID ===
    bool atGoal = (fabs(set_L) < 0.001 && fabs(set_R) < 0.001);
    if (atGoal && !wasAtGoal) {
      pidL.SetMode(MANUAL);  // Disable PID temporarily
      pidR.SetMode(MANUAL);
      output_L = 0;          // Force output to zero
      output_R = 0;
      pidL.SetMode(AUTOMATIC);  // Re-enable (this clears integral term)
      pidR.SetMode(AUTOMATIC);
      Serial.println("🛑 PID RESET - Clearing integral windup");
    }
    wasAtGoal = atGoal;
    pidL.Compute();
    pidR.Compute();


    // === VELOCITY DEBUG OUTPUT ===
    double errorL = set_L - input_L;
    double errorR = set_R - input_R;

    errorL*=10;
    errorR*=10;

    // Serial.println("╔════════════════════════════════════════════════════════════╗");
    // Serial.println("║                    PID VELOCITY DEBUG                      ║");
    // Serial.println("╠════════════════════════════════════════════════════════════╣");
    // Serial.printf("║ LEFT WHEEL:                                                ║\n");
    // Serial.printf("║   Setpoint:  %.4f rev/s  (%.1f RPM)                       ║\n", set_L, set_L * 60.0);
    // Serial.printf("║   Current:   %.4f rev/s  (%.1f RPM)                       ║\n", input_L, input_L * 60.0);
    // Serial.printf("║   Error:     %.4f rev/s  (%.1f RPM)                       ║\n", errorL, errorL * 60.0);
    // Serial.printf("║   PID Out:   %.3f  (range: -100 to +100)                  ║\n", output_L);
    // Serial.println("╠════════════════════════════════════════════════════════════╣");
    // Serial.printf("║ RIGHT WHEEL:                                               ║\n");
    // Serial.printf("║   Setpoint:  %.4f rev/s  (%.1f RPM)                       ║\n", set_R, set_R * 60.0);
    // Serial.printf("║   Current:   %.4f rev/s  (%.1f RPM)                       ║\n", input_R, input_R * 60.0);
    // Serial.printf("║   Error:     %.4f rev/s  (%.1f RPM)                       ║\n", errorR, errorR * 60.0);
    // Serial.printf("║   PID Out:   %.3f  (range: -100 to +100)                  ║\n", output_R);
    // Serial.println("╚════════════════════════════════════════════════════════════╝\n");

    // ---- Apply motor outputs ----
    int pwmL = remapPID(output_L, isRotatingInPlace);
    int pwmR = remapPID(output_R, isRotatingInPlace);
    
    Serial.printf("PWM COMPUTED LEFT: %d PWM COMPUTED RIGHT: %d\n\n", pwmL, pwmR);

    motorWrite(true, pwmL);
    motorWrite(false, pwmR);

    vTaskDelay(pdMS_TO_TICKS(LOOP_DT_MS));
  }
}
void navigationTask(void *pv) {
    // === STATE MACHINE STATES ===
    enum State {
        ROTATE_TO_GOAL,      // Step 1: Rotate to face the goal
        DRIVE_TO_GOAL,       // Step 2: Drive straight to goal position
        GOAL_REACHED         // Step 4: Stay stopped
    };
    
    State currentState = ROTATE_TO_GOAL;

    // Gains
    const double Kp_rho   = 0.7;
    const double Kp_alpha = 5;   // Increased for faster initial rotation
    const double Kp_theta = 1;   //5 Increased for final rotation

    const double goal_tolerance  = 0.02;  // meters
    const double angle_tolerance = 0.05;  // radians (~2 degrees)

    const double MAX_LINEAR_VEL  = 0.1;   // m/s
    const double MAX_ANGULAR_VEL = 0.7;    // rad/s
    const double MAX_WHEEL_SPEED = 0.5;    // rev/s
    
    // Minimum angular velocity to overcome friction
    const double MIN_ANGULAR_VEL = 0.05;   // rad/s

    unsigned long stateChangeTime = millis();
    const unsigned long SETTLE_TIME = 300;  // ms to stay in state before transition

    for (;;) {
        double dx = x_s - x;
        double dy = y_s - y;
        double rho = sqrt(dx*dx + dy*dy);

        double alpha = wrapAngle(atan2(dy, dx) - theta);  // Angle to goal
        double dtheta = wrapAngle(theta_s - theta);        // Final orientation error
        
        // Debug: Print raw angle values when holding position
        static int holdCounter = 0;
        if (currentState == GOAL_REACHED) {
            holdCounter++;
            if (holdCounter % 10 == 0) {  // Print every 10 cycles
                Serial.printf("   [HOLD DEBUG] theta=%.6f theta_s=%.6f raw_diff=%.6f wrapped_dtheta=%.6f\n",
                             theta, theta_s, theta_s - theta, dtheta);
            }
        } else {
            holdCounter = 0;
        }

        double v = 0;
        double w = 0;

        // === STATE MACHINE ===
        switch (currentState) {
            case ROTATE_TO_GOAL:
                // Rotate to final desired orientation
                Serial.printf("[STATE 3: ROTATE_TO_GOAL] dθ=%.3f (%.1f°)\n", dtheta, dtheta * 180.0 / M_PI);
                
                v = 0;  // No linear motion
                w = Kp_theta * dtheta ;
                
                // Apply minimum velocity
                // if (fabs(w) < MIN_ANGULAR_VEL && fabs(w) > 0.001) {
                //     Serial.printlN("APPLYING MIN VELOCITY")
                //     w = (w > 0) ? MIN_ANGULAR_VEL : -MIN_ANGULAR_VEL;
                // }
                
                // Transition: correct orientation?
                if (fabs(dtheta) < angle_tolerance) {
                    Serial.println("Correct Orientation");

                    if (millis() - stateChangeTime > SETTLE_TIME) {
                        if (fabs(rho) < goal_tolerance){
                          Serial.println("Correct Orientation Everything, Goal Reached");

                          currentState = GOAL_REACHED;
                          stateChangeTime = millis();
                        }else{
                          Serial.println("Driving to Goal");

                          currentState = DRIVE_TO_GOAL;
                          stateChangeTime = millis();
                        }
                    }
                } else {
                    stateChangeTime = millis();  // Reset settle timer
                }
                break;

            case DRIVE_TO_GOAL:
                // Drive straight to goal (with minor heading corrections)
                Serial.printf("[STATE 2: DRIVE_TO_GOAL] ρ=%.4f m, α=%.3f\n", rho, alpha);
                
                v = Kp_rho * rho;
                w = Kp_alpha * alpha ;  // Gentle correction to stay on course
                
                // Transition: reached position?
                if (rho < goal_tolerance) {
                    if (millis() - stateChangeTime > SETTLE_TIME) {
                        currentState = ROTATE_TO_GOAL;
                        stateChangeTime = millis();
                        Serial.println("✓ Position reached! Adjusting final angle...");
                    }
                } else {
                    stateChangeTime = millis();  // Reset settle timer
                }
                break;

    

            case GOAL_REACHED:
                // Stop all motion
                v = 0;
                w = 0;
                
                Serial.printf("[STATE 4: GOAL_REACHED] ρ=%.4f (tol %.4f) dθ=%.4f (tol %.4f) | Holding...\n", 
                             rho, goal_tolerance * 3.0, dtheta, angle_tolerance * 3.0);
                

                if (millis() - stateChangeTime > SETTLE_TIME*5){
                  if (c<3){
                    c++;
                  };
                  x_s = setpointx[c];
                  y_s = setpointy[c];
                  theta_s = setpointtheta[c];
                  currentState = ROTATE_TO_GOAL;
                  stateChangeTime = millis();

                }

                // If we drift away significantly, restart (use 3x tolerance to avoid false triggers)
                if (rho > goal_tolerance * 3.0) {
                    Serial.printf("⚠️ Position drift detected! ρ=%.4f > %.4f\n", rho, goal_tolerance * 3.0);
                    currentState = ROTATE_TO_GOAL;
                    stateChangeTime = millis();
                } else if (fabs(dtheta) > angle_tolerance * 3.0) {
                    Serial.printf("⚠️ Angle drift detected! |dθ|=%.4f > %.4f (%.1f°)\n", 
                                 fabs(dtheta), angle_tolerance * 3.0, fabs(dtheta) * 180.0 / M_PI);
                    Serial.printf("   Current θ=%.4f (%.1f°), Target θ_s=%.4f (%.1f°)\n",
                                 theta, theta * 180.0 / M_PI, theta_s, theta_s * 180.0 / M_PI);
                    



                    // Check if this is angle wrapping issue
                    double alt_dtheta = theta_s - theta;  // Unwrapped difference
                    Serial.printf("   Unwrapped dθ=%.4f, Wrapped dθ=%.4f\n", alt_dtheta, dtheta);
                    
                    // Only restart if drift is real (not a wrapping artifact)
                    if (fabs(alt_dtheta) < M_PI) {
                        currentState = ROTATE_TO_GOAL;  // Just fix angle, don't restart fully
                        stateChangeTime = millis();
                    }

                }

                break;
        }

        // Clamp velocities


        v = constrain(v, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
        w = constrain(w, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);

        Serial.println("Velocities: ");
        Serial.println(v);
        Serial.println(w);
        // Convert to wheel speeds
        double vL = (v - w * WHEEL_BASE / 2.0) / (2 * M_PI * WHEEL_RADIUS);
        double vR = (v + w * WHEEL_BASE / 2.0) / (2 * M_PI * WHEEL_RADIUS);

        vL = constrain(vL, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
        vR = constrain(vR, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);

        Serial.print(vL);
        Serial.print(" VR: ");
        Serial.print(vR);
        Serial.println("_____________________");

        // Update PID setpoints
        set_L = vL;
        set_R = vR;
        
        // CRITICAL: In GOAL_REACHED state, force setpoints to exactly zero
        if (currentState == GOAL_REACHED) {
            set_L = 0.0;
            set_R = 0.0;
            vL = 0.0;
            vR = 0.0;
            v = 0.0;
            w = 0.0;
        }
        
        // Update global rotation flag
        isRotatingInPlace = (set_L * set_R < 0);

        // === DETAILED DEBUG OUTPUT ===
        
        // Convert velocities to understandable units
        double linear_speed_cm_s = v * 100.0;  // m/s → cm/s
        double angular_speed_deg_s = w * 180.0 / M_PI;  // rad/s → deg/s
        double left_rpm = vL * 60.0;  // rev/s → RPM
        double right_rpm = vR * 60.0;  // rev/s → RPM
        
        // Determine motion direction
        String motion_desc = "";
        if (fabs(v) < 0.001 && fabs(w) < 0.001) {
            motion_desc = "⏸️  STOPPED";
        } else if (fabs(v) < 0.001) {
            if (w > 0) {
                motion_desc = "🔄 ROTATING CCW (left)";
            } else {
                motion_desc = "🔃 ROTATING CW (right)";
            }
        } else if (fabs(w) < 0.01) {
            if (v > 0) {
                motion_desc = "⬆️  DRIVING FORWARD";
            } else {
                motion_desc = "⬇️  DRIVING BACKWARD";
            }
        } else {
            if (v > 0 && w > 0) {
                motion_desc = "↖️  FORWARD + LEFT TURN";
            } else if (v > 0 && w < 0) {
                motion_desc = "↗️  FORWARD + RIGHT TURN";
            } else if (v < 0 && w > 0) {
                motion_desc = "↙️  BACKWARD + LEFT TURN";
            } else {
                motion_desc = "↘️  BACKWARD + RIGHT TURN";
            }
        }
        
        Serial.println("┌─────────────────────────────────────────────────────────────┐");
        Serial.printf("│ Position: x=%.3fm y=%.3fm θ=%.2f° (%.3frad)            \n", 
                      x, y, theta * 180.0 / M_PI, theta);
        Serial.printf("│ Target:   x=%.3fm y=%.3fm θ=%.2f° (%.3frad)            \n", 
                      x_s, y_s, theta_s * 180.0 / M_PI, theta_s);
        Serial.printf("│ Errors:   dist=%.3fm (%.1fcm)  angle=%.2f° (%.3frad)       \n",
                      rho, rho * 100.0, dtheta * 180.0 / M_PI, dtheta);
        Serial.println("├─────────────────────────────────────────────────────────────┤");
        Serial.printf("│ Motion:   %s\n", motion_desc.c_str());
        Serial.printf("│ Linear:   v = %.3f m/s  (%.1f cm/s)                        \n", 
                      v, linear_speed_cm_s);
        Serial.printf("│ Angular:  w = %.3f rad/s  (%.1f deg/s)                     \n", 
                      w, angular_speed_deg_s);
        Serial.println("├─────────────────────────────────────────────────────────────┤");
        Serial.printf("│ Left wheel:  vL = %.3f rev/s  (%.1f RPM)  ", vL, left_rpm);
        if (vL > 0.01) Serial.println("⬆️ FORWARD");
        else if (vL < -0.01) Serial.println("⬇️ BACKWARD");
        else Serial.println("⏸️  STOPPED");
        Serial.printf("│ Right wheel: vR = %.3f rev/s  (%.1f RPM)  ", vR, right_rpm);
        if (vR > 0.01) Serial.println("⬆️ FORWARD");
        else if (vR < -0.01) Serial.println("⬇️ BACKWARD");
        else Serial.println("⏸️  STOPPED");
        
        // Show if wheels are creating rotation
        if (isRotatingInPlace) {
            Serial.println("│ ⚙️  IN-PLACE ROTATION DETECTED (opposite wheel directions)");
        }
        
        Serial.println("└─────────────────────────────────────────────────────────────┘\n");

        vTaskDelay(pdMS_TO_TICKS(LOOP_DT_MS * 2));
    }
}

// ====== SETUP ======
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("🤖 Robot Navigation Starting...");
  
  // Run motor direction test if enabled
  if (MOTOR_DIRECTION_TEST) {
    testMotorDirections();
    // Never returns from test mode
  }
  
  // Run encoder test if enabled
  if (ENCODER_TEST_MODE) {
    testEncoders();
    // Never returns from test mode
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