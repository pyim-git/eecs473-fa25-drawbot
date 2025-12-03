#define Serial Serial0
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <math.h>
#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WifiClient.h>
#include <string>

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


extern volatile double x, y, theta;
extern volatile double x_s, y_s, theta_s;
extern double set_L, input_L, output_L;
extern double set_R, input_R, output_R;
extern volatile bool isRotatingInPlace;
extern volatile int currentNavState;
volatile int currentNavState = 0;
volatile double S_param = 50.0;



// --- WebSocket variables ---
// WifiClient webserver;
AsyncWebSocket ws("/ws");
AsyncWebServer server(80);
bool clientConnected = false;

volatile double Kp_rho   = 0.7;
volatile double Kp_alpha = 5.0;
volatile double Kp_theta = 1.0;
volatile double Kp = 500.0;
volatile double Ki = 0.0;
volatile double Kd = 0.0;
// --- WebSocket Functions ---

// Forward declarations
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len);

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    String msg = (char*)data;

    // if (msg == "FORWARD") {
    //   ledState = true;
    //   digitalWrite(LED_BUILTIN, HIGH);
    //   digitalWrite(in1, HIGH);
    //   digitalWrite(in2, LOW);
    // } else if (msg == "STOP") {
    //   ledState = false;
    //   digitalWrite(LED_BUILTIN, LOW);
    //   digitalWrite(in1, LOW);
    //   digitalWrite(in2, LOW);
    // }
    // notifyClients();
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    clientConnected = true;
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
    clientConnected = false;
  } else if (type == WS_EVT_DATA) {
    handleWebSocketMessage(arg, data, len);
  } else if (type == WS_EVT_ERROR) {
    Serial.printf("WebSocket error on client #%u\n", client->id());
  }
}
void wsCleanupTask(void *pv) {
  for (;;) {
    ws.cleanupClients();
    vTaskDelay(pdMS_TO_TICKS(1000)); // Clean up every second
  }
}
void sendTelemetry() {
  // Check if any clients are connected
  if (ws.count() == 0) return;
  
  // State names
  const char* stateNames[] = {"ROTATE_TO_GOAL", "DRIVE_TO_GOAL", "ROTATE_TO_FINAL", "GOAL_REACHED"};
  
  // Create JSON string with telemetry data
  String json = "{";
  json += "\"x\":" + String(x, 3) + ",";
  json += "\"y\":" + String(y, 3) + ",";
  json += "\"theta\":" + String(theta * 180.0 / M_PI, 2) + ",";
  json += "\"x_s\":" + String(x_s, 3) + ",";
  json += "\"y_s\":" + String(y_s, 3) + ",";
  json += "\"theta_s\":" + String(theta_s * 180.0 / M_PI, 2) + ",";
  json += "\"rawL\":" + String(output_L, 3) + ",";
  json += "\"rawR\":" + String(output_R, 3) + ",";

  
  double dx = x_s - x;
  double dy = y_s - y;
  double rho = sqrt(dx*dx + dy*dy);
  double dtheta = wrapAngle(theta_s - theta);
  
  json += "\"error_dist\":" + String(rho, 4) + ",";
  json += "\"error_angle\":" + String(dtheta * 180.0 / M_PI, 2) + ",";
  
  // Get PWM values from remapPID
  int pwmL = remapPID(output_L, isRotatingInPlace);
  int pwmR = remapPID(output_R, isRotatingInPlace);
  
  json += "\"pwm_left\":" + String(pwmL) + ",";
  json += "\"pwm_right\":" + String(pwmR) + ",";
  json += "\"set_L\":" + String(set_L, 3) + ",";
  json += "\"set_R\":" + String(set_R, 3) + ",";
  
  // Add state information
  json += "\"state\":" + String(currentNavState) + ",";
  json += "\"state_name\":\"" + String(stateNames[currentNavState]) + "\"";
  
  json += "}";
  
  ws.textAll(json);
}
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
    const int MIN_PWM = 150;
    const int DEAD_PWM = 40;
    const int MAX_PWM = 255;

    double scaledPID = pidOutput * 1;

    if (scaledPID < -atanh(MIN_PWM/255.0)*S_param || scaledPID > atanh(MIN_PWM/255.0)*S_param) {
      scaledPID = tanh(scaledPID/S_param)*255.0;
    } else if (scaledPID < -atanh(DEAD_PWM/255.0)*S_param) {
      scaledPID = -MIN_PWM;
    } else if (scaledPID > atanh(DEAD_PWM/255.0)*S_param) {
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

      if (cmd.length() > 0) {
        char cmdType = cmd.charAt(0);

        // Setpoint command: S <x> <y> <thetaDeg>
        if (cmdType == 'S' || cmdType == 's') {
          double newX, newY, thetaDeg;
          int count = sscanf(cmd.c_str() + 1, "%lf %lf %lf", &newX, &newY, &thetaDeg);

          if (count == 3) {
            x_s = newX;
            y_s = newY;
            theta_s = wrapAngle(thetaDeg * M_PI / 180.0);
            
            // RESET NAVIGATION STATE TO START
            currentNavState = 0;  // Reset to ROTATE_TO_GOAL state
            
            Serial.printf("✅ New setpoint: X=%.3f m, Y=%.3f m, Theta=%.2f° (%.3f rad)\n",
                          x_s, y_s, thetaDeg, theta_s);
            Serial.println("🔄 Navigation state reset to ROTATE_TO_GOAL");
          } else {
            Serial.println("❌ Format error! Use: S <x> <y> <thetaDeg>");
          }
        }
        else if (cmdType == 'C' || cmdType == 'c') {
          double newKp_rho, newKp_alpha, newKp_theta;
          int count = sscanf(cmd.c_str() + 1, "%lf %lf %lf", &newKp_rho, &newKp_alpha, &newKp_theta);

          if (count == 3) {

            Kp_rho = newKp_rho;
            Kp_alpha = newKp_alpha;
            Kp_theta = newKp_theta;
                
          } else {
            Serial.println("❌ Format error! Use: S <x> <y> <thetaDeg>");
          }
        }
        else if (cmdType == 'D' || cmdType == 'd') {
          double newKp, newKi, newKd;
          int count = sscanf(cmd.c_str() + 1, "%lf %lf %lf", &newKp, &newKi, &newKd);

          if (count == 3) {


            Kp = newKp;
            Ki = newKi;
            Kd = newKd;
                
          } else {
            Serial.println("❌ Format error! Use: D <kp> <ki> <kd>");
          }
        }
        else if (cmdType == 't' || cmdType == 'T') {
            double newS;
            int count = sscanf(cmd.c_str() + 2, "%lf", &newS);

            if (count == 1 && newS > 0) {
                S_param = newS;
                Serial.printf("✅ S_param updated to %.3f\n", S_param);
            } else {
                Serial.println("❌ Format: SS <value>   (value > 0)");
            }
          }
        // Position reset command: P <x> <y> <thetaDeg>
        else if (cmdType == 'P' || cmdType == 'p') {
          double newX, newY, thetaDeg;
          int count = sscanf(cmd.c_str() + 1, "%lf %lf %lf", &newX, &newY, &thetaDeg);

          if (count == 3) {
            x = newX;
            y = newY;
            theta = wrapAngle(thetaDeg * M_PI / 180.0);
            
            // Reset encoders
            encLeft.clearCount();
            encRight.clearCount();
            
            // RESET NAVIGATION STATE TO START
            currentNavState = 0;  // Reset to ROTATE_TO_GOAL state
            
            Serial.printf("✅ Position reset: X=%.3f m, Y=%.3f m, Theta=%.2f° (%.3f rad)\n",
                          x, y, thetaDeg, theta);
            Serial.println("   Encoders cleared");
            Serial.println("🔄 Navigation state reset to ROTATE_TO_GOAL");
          } else {
            Serial.println("❌ Format error! Use: P <x> <y> <thetaDeg>");
          }
        }
        
        else {
          Serial.println("❌ Unknown command! Available commands:");
          Serial.println("   S <x> <y> <thetaDeg> - Set target setpoint");
          Serial.println("   P <x> <y> <thetaDeg> - Reset current position");
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
    // sfe_otos_pose2d_t myPosition;
    // myOtos.getPosition(myPosition);
    // double rawTheta = myPosition.h * 1000 / 57296;
    double rawTheta = 0;
    
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
    // Serial.printf("PID: L[set=%.3f cur=%.3f err=%.3f out=%.1f] R[set=%.3f cur=%.3f err=%.3f out=%.1f]\n",
    //               set_L, input_L, errorL, output_L,
    //               set_R, input_R, errorR, output_R);

    // ws.textAll("PWM");

    // === Apply PWM ===
    int pwmL = remapPID(output_L, isRotatingInPlace);
    int pwmR = remapPID(output_R, isRotatingInPlace);
    
    // Serial.printf("PWM: LEFT=%d RIGHT=%d\n\n", pwmL, pwmR);

    motorWrite(true, pwmL);
    motorWrite(false, pwmR);

    vTaskDelay(pdMS_TO_TICKS(LOOP_DT_MS));
  }
}

// ====== NAVIGATION TASK (WITH BACKWARD MOTION) ======
void navigationTask(void *pv) {
    enum State {
        ROTATE_TO_GOAL = 0,
        DRIVE_TO_GOAL = 1,
        ROTATE_TO_FINAL = 2,
        GOAL_REACHED = 3
    };
    
    State currentState = ROTATE_TO_GOAL;
    currentNavState = (int)currentState;

    // Tolerances
    const double goal_tolerance  = 0.01;
    const double angle_tolerance = 0.01;
    const double MAX_DRIVE_HEADING_ERROR=0.1;
    // Velocity limits
    const double MAX_LINEAR_VEL  = 0.1;
    const double MAX_ANGULAR_VEL = 0.7;
    const double MAX_WHEEL_SPEED = 0.5;

    // State management
    unsigned long stateChangeTime = millis();
    const unsigned long SETTLE_TIME = 300;

    for (;;) {
        // CHECK FOR EXTERNAL STATE RESET (from serial command)
        if (currentNavState != (int)currentState) {
            currentState = (State)currentNavState;
            stateChangeTime = millis();
            Serial.printf("🔄 State externally reset to: %d\n", currentNavState);
        }
        
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
            case ROTATE_TO_GOAL: {
                double targetangle = wrapAngle(atan2(y_s - y, x_s - x));
                double angle_to_goal = wrapAngle(targetangle - theta);
                
                // Serial.printf("[STATE 1: ROTATE_TO_GOAL] target_angle=%.3f (%.1f°), error=%.3f (%.1f°)\n", 
                //              targetangle, targetangle * 180.0 / M_PI, 
                //              angle_to_goal, angle_to_goal * 180.0 / M_PI);
                
                v = 0;
                w = Kp_theta * angle_to_goal;
                
                if (fabs(angle_to_goal) < angle_tolerance) {
                    if (millis() - stateChangeTime > SETTLE_TIME) {
                        currentState = DRIVE_TO_GOAL;
                        currentNavState = (int)currentState;
                        stateChangeTime = millis();
                        Serial.println("✓ Aligned to goal! Starting drive...");
                    }
                } else {
                    stateChangeTime = millis();
                }
                break;
            }

            case DRIVE_TO_GOAL:
                // Serial.printf("[STATE 2: DRIVE_TO_GOAL] ρ=%.4f m, α=%.3f (%.1f°)\n", 
                //              rho, alpha, alpha * 180.0 / M_PI);
                if (fabs(alpha) > MAX_DRIVE_HEADING_ERROR) {   // e.g. 0.35 rad (~20°)
                  Serial.println("! Heading drift too large → switching to ROTATE_TO_GOAL");
                  currentState = ROTATE_TO_GOAL;
                  currentNavState = (int)currentState;
                  stateChangeTime = millis();
                  stopMotors();
                  break;
                }
                // Drive forward with heading correction
                v = Kp_rho * rho;
                w = Kp_alpha * alpha;
                
                // Check if we've reached position
                if (rho < goal_tolerance) {
                    if (millis() - stateChangeTime > SETTLE_TIME) {
                        currentState = ROTATE_TO_FINAL;
                        currentNavState = (int)currentState;
                        stateChangeTime = millis();
                        Serial.println("✓ Position reached! Rotating to final angle...");
                    }
                } else {
                    stateChangeTime = millis();
                }
                break;

            case ROTATE_TO_FINAL:
                // Serial.printf("[STATE 3: ROTATE_TO_FINAL] dθ=%.3f (%.1f°)\n", 
                //              dtheta, dtheta * 180.0 / M_PI);
                
                v = 0;
                w = Kp_theta * dtheta;
                
                if (fabs(dtheta) < angle_tolerance) {
                    if (millis() - stateChangeTime > SETTLE_TIME) {
                        currentState = GOAL_REACHED;
                        currentNavState = (int)currentState;
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
                
                // Serial.printf("[STATE 4: GOAL_REACHED] ρ=%.4f (tol %.4f) dθ=%.4f (tol %.4f) | Holding...\n", 
                //              rho, goal_tolerance * 3.0, dtheta, angle_tolerance * 3.0);
                
                // After holding for a while, move to next setpoint
                if (millis() - stateChangeTime > SETTLE_TIME * 5) {
                    if (c < 3) {
                        c++;
                        x_s = setpointx[c];
                        y_s = setpointy[c];
                        theta_s = setpointtheta[c];
                        currentState = ROTATE_TO_GOAL;
                        currentNavState = (int)currentState;
                        stateChangeTime = millis();
                        Serial.printf("\n🎯 Moving to setpoint %d: (%.2f, %.2f, %.1f°)\n\n", 
                                     c, x_s, y_s, theta_s * 180.0 / M_PI);
                    }
                }

                // Check for drift - return to motion if needed
                if (rho > goal_tolerance * 3.0) {
                    Serial.printf("⚠️ Position drift detected! ρ=%.4f > %.4f\n", rho, goal_tolerance * 3.0);
                    currentState = ROTATE_TO_GOAL;
                    currentNavState = (int)currentState;
                    stateChangeTime = millis();
                } else if (fabs(dtheta) > angle_tolerance * 3.0) {
                    Serial.printf("⚠️ Angle drift detected! |dθ|=%.4f > %.4f (%.1f°)\n", 
                                 fabs(dtheta), angle_tolerance * 3.0, fabs(dtheta) * 180.0 / M_PI);
                    currentState = ROTATE_TO_FINAL;
                    currentNavState = (int)currentState;
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

        sendTelemetry();
        
        vTaskDelay(pdMS_TO_TICKS(LOOP_DT_MS * 2));
    }
}
// ====== SETUP ======
void setup() {
  Serial.begin(115200);
  delay(100);

  Wire.begin(CUSTOM_SDA, CUSTOM_SCL);
  delay(1000);
  
  WifiClient wifi;
  Serial.print("ESP IP Address: ");
  Serial.println(wifi.connectWiFi("shruti", "shruti05"));
  
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.begin();
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(200, "text/html", R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Robot Telemetry</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    
    body {
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      background: linear-gradient(135deg, #1e3c72 0%, #2a5298 100%);
      color: #fff;
      padding: 20px;
      min-height: 100vh;
    }
    
    .container {
      max-width: 1200px;
      margin: 0 auto;
    }
    
    h1 {
      text-align: center;
      margin-bottom: 30px;
      font-size: 2.5em;
      text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
    }
    
    .status-badge {
      display: inline-block;
      padding: 8px 16px;
      border-radius: 20px;
      font-size: 0.9em;
      font-weight: bold;
      margin-left: 15px;
      animation: pulse 2s infinite;
    }
    
    .status-connected {
      background: #4caf50;
    }
    
    .status-disconnected {
      background: #f44336;
    }
    
    @keyframes pulse {
      0%, 100% { opacity: 1; }
      50% { opacity: 0.7; }
    }
    
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
      gap: 20px;
      margin-bottom: 20px;
    }
    
    .card {
      background: rgba(255, 255, 255, 0.1);
      backdrop-filter: blur(10px);
      border-radius: 15px;
      padding: 25px;
      box-shadow: 0 8px 32px rgba(0, 0, 0, 0.3);
      border: 1px solid rgba(255, 255, 255, 0.18);
    }
    
    .card h2 {
      font-size: 1.4em;
      margin-bottom: 20px;
      color: #ffd700;
      border-bottom: 2px solid rgba(255, 215, 0, 0.3);
      padding-bottom: 10px;
    }
    
    .metric {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 12px 0;
      border-bottom: 1px solid rgba(255, 255, 255, 0.1);
    }
    
    .metric:last-child {
      border-bottom: none;
    }
    
    .metric-label {
      font-weight: 600;
      color: #e0e0e0;
    }
    
    .metric-value {
      font-size: 1.3em;
      font-weight: bold;
      color: #fff;
      font-family: 'Courier New', monospace;
    }
    
    .unit {
      font-size: 0.8em;
      color: #b0b0b0;
      margin-left: 5px;
    }
    
    .pwm-bar-container {
      margin-top: 8px;
      background: rgba(0, 0, 0, 0.3);
      border-radius: 10px;
      height: 20px;
      overflow: hidden;
      position: relative;
    }
    
    .pwm-bar {
      height: 100%;
      transition: width 0.3s ease;
      border-radius: 10px;
    }
    
    .pwm-bar.positive {
      background: linear-gradient(90deg, #4caf50, #8bc34a);
      float: left;
    }
    
    .pwm-bar.negative {
      background: linear-gradient(90deg, #ff5722, #f44336);
      float: right;
    }
    
    .pwm-label {
      position: absolute;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
      font-size: 0.85em;
      font-weight: bold;
      text-shadow: 1px 1px 2px rgba(0,0,0,0.8);
    }
    
    .error-warning {
      color: #ff6b6b;
      font-weight: bold;
    }
    
    .error-ok {
      color: #51cf66;
    }
    
    .visualizer {
      grid-column: 1 / -1;
      height: 400px;
      position: relative;
      background: rgba(0, 0, 0, 0.2);
      border: 2px solid rgba(255, 255, 255, 0.2);
      overflow: hidden;
    }
    
    .robot-dot {
      position: absolute;
      width: 20px;
      height: 20px;
      background: #4caf50;
      border-radius: 50%;
      transform: translate(-50%, -50%);
      box-shadow: 0 0 20px rgba(76, 175, 80, 0.8);
      z-index: 10;
    }
    
    .robot-heading {
      position: absolute;
      width: 30px;
      height: 3px;
      background: #fff;
      transform-origin: left center;
      box-shadow: 0 0 10px rgba(255, 255, 255, 0.8);
    }
    
    .target-dot {
      position: absolute;
      width: 16px;
      height: 16px;
      background: #ff6b6b;
      border-radius: 50%;
      transform: translate(-50%, -50%);
      box-shadow: 0 0 20px rgba(255, 107, 107, 0.8);
    }
    
    .grid-lines {
      position: absolute;
      width: 100%;
      height: 100%;
      opacity: 0.2;
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>
      🤖 Robot Navigation Telemetry
      <span id="status" class="status-badge status-disconnected">DISCONNECTED</span>
    </h1>
    
    <div class="grid">
      <!-- Position Card -->
      <div class="card">
        <h2>📍 Current Position</h2>
        <div class="metric">
          <span class="metric-label">X Position:</span>
          <span class="metric-value"><span id="pos-x">0.000</span><span class="unit">m</span></span>
        </div>
        <div class="metric">
          <span class="metric-label">Y Position:</span>
          <span class="metric-value"><span id="pos-y">0.000</span><span class="unit">m</span></span>
        </div>
        <div class="metric">
          <span class="metric-label">Heading (θ):</span>
          <span class="metric-value"><span id="pos-theta">0.0</span><span class="unit">°</span></span>
        </div>
      </div>
      
      <div class="card">
        <h2>🧭 Navigation State</h2>
        <div class="metric">
          <span class="metric-label">Current State:</span>
          <span class="metric-value" id="nav-state" style="font-size: 1.1em;">UNKNOWN</span>
        </div>
        <div style="margin-top: 15px; padding: 10px; background: rgba(0,0,0,0.2); border-radius: 8px;">
          <div style="font-size: 0.9em; color: #b0b0b0;">
            <div id="state-desc">Waiting for data...</div>
          </div>
        </div>
      </div>

      <!-- Target Card -->
      <div class="card">
        <h2>🎯 Target Setpoint</h2>
        <div class="metric">
          <span class="metric-label">Target X:</span>
          <span class="metric-value"><span id="target-x">0.000</span><span class="unit">m</span></span>
        </div>
        <div class="metric">
          <span class="metric-label">Target Y:</span>
          <span class="metric-value"><span id="target-y">0.000</span><span class="unit">m</span></span>
        </div>
        <div class="metric">
          <span class="metric-label">Target θ:</span>
          <span class="metric-value"><span id="target-theta">0.0</span><span class="unit">°</span></span>
        </div>
      </div>
      
      <!-- Errors Card -->
      <div class="card">
        <h2>⚠️ Position Errors</h2>
        <div class="metric">
          <span class="metric-label">Distance Error:</span>
          <span class="metric-value" id="error-dist-display">
            <span id="error-dist">0.0000</span><span class="unit">m</span>
          </span>
        </div>
        <div class="metric">
          <span class="metric-label">Distance (cm):</span>
          <span class="metric-value">
            <span id="error-dist-cm">0.0</span><span class="unit">cm</span>
          </span>
        </div>
        <div class="metric">
          <span class="metric-label">Angle Error:</span>
          <span class="metric-value" id="error-angle-display">
            <span id="error-angle">0.0</span><span class="unit">°</span>
          </span>
        </div>
      </div>
      
      <!-- Motor Control Card -->
      <div class="card">
        <h2>⚡ Motor Control</h2>
        <div class="metric">
          <span class="metric-label">Left Motor PWM:</span>
          <span class="metric-value"><span id="pwm-left">0</span></span>
        </div>
        <div class="pwm-bar-container">
          <div class="pwm-bar" id="pwm-bar-left"></div>
          <span class="pwm-label" id="pwm-label-left">0</span>
        </div>
        
        <div class="metric" style="margin-top: 15px;">
          <span class="metric-label">Right Motor PWM:</span>
          <span class="metric-value"><span id="pwm-right">0</span></span>
        </div>
        <div class="pwm-bar-container">
          <div class="pwm-bar" id="pwm-bar-right"></div>
          <span class="pwm-label" id="pwm-label-right">0</span>
        </div>
        
        <div class="metric" style="margin-top: 15px;">
          <span class="metric-label">Left Setpoint:</span>
          <span class="metric-value"><span id="set-l">0.000</span><span class="unit">rev/s</span></span>
        </div>
        <div class="metric">
          <span class="metric-label">Right Setpoint:</span>
          <span class="metric-value"><span id="set-r">0.000</span><span class="unit">rev/s</span></span>
        </div>
      </div>
      <!-- Raw Wheel Measurements -->
      <div class="card">
        <h2>📡 Raw Wheel Data</h2>

        <div class="metric">
          <span class="metric-label">Raw Left (rawL):</span>
          <span class="metric-value">
            <span id="raw-l">0</span>
          </span>
        </div>

        <div class="metric">
          <span class="metric-label">Raw Right (rawR):</span>
          <span class="metric-value">
            <span id="raw-r">0</span>
          </span>
        </div>
      </div>
      
      <!-- Visualization -->
      <div class="card visualizer">
        <canvas id="canvas" width="800" height="400"></canvas>
      </div>
    </div>
  </div>

<script>
    const ws = new WebSocket('ws://' + location.host + '/ws');
    const canvas = document.getElementById('canvas');
    const ctx = canvas.getContext('2d');
    
    // Scale factor: pixels per meter (adjust as needed)
    const SCALE = 1000; // 1000 pixels = 1 meter
    const ORIGIN_X = canvas.width / 2;
    const ORIGIN_Y = canvas.height / 2;
    
    ws.onopen = function() {
      console.log('WebSocket connected');
      document.getElementById('status').textContent = 'CONNECTED';
      document.getElementById('status').className = 'status-badge status-connected';
      
      // Send periodic keepalive pings
      setInterval(() => {
        if (ws.readyState === WebSocket.OPEN) {
          ws.send('ping');
        }
      }, 5000); // Ping every 5 seconds
    };
    
    ws.onclose = function() {
      document.getElementById('status').textContent = 'DISCONNECTED';
      document.getElementById('status').className = 'status-badge status-disconnected';
    };
    
    ws.onmessage = function(event) {
      try {
        const data = JSON.parse(event.data);
        updateTelemetry(data);
      } catch (e) {
        console.log('Non-JSON message:', event.data);
      }
    };
    
    function updateTelemetry(data) {
      // Update position
      document.getElementById('pos-x').textContent = data.x.toFixed(3);
      document.getElementById('pos-y').textContent = data.y.toFixed(3);
      document.getElementById('pos-theta').textContent = data.theta.toFixed(1);
      if (data.rawL !== undefined) {
        document.getElementById("raw-l").textContent = data.rawL.toFixed(0);
      }
      if (data.rawR !== undefined) {
        document.getElementById("raw-r").textContent = data.rawR.toFixed(0);
      }
      // Update target
      document.getElementById('target-x').textContent = data.x_s.toFixed(3);
      document.getElementById('target-y').textContent = data.y_s.toFixed(3);
      document.getElementById('target-theta').textContent = data.theta_s.toFixed(1);
      
      // Update navigation state
      document.getElementById('nav-state').textContent = data.state_name;
      
      // Update state description
      const stateDescriptions = {
        'DECIDE_MOTION': '🤔 Analyzing target position to decide whether to drive forward or backward',
        'DRIVE_TO_GOAL': '🚗 Driving towards target position',
        'ROTATE_TO_FINAL': '🔄 Rotating to match target heading angle',
        'GOAL_REACHED': '✅ At goal position - holding steady'
      };
      document.getElementById('state-desc').textContent = stateDescriptions[data.state_name] || 'Unknown state';
      
      // Color code the state
      const stateElement = document.getElementById('nav-state');
      if (data.state_name === 'GOAL_REACHED') {
        stateElement.style.color = '#51cf66';
      } else if (data.state_name === 'DRIVE_TO_GOAL') {
        stateElement.style.color = '#ffd700';
      } else {
        stateElement.style.color = '#fff';
      }

      
      // Update errors with color coding
      const distError = data.error_dist;
      const angleError = Math.abs(data.error_angle);
      
      document.getElementById('error-dist').textContent = distError.toFixed(4);
      document.getElementById('error-dist-cm').textContent = (distError * 100).toFixed(1);
      document.getElementById('error-angle').textContent = data.error_angle.toFixed(1);
      
      // Color code errors
      const distDisplay = document.getElementById('error-dist-display');
      const angleDisplay = document.getElementById('error-angle-display');
      
      if (distError < 0.01) {
        distDisplay.className = 'metric-value error-ok';
      } else {
        distDisplay.className = 'metric-value error-warning';
      }
      
      if (angleError < 5) {
        angleDisplay.className = 'metric-value error-ok';
      } else {
        angleDisplay.className = 'metric-value error-warning';
      }
      
      // Update PWM values
      updatePWM('left', data.pwm_left);
      updatePWM('right', data.pwm_right);
      
      // Update setpoints
      document.getElementById('set-l').textContent = data.set_L.toFixed(3);
      document.getElementById('set-r').textContent = data.set_R.toFixed(3);
      
      // Update visualization
      drawRobot(data);
    }
    
    function updatePWM(side, value) {
      document.getElementById(`pwm-${side}`).textContent = value;
      document.getElementById(`pwm-label-${side}`).textContent = value;
      
      const bar = document.getElementById(`pwm-bar-${side}`);
      const percent = Math.abs(value) / 255 * 100;
      
      bar.style.width = percent + '%';
      bar.className = 'pwm-bar ' + (value >= 0 ? 'positive' : 'negative');
    }
    
    function drawRobot(data) {
      // Clear canvas
      ctx.fillStyle = 'rgba(0, 0, 0, 0.2)';
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      
      // Draw grid
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
      ctx.lineWidth = 1;
      for (let i = -canvas.width; i < canvas.width; i += 50) {
        ctx.beginPath();
        ctx.moveTo(ORIGIN_X + i, 0);
        ctx.lineTo(ORIGIN_X + i, canvas.height);
        ctx.stroke();
        
        ctx.beginPath();
        ctx.moveTo(0, ORIGIN_Y + i);
        ctx.lineTo(canvas.width, ORIGIN_Y + i);
        ctx.stroke();
      }
      
      // Draw axes
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(ORIGIN_X, 0);
      ctx.lineTo(ORIGIN_X, canvas.height);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(0, ORIGIN_Y);
      ctx.lineTo(canvas.width, ORIGIN_Y);
      ctx.stroke();
      
      // Convert robot position to canvas coordinates
      const robotX = ORIGIN_X + data.x * SCALE;
      const robotY = ORIGIN_Y - data.y * SCALE; // Invert Y for screen coordinates
      
      // Convert target position
      const targetX = ORIGIN_X + data.x_s * SCALE;
      const targetY = ORIGIN_Y - data.y_s * SCALE;
      
      // Draw target
      ctx.fillStyle = '#ff6b6b';
      ctx.shadowBlur = 20;
      ctx.shadowColor = '#ff6b6b';
      ctx.beginPath();
      ctx.arc(targetX, targetY, 12, 0, 2 * Math.PI);
      ctx.fill();
      ctx.shadowBlur = 0;
      
      // Draw target heading indicator
      const targetHeadingRad = data.theta_s * Math.PI / 180;
      ctx.strokeStyle = '#ff6b6b';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(targetX, targetY);
      ctx.lineTo(targetX + Math.cos(targetHeadingRad) * 25, 
                 targetY - Math.sin(targetHeadingRad) * 25);
      ctx.stroke();
      
      // Draw robot
      ctx.fillStyle = '#4caf50';
      ctx.shadowBlur = 20;
      ctx.shadowColor = '#4caf50';
      ctx.beginPath();
      ctx.arc(robotX, robotY, 15, 0, 2 * Math.PI);
      ctx.fill();
      ctx.shadowBlur = 0;
      
      // Draw robot heading
      const headingRad = data.theta * Math.PI / 180;
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(robotX, robotY);
      ctx.lineTo(robotX + Math.cos(headingRad) * 30, 
                 robotY - Math.sin(headingRad) * 30);
      ctx.stroke();
      
      // Draw line to target
      ctx.strokeStyle = 'rgba(255, 255, 0, 0.5)';
      ctx.lineWidth = 2;
      ctx.setLineDash([5, 5]);
      ctx.beginPath();
      ctx.moveTo(robotX, robotY);
      ctx.lineTo(targetX, targetY);
      ctx.stroke();
      ctx.setLineDash([]);
    }
    
    // Initial draw
    drawRobot({
      x: 0, y: 0, theta: 90,
      x_s: 0, y_s: 0, theta_s: 0
    });
  </script>
</body>
</html>
      )rawliteral");
  });
  // while (!clientConnected) {
  //     Serial.println("Waiting for WebSocket client...");
  //     delay(100);
  // }
  ws.textAll("HELLO!!! :) Debug line set up");
  // Initialize OTOS sensor
  // ws.textAll("Initializing OTOS sensor...");
  // if (!myOtos.begin()) {
  //   Serial.println("❌ OTOS sensor failed to initialize!");
  //   Serial.println("Check I2C wiring: SDA=" + String(CUSTOM_SDA) + " SCL=" + String(CUSTOM_SCL));
  //   Serial.println("System halted.");
  //   while(1) delay(1000);
  // }
  // Serial.println("✅ OTOS sensor initialized");

  // myOtos.calibrateImu();
  // myOtos.resetTracking();
  // myOtos.setAngularScalar(0.9985437903);
  
  // Wire.beginTransmission(0x0A);
  // Wire.write(0x3A);
  // Wire.write(0x5A);
  // Wire.endTransmission();
  
  Serial.println("Robot Navigation Starting...");

  // Run motor direction test if enabled
  if (MOTOR_DIRECTION_TEST) {
    testMotorDirections();
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
  xTaskCreatePinnedToCore(wsCleanupTask, "WSCleanup", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(encoderTask, "Encoders", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(pidTask, "PID", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(navigationTask, "Nav", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(SerialCommandTask, "Serial", 4096, NULL, 1, NULL, 1);

  Serial.println("✅ All tasks started!");
}

void loop() {
  // All work done in FreeRTOS tasks
}
        //SETPOINT: S x y theta
        //Position: P x y theta
        //Constants: C Kprho Kpalpha Kptheta
        //PID Cons: D Kp Ki Kd
        //Tanh constant: T Scons
//use ws.textAll instead of Serial.println