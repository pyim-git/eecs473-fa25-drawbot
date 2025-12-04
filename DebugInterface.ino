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
#include <ArduinoJson.h>

// ====== L298N MOTOR PINS ======
#define IN1_R 16
#define IN2_R 17


#define IN3_L 41
#define IN4_L 42

// ====== ENCODER PINS ======
#define ENC_R_A 18
#define ENC_R_B 8
#define ENC_L_A 40
#define ENC_L_B 39

// ====== I2C PINS ======
#define CUSTOM_SDA 4  
#define CUSTOM_SCL 5  

// ====== ROBOT PARAMETERS ======
#define WHEEL_RADIUS 0.0358  // 3.58 cm
#define WHEEL_BASE   0.246063    // 24.6 cm
#define TICKS_PER_REV 8400.0
#define LOOP_DT_MS 20

extern long currentEncL, currentEncR;
long currentEncL = 0, currentEncR = 0;

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

volatile double Kp_rho   = 50.0;
volatile double Kp_alpha = 20.0;
volatile double Kp_theta = 80.0;
volatile double Kp = 500.0;
volatile double Ki = 0.01;
volatile double Kd = 0.01;
// --- WebSocket Functions ---

// Forward declarations
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len);

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;

  if (!(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)) 
      return;

  String msg = String((char*)data).substring(0, len);
  Serial.println("Got Msg: " + msg);

  DynamicJsonDocument doc(256);
  DeserializationError err = deserializeJson(doc, msg);

  if (err) {
    // If not JSON, treat as command string
    processCommand(msg);
    return;
  }

  String cmd = doc["cmd"];

  if (cmd == "setpoint") {
    double newX = doc["x"];
    double newY = doc["y"];
    double newTdeg = doc["theta"];

    x_s = newX;
    y_s = newY;
    theta_s = wrapAngle(newTdeg * M_PI / 180.0);
    currentNavState = 0;

    Serial.printf("📌 New setpoint via Web: X=%.2f  Y=%.2f  Th=%.1f°\n",
                  x_s, y_s, newTdeg);
  }
  else if (cmd == "command") {
    String cmdStr = doc["text"];
    processCommand(cmdStr);
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
// Replace your sendTelemetry() function with this improved version:
void sendTelemetry() {
  if (ws.count() == 0) return;
  
  static unsigned long lastSendTime = 0;
  unsigned long now = millis();
  if (now - lastSendTime < 100) return; // Max 10 Hz (was 25 Hz)
  lastSendTime = now;
  
  const char* stateNames[] = {"ROTATE_TO_GOAL", "DRIVE_TO_GOAL", "ROTATE_TO_FINAL", "GOAL_REACHED"};
  
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
  
  int pwmL = remapPID(output_L, isRotatingInPlace);
  int pwmR = remapPID(output_R, isRotatingInPlace);
  
  json += "\"pwm_left\":" + String(pwmL) + ",";
  json += "\"pwm_right\":" + String(pwmR) + ",";
  json += "\"set_L\":" + String(set_L, 3) + ",";
  json += "\"set_R\":" + String(set_R, 3) + ",";
  json += "\"enc_left\":" + String(currentEncL) + ",";
  json += "\"enc_right\":" + String(currentEncR) + ",";
  json += "\"state\":" + String(currentNavState) + ",";
  json += "\"state_name\":\"" + String(stateNames[currentNavState]) + "\"";
  json += "}";
  
  ws.textAll(json);
}
// ====== ENCODER DIRECTION ======
#define ENC_L_DIR 1   // Try -1 if left encoder backwards
#define ENC_R_DIR -1   // Try -1 if right encoder backwards


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
double setpointx[4] = {0.0, 0.2, 0.2, 0.0};
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

// Remap PID output to skip deadzone
int remapPID(double pidOutput, bool rotatingInPlace) {
    const int MIN_PWM = 150;
    const int DEAD_PWM = 40;
    const int MAX_PWM = 255;

    double scaledPID = pidOutput;

    if(pidOutput < DEAD_PWM && pidOutput > -DEAD_PWM){
      scaledPID = 0;
    }
    else if (pidOutput < MIN_PWM && pidOutput > DEAD_PWM){
      scaledPID = MIN_PWM;

    }
      else if (pidOutput > -MIN_PWM && pidOutput < -DEAD_PWM){
      scaledPID = -MIN_PWM;

    }
    else if(pidOutput >255){
      scaledPID = 255;
    }
    else if(pidOutput < -255){
      scaledPID = -255;
    }



    // if (scaledPID < -atanh(MIN_PWM/255.0)*S_param || scaledPID > atanh(MIN_PWM/255.0)*S_param) {
    //   scaledPID = tanh(scaledPID/S_param)*255.0;
    // } else if (scaledPID < -atanh(DEAD_PWM/255.0)*S_param) {
    //   scaledPID = -MIN_PWM;
    // } else if (scaledPID > atanh(DEAD_PWM/255.0)*S_param) {
    //   scaledPID = MIN_PWM;
    // } else {
    //   scaledPID = 0;
    // }
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
    ledcWrite(IN3_L, fwd ? 0 : pwm);   // reverse
    ledcWrite(IN4_L, fwd ? pwm : 0);   // forward
  } else {
    ledcWrite(IN1_R, fwd ? pwm : 0);
    ledcWrite(IN2_R, fwd ? 0 : pwm);
  }

  // --- PRINT ---
  // Serial.printf("Wheel: %s   PWM: %d\n",
  //               left ? "LEFT" : "RIGHT",
  //               pwmVal);
}

void stopMotors() {
    ledcWrite(IN1_R, 255);
    ledcWrite(IN2_R, 255);
    ledcWrite(IN3_L, 255);
    ledcWrite(IN4_L, 255);
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
  const double ANGULAR_DRIFT_THRESHOLD = 0.005;
  const unsigned long STILL_TIME_MS = 100;
  
  for (;;) {
    long currL = encLeft.getCount() * ENC_L_DIR;
    long currR = encRight.getCount() * ENC_R_DIR;
    
    // Store current encoder values for telemetry
    currentEncL = currL;
    currentEncR = currR;
    
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
    double rawTheta = myPosition.h * M_PI/180;
    // double rawTheta =  M_PI/180;

    
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



void processCommand(String cmd) {
  cmd.trim();
  
  if (cmd.length() > 0) {
    char cmdType = cmd.charAt(0);
    
    if (cmdType == 'S' || cmdType == 's') {
      double newX, newY, thetaDeg;
      int count = sscanf(cmd.c_str() + 1, "%lf %lf %lf", &newX, &newY, &thetaDeg);
      
      if (count == 3) {
        x_s = newX;
        y_s = newY;
        theta_s = wrapAngle(thetaDeg * M_PI / 180.0);
        currentNavState = 0;
        Serial.printf("✅ New setpoint: X=%.3f m, Y=%.3f m, Theta=%.2f°\n", x_s, y_s, thetaDeg);
        ws.textAll("{\"response\":\"Setpoint updated\"}");
      }
    }
    else if (cmdType == 'C' || cmdType == 'c') {
      double newKp_rho, newKp_alpha, newKp_theta;
      int count = sscanf(cmd.c_str() + 1, "%lf %lf %lf", &newKp_rho, &newKp_alpha, &newKp_theta);
      
      if (count == 3) {
        Kp_rho = newKp_rho;
        Kp_alpha = newKp_alpha;
        Kp_theta = newKp_theta;
        Serial.printf("✅ Controller gains updated\n");
        ws.textAll("{\"response\":\"Controller gains updated\"}");
      }
    }
    else if (cmdType == 'D' || cmdType == 'd') {
      double newKp, newKi, newKd;
      int count = sscanf(cmd.c_str() + 1, "%lf %lf %lf", &newKp, &newKi, &newKd);
      
      if (count == 3) {
        Kp = newKp;
        Ki = newKi;
        Kd = newKd;
        Serial.printf("✅ PID gains updated\n");
        ws.textAll("{\"response\":\"PID gains updated\"}");
      }
    }
    else if (cmdType == 'T' || cmdType == 't') {
      double newS;
      int count = sscanf(cmd.c_str() + 2, "%lf", &newS);
      
      if (count == 1 && newS > 0) {
        S_param = newS;
        Serial.printf("✅ S_param updated to %.3f\n", S_param);
        ws.textAll("{\"response\":\"S_param updated\"}");
      }
    }
    else if (cmdType == 'P' || cmdType == 'p') {
      double newX, newY, thetaDeg;
      int count = sscanf(cmd.c_str() + 1, "%lf %lf %lf", &newX, &newY, &thetaDeg);
      
      if (count == 3) {
        x = newX;
        y = newY;
        theta = wrapAngle(thetaDeg * M_PI / 180.0);
        encLeft.clearCount();
        encRight.clearCount();
        currentNavState = 0;
        Serial.printf("✅ Position reset\n");
        ws.textAll("{\"response\":\"Position reset\"}");
      }
    }
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

// ====== IMPROVED NAVIGATION TASK (ANTI-OSCILLATION) ======
void navigationTask(void *pv) {
    enum State {
        ROTATE_TO_GOAL = 0,
        DRIVE_TO_GOAL = 1,
        ROTATE_TO_FINAL = 2,
        GOAL_REACHED = 3
    };
    
    State currentState = ROTATE_TO_GOAL;
    currentNavState = (int)currentState;

    // ========== TUNING PARAMETERS (IMPROVED) ==========
    // Tolerances
    const double GOAL_TOLERANCE = 0.01;           // Position tolerance (m)
    const double ANGLE_TOLERANCE = 0.05;          // Increased from 0.01 (~2.86° instead of 0.57°)
    const double SEVERE_DRIFT_THRESHOLD = 1.05;   // Increased from 0.52 (~60° instead of 30°)
    const double DRIFT_WARNING_THRESHOLD = 0.35;  // Increased from 0.087 (~20° instead of 5°)
    
    // Velocity limits
    const double MAX_LINEAR_VEL = 0.1;            // m/s
    const double MAX_ANGULAR_VEL = 0.7;           // rad/s
    const double MAX_WHEEL_SPEED = 0.5;           // rev/s
    const double MIN_DRIVE_SPEED = 0.5;           // Increased from 0.3
    
   
    
    // State management with HYSTERESIS to prevent oscillation
    const unsigned long SETTLE_TIME = 500;        // Increased from 300ms
    const unsigned long MIN_STATE_TIME = 1000;    // NEW: Minimum time in each state
    const unsigned long GOAL_HOLD_TIME = 1500;    
    const double DRIFT_MULTIPLIER = 3.0;          
    
    unsigned long stateChangeTime = millis();
    unsigned long lastPrintTime = 0;
    const unsigned long PRINT_INTERVAL = 500;

    // Oscillation prevention
    unsigned long lastStateChange = millis();
    State previousState = ROTATE_TO_GOAL;
    int stateChangeCounter = 0;
    unsigned long stateChangeResetTime = millis();

    for (;;) {
        unsigned long currentTime = millis();
        
        // Detect rapid state oscillation
        if (currentTime - stateChangeResetTime > 5000) {
            stateChangeCounter = 0;
            stateChangeResetTime = currentTime;
        }
        
        // Check for external state reset
        if (currentNavState != (int)currentState) {
            currentState = (State)currentNavState;
            stateChangeTime = currentTime;
            lastStateChange = currentTime;
            Serial.printf("🔄 State externally reset to: %d\n", currentNavState);
        }
        
        // Calculate errors
        double dx = x_s - x;
        double dy = y_s - y;
        double rho = sqrt(dx*dx + dy*dy);
        double targetHeading = atan2(dy, dx);
        double headingError = wrapAngle(targetHeading - theta);
        double finalAngleError = wrapAngle(theta_s - theta);

        double v = 0.0;
        double w = 0.0;

        // Prevent state changes too quickly (hysteresis)
        bool canChangeState = (currentTime - lastStateChange) > MIN_STATE_TIME;

        switch (currentState) {
            
            // ===== STATE 0: ROTATE TO FACE GOAL =====
            case ROTATE_TO_GOAL: {
                v = 0.0;
                w = Kp_theta * headingError;
                
                // More lenient position drift during rotation
                const double ROTATION_DRIFT_TOLERANCE = 0.08; // Increased from 0.05
                
                if (currentTime - lastPrintTime > PRINT_INTERVAL) {
                    Serial.printf("[ROTATE_TO_GOAL] Heading: %.1f° | Pos drift: %.3fm | Can transition: %s\n", 
                                 headingError * 180.0 / M_PI, rho, canChangeState ? "YES" : "NO");
                    lastPrintTime = currentTime;
                }
                
                // Only transition if well-aligned AND enough time has passed
                if (fabs(headingError) < ANGLE_TOLERANCE && canChangeState) {
                    if (currentTime - stateChangeTime > SETTLE_TIME) {
                        currentState = DRIVE_TO_GOAL;
                        currentNavState = (int)currentState;
                        stateChangeTime = currentTime;
                        lastStateChange = currentTime;
                        stateChangeCounter++;
                        Serial.println("✓ Aligned to goal! Starting drive...");
                    }
                } else {
                    stateChangeTime = currentTime;
                }
                break;
            }

            // ===== STATE 1: DRIVE FORWARD WITH CONTINUOUS CORRECTION =====
            case DRIVE_TO_GOAL: {
                // Check for severe drift ONLY if enough time has passed
                if (fabs(headingError) > SEVERE_DRIFT_THRESHOLD && canChangeState) {
                    Serial.printf("⚠️ SEVERE DRIFT: %.1f° → Re-rotating (after %.1fs in DRIVE)\n", 
                                 headingError * 180.0 / M_PI, 
                                 (currentTime - lastStateChange) / 1000.0);
                    currentState = ROTATE_TO_GOAL;
                    currentNavState = (int)currentState;
                    stateChangeTime = currentTime;
                    lastStateChange = currentTime;
                    stateChangeCounter++;
                    stopMotors();
                    vTaskDelay(pdMS_TO_TICKS(200)); // Brief pause
                    break;
                }
                
                // Base velocities with STRONGER heading correction
                v = Kp_rho * rho;
                w = Kp_alpha * headingError;
                
                // Less aggressive speed reduction
                double headingFactor = cos(headingError);
                if (fabs(headingError) > DRIFT_WARNING_THRESHOLD) {
                    double speedScale = max(MIN_DRIVE_SPEED, headingFactor);
                    v *= speedScale;
                    
                    if (currentTime - lastPrintTime > PRINT_INTERVAL) {
                        Serial.printf("[DRIVE] Correcting drift: %.1f° | Speed: %.0f%% | Time in state: %.1fs\n",
                                     headingError * 180.0 / M_PI, speedScale * 100,
                                     (currentTime - lastStateChange) / 1000.0);
                        lastPrintTime = currentTime;
                    }
                }
                
                if (currentTime - lastPrintTime > PRINT_INTERVAL) {
                    Serial.printf("[DRIVE_TO_GOAL] Distance: %.4fm | Heading: %.1f°\n", 
                                 rho, headingError * 180.0 / M_PI);
                    lastPrintTime = currentTime;
                }
                
                // Transition when position reached
                if (rho < GOAL_TOLERANCE) {
                    if (currentTime - stateChangeTime > SETTLE_TIME) {
                        currentState = ROTATE_TO_FINAL;
                        currentNavState = (int)currentState;
                        stateChangeTime = currentTime;
                        lastStateChange = currentTime;
                        Serial.println("✓ Position reached! Rotating to final angle...");
                    }
                } else {
                    stateChangeTime = currentTime;
                }
                break;
            }

            // ===== STATE 2: ROTATE TO FINAL ORIENTATION =====
            case ROTATE_TO_FINAL: {
                v = 0.0;
                w = Kp_theta * finalAngleError;
                
                const double FINAL_ROTATION_DRIFT_TOLERANCE = 0.03; // Increased from 0.02
                if (rho > FINAL_ROTATION_DRIFT_TOLERANCE && canChangeState) {
                    Serial.printf("⚠️ Position drift: %.3fm → Re-approaching\n", rho);
                    currentState = ROTATE_TO_GOAL;
                    currentNavState = (int)currentState;
                    stateChangeTime = currentTime;
                    lastStateChange = currentTime;
                    break;
                }
                
                if (currentTime - lastPrintTime > PRINT_INTERVAL) {
                    Serial.printf("[ROTATE_TO_FINAL] Angle: %.1f° | Pos drift: %.4fm\n", 
                                 finalAngleError * 180.0 / M_PI, rho);
                    lastPrintTime = currentTime;
                }
                
                if (fabs(finalAngleError) < ANGLE_TOLERANCE) {
                    if (currentTime - stateChangeTime > SETTLE_TIME) {
                        currentState = GOAL_REACHED;
                        currentNavState = (int)currentState;
                        stateChangeTime = currentTime;
                        lastStateChange = currentTime;
                        Serial.println("✓✓✓ GOAL REACHED! ✓✓✓");
                    }
                } else {
                    stateChangeTime = currentTime;
                }
                break;
            }

            // ===== STATE 3: GOAL REACHED =====
            case GOAL_REACHED: {
                v = 0.0;
                w = 0.0;
                
                // Monitor for drift
                double positionDriftTolerance = GOAL_TOLERANCE * DRIFT_MULTIPLIER;
                double angleDriftTolerance = ANGLE_TOLERANCE * DRIFT_MULTIPLIER;
                
                if (rho > positionDriftTolerance) {
                    Serial.printf("⚠️ Position drift: %.4fm → Re-approaching goal\n", rho);
                    currentState = ROTATE_TO_GOAL;
                    currentNavState = (int)currentState;
                    stateChangeTime = currentTime;
                    lastStateChange = currentTime;
                } else if (fabs(finalAngleError) > angleDriftTolerance) {
                    Serial.printf("⚠️ Angle drift: %.1f° → Re-orienting\n", 
                                 finalAngleError * 180.0 / M_PI);
                    currentState = ROTATE_TO_FINAL;
                    currentNavState = (int)currentState;
                    stateChangeTime = currentTime;
                    lastStateChange = currentTime;
                }
                break;
            }
        }

        // Detect and warn about oscillation
        if (stateChangeCounter > 5) {
            Serial.println("⚠️⚠️⚠️ WARNING: Rapid state oscillation detected! ⚠️⚠️⚠️");
            Serial.printf("    Consider tuning: KP_ALPHA=%.2f, SEVERE_DRIFT=%.2f°\n",
                         Kp_alpha, SEVERE_DRIFT_THRESHOLD * 180.0 / M_PI);
        }

        // ========== VELOCITY LIMITING & CONVERSION ==========
        v = constrain(v, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
        w = constrain(w, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);

        double vL = (v - w * WHEEL_BASE / 2.0) / (2.0 * M_PI * WHEEL_RADIUS);
        double vR = (v + w * WHEEL_BASE / 2.0) / (2.0 * M_PI * WHEEL_RADIUS);

        vL = constrain(vL, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
        vR = constrain(vR, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);

        if (currentState == GOAL_REACHED) {
            vL = 0.0;
            vR = 0.0;
        }

        set_L = vL;
        set_R = vR;
        
        isRotatingInPlace = (set_L * set_R < 0);

        sendTelemetry();
        
        vTaskDelay(pdMS_TO_TICKS(LOOP_DT_MS * 2));
    }
}


static  char incommon_ca[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIF+TCCA+GgAwIBAgIQRyDQ+oVGGn4XoWQCkYRjdDANBgkqhkiG9w0BAQwFADCB
iDELMAkGA1UEBhMCVVMxEzARBgNVBAgTCk5ldyBKZXJzZXkxFDASBgNVBAcTC0pl
cnNleSBDaXR5MR4wHAYDVQQKExVUaGUgVVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNV
BAMTJVVTRVJUcnVzdCBSU0EgQ2VydGlmaWNhdGlvbiBBdXRob3JpdHkwHhcNMTQx
MDA2MDAwMDAwWhcNMjQxMDA1MjM1OTU5WjB2MQswCQYDVQQGEwJVUzELMAkGA1UE
CBMCTUkxEjAQBgNVBAcTCUFubiBBcmJvcjESMBAGA1UEChMJSW50ZXJuZXQyMREw
DwYDVQQLEwhJbkNvbW1vbjEfMB0GA1UEAxMWSW5Db21tb24gUlNBIFNlcnZlciBD
QTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAJwb8bsvf2MYFVFRVA+e
xU5NEFj6MJsXKZDmMwysE1N8VJG06thum4ltuzM+j9INpun5uukNDBqeso7JcC7v
HgV9lestjaKpTbOc5/MZNrun8XzmCB5hJ0R6lvSoNNviQsil2zfVtefkQnI/tBPP
iwckRR6MkYNGuQmm/BijBgLsNI0yZpUn6uGX6Ns1oytW61fo8BBZ321wDGZq0GTl
qKOYMa0dYtX6kuOaQ80tNfvZnjNbRX3EhigsZhLI2w8ZMA0/6fDqSl5AB8f2IHpT
eIFken5FahZv9JNYyWL7KSd9oX8hzudPR9aKVuDjZvjs3YncJowZaDuNi+L7RyML
fzcCAwEAAaOCAW4wggFqMB8GA1UdIwQYMBaAFFN5v1qqK0rPVIDh2JvAnfKyA2bL
MB0GA1UdDgQWBBQeBaN3j2yW4luHS6a0hqxxAAznODAOBgNVHQ8BAf8EBAMCAYYw
EgYDVR0TAQH/BAgwBgEB/wIBADAdBgNVHSUEFjAUBggrBgEFBQcDAQYIKwYBBQUH
AwIwGwYDVR0gBBQwEjAGBgRVHSAAMAgGBmeBDAECAjBQBgNVHR8ESTBHMEWgQ6BB
hj9odHRwOi8vY3JsLnVzZXJ0cnVzdC5jb20vVVNFUlRydXN0UlNBQ2VydGlmaWNh
dGlvbkF1dGhvcml0eS5jcmwwdgYIKwYBBQUHAQEEajBoMD8GCCsGAQUFBzAChjNo
dHRwOi8vY3J0LnVzZXJ0cnVzdC5jb20vVVNFUlRydXN0UlNBQWRkVHJ1c3RDQS5j
cnQwJQYIKwYBBQUHMAGGGWh0dHA6Ly9vY3NwLnVzZXJ0cnVzdC5jb20wDQYJKoZI
hvcNAQEMBQADggIBAC0RBjjW29dYaK+qOGcXjeIT16MUJNkGE+vrkS/fT2ctyNMU
11ZlUp5uH5gIjppIG8GLWZqjV5vbhvhZQPwZsHURKsISNrqOcooGTie3jVgU0W+0
+Wj8mN2knCVANt69F2YrA394gbGAdJ5fOrQmL2pIhDY0jqco74fzYefbZ/VS29fR
5jBxu4uj1P+5ZImem4Gbj1e4ZEzVBhmO55GFfBjRidj26h1oFBHZ7heDH1Bjzw72
hipu47Gkyfr2NEx3KoCGMLCj3Btx7ASn5Ji8FoU+hCazwOU1VX55mKPU1I2250Lo
RCASN18JyfsD5PVldJbtyrmz9gn/TKbRXTr80U2q5JhyvjhLf4lOJo/UzL5WCXED
Smyj4jWG3R7Z8TED9xNNCxGBMXnMete+3PvzdhssvbORDwBZByogQ9xL2LUZFI/i
eoQp0UM/L8zfP527vWjEzuDN5xwxMnhi+vCToh7J159o5ah29mP+aJnvujbXEnGa
nrNxHzu+AGOePV8hwrGGG7hOIcPDQwkuYwzN/xT29iLp/cqf9ZhEtkGcQcIImH3b
oJ8ifsCnSbu0GB9L06Yqh7lcyvKDTEADslIaeSEINxhO2Y1fmcYFX/Fqrrp1WnhH
OjplXuXE0OPa0utaKC25Aplgom88L2Z8mEWcyfoB7zKOfD759AN7JKZWCYwk
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIF3jCCA8agAwIBAgIQAf1tMPyjylGoG7xkDjUDLTANBgkqhkiG9w0BAQwFADCB
iDELMAkGA1UEBhMCVVMxEzARBgNVBAgTCk5ldyBKZXJzZXkxFDASBgNVBAcTC0pl
cnNleSBDaXR5MR4wHAYDVQQKExVUaGUgVVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNV
BAMTJVVTRVJUcnVzdCBSU0EgQ2VydGlmaWNhdGlvbiBBdXRob3JpdHkwHhcNMTAw
MjAxMDAwMDAwWhcNMzgwMTE4MjM1OTU5WjCBiDELMAkGA1UEBhMCVVMxEzARBgNV
BAgTCk5ldyBKZXJzZXkxFDASBgNVBAcTC0plcnNleSBDaXR5MR4wHAYDVQQKExVU
aGUgVVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNVBAMTJVVTRVJUcnVzdCBSU0EgQ2Vy
dGlmaWNhdGlvbiBBdXRob3JpdHkwggIiMA0GCSqGSIb3DQEBAQUAA4ICDwAwggIK
AoICAQCAEmUXNg7D2wiz0KxXDXbtzSfTTK1Qg2HiqiBNCS1kCdzOiZ/MPans9s/B
3PHTsdZ7NygRK0faOca8Ohm0X6a9fZ2jY0K2dvKpOyuR+OJv0OwWIJAJPuLodMkY
tJHUYmTbf6MG8YgYapAiPLz+E/CHFHv25B+O1ORRxhFnRghRy4YUVD+8M/5+bJz/
Fp0YvVGONaanZshyZ9shZrHUm3gDwFA66Mzw3LyeTP6vBZY1H1dat//O+T23LLb2
VN3I5xI6Ta5MirdcmrS3ID3KfyI0rn47aGYBROcBTkZTmzNg95S+UzeQc0PzMsNT
79uq/nROacdrjGCT3sTHDN/hMq7MkztReJVni+49Vv4M0GkPGw/zJSZrM233bkf6
c0Plfg6lZrEpfDKEY1WJxA3Bk1QwGROs0303p+tdOmw1XNtB1xLaqUkL39iAigmT
Yo61Zs8liM2EuLE/pDkP2QKe6xJMlXzzawWpXhaDzLhn4ugTncxbgtNMs+1b/97l
c6wjOy0AvzVVdAlJ2ElYGn+SNuZRkg7zJn0cTRe8yexDJtC/QV9AqURE9JnnV4ee
UB9XVKg+/XRjL7FQZQnmWEIuQxpMtPAlR1n6BB6T1CZGSlCBst6+eLf8ZxXhyVeE
Hg9j1uliutZfVS7qXMYoCAQlObgOK6nyTJccBz8NUvXt7y+CDwIDAQABo0IwQDAd
BgNVHQ4EFgQUU3m/WqorSs9UgOHYm8Cd8rIDZsswDgYDVR0PAQH/BAQDAgEGMA8G
A1UdEwEB/wQFMAMBAf8wDQYJKoZIhvcNAQEMBQADggIBAFzUfA3P9wF9QZllDHPF
Up/L+M+ZBn8b2kMVn54CVVeWFPFSPCeHlCjtHzoBN6J2/FNQwISbxmtOuowhT6KO
VWKR82kV2LyI48SqC/3vqOlLVSoGIG1VeCkZ7l8wXEskEVX/JJpuXior7gtNn3/3
ATiUFJVDBwn7YKnuHKsSjKCaXqeYalltiz8I+8jRRa8YFWSQEg9zKC7F4iRO/Fjs
8PRF/iKz6y+O0tlFYQXBl2+odnKPi4w2r78NBc5xjeambx9spnFixdjQg3IM8WcR
iQycE0xyNN+81XHfqnHd4blsjDwSXWXavVcStkNr/+XeTWYRUc+ZruwXtuhxkYze
Sf7dNXGiFSeUHM9h4ya7b6NnJSFd5t0dCy5oGzuCr+yDZ4XUmFF0sbmZgIn/f3gZ
XHlKYC6SQK5MNyosycdiyA5d9zZbyuAlJQG03RoHnHcAP9Dc1ew91Pq7P8yF1m9/
qS3fuQL39ZeatTXaw2ewh0qpKJ4jjv9cJ2vhsE/zB+4ALtRZh8tSQZXq9EfX7mRB
VXyNWQKV3WKdwrnuWih0hKWbt5DHDAff9Yk2dDLWKMGwsAvgnEzDHNb842m1R0aB
L6KCq9NjRHDEjf8tM7qtj3u1cIiuPhnPQCjY/MiQu12ZIvVS5ljFH4gxQ+6IHdfG
jjxDah2nGN59PRbxYvnKkKj9
-----END CERTIFICATE-----
)EOF";

// ====== SETUP ======
// ====== COMPLETE SETUP FUNCTION ======
void setup() {
  Serial.begin(115200);
  delay(100);

  Wire.begin(CUSTOM_SDA, CUSTOM_SCL);
  delay(1000);
  
  // ====== WiFi Setup ======
  WifiClient wifi;
  Serial.print("ESP IP Address: ");
  Serial.println(wifi.connectWiFi("athomare@umich.edu", "Aryan2004Michig!!", incommon_ca));

  // ====== WebSocket & Web Server Setup ======
  
  // Handle favicon to prevent 404 errors
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(204); // No content
  });
  
  // Main web page
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
    
    .input-group {
      margin: 15px 0;
    }
    
    .input-group label {
      display: block;
      margin-bottom: 8px;
      font-weight: 600;
      color: #e0e0e0;
    }
    
    .input-group input {
      width: 100%;
      padding: 12px;
      border-radius: 8px;
      border: 2px solid rgba(255, 255, 255, 0.2);
      background: rgba(0, 0, 0, 0.3);
      color: #fff;
      font-size: 1em;
      font-family: 'Courier New', monospace;
    }
    
    .input-group input:focus {
      outline: none;
      border-color: #ffd700;
    }
    
    button {
      width: 100%;
      padding: 12px;
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      color: white;
      border: none;
      border-radius: 8px;
      font-size: 1.1em;
      font-weight: bold;
      cursor: pointer;
      transition: transform 0.2s;
      margin-top: 10px;
    }
    
    button:hover {
      transform: translateY(-2px);
      box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
    }
    
    button:active {
      transform: translateY(0);
    }
    
    .command-output {
      margin-top: 10px;
      padding: 10px;
      background: rgba(0, 0, 0, 0.3);
      border-radius: 8px;
      min-height: 80px;
      max-height: 150px;
      overflow-y: auto;
      font-family: 'Courier New', monospace;
      font-size: 0.9em;
    }
    
    .command-output div {
      padding: 4px 0;
      border-bottom: 1px solid rgba(255, 255, 255, 0.1);
    }
    
    .help-text {
      font-size: 0.85em;
      color: #b0b0b0;
      margin-top: 8px;
      line-height: 1.4;
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
      
      <!-- Navigation State Card -->
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
      
      <!-- Encoder Values Card -->
      <div class="card">
        <h2>🔢 Encoder Values</h2>
        <div class="metric">
          <span class="metric-label">Left Encoder:</span>
          <span class="metric-value"><span id="enc-left">0</span><span class="unit">ticks</span></span>
        </div>
        <div class="metric">
          <span class="metric-label">Right Encoder:</span>
          <span class="metric-value"><span id="enc-right">0</span><span class="unit">ticks</span></span>
        </div>
        <div class="metric">
          <span class="metric-label">Left Velocity:</span>
          <span class="metric-value"><span id="vel-left">0.000</span><span class="unit">rev/s</span></span>
        </div>
        <div class="metric">
          <span class="metric-label">Right Velocity:</span>
          <span class="metric-value"><span id="vel-right">0.000</span><span class="unit">rev/s</span></span>
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
          <span class="metric-value"><span id="raw-l">0</span></span>
        </div>
        <div class="metric">
          <span class="metric-label">Raw Right (rawR):</span>
          <span class="metric-value"><span id="raw-r">0</span></span>
        </div>
      </div>

      <!-- Set Target Interface -->
      <div class="card">
        <h2>🎮 Set New Target</h2>
        <div class="input-group">
          <label>X (m):</label>
          <input id="inputX" type="number" step="0.01" placeholder="0.00">
        </div>
        <div class="input-group">
          <label>Y (m):</label>
          <input id="inputY" type="number" step="0.01" placeholder="0.00">
        </div>
        <div class="input-group">
          <label>Theta (°):</label>
          <input id="inputTheta" type="number" step="1" placeholder="0">
        </div>
        <button onclick="sendSetpoint()">Send Setpoint</button>
      </div>

      <!-- Command Input Card -->
      <div class="card">
        <h2>⌨️ Serial Command Input</h2>
        <div class="input-group">
          <label>Enter Command:</label>
          <input id="commandInput" type="text" placeholder="S 0.2 0.2 45" onkeypress="if(event.key==='Enter')sendCommand()">
        </div>
        <button onclick="sendCommand()">Send Command</button>
        <div class="help-text">
          <strong>Available Commands:</strong><br>
          • S x y theta - Set setpoint<br>
          • P x y theta - Reset position<br>
          • C Kp_rho Kp_alpha Kp_theta - Set controller gains<br>
          • D Kp Ki Kd - Set PID gains<br>
          • T S_value - Set tanh parameter
        </div>
        <div class="command-output" id="commandOutput">
          <div style="color: #888;">Command responses will appear here...</div>
        </div>
      </div>
      
      <!-- Visualization -->
      <div class="card visualizer">
        <canvas id="canvas" width="800" height="400"></canvas>
      </div>
    </div>
  </div>

  <script>
    // ========== IMPROVED WEBSOCKET CONNECTION ==========
    
    const protocol = window.location.protocol === 'https:' ? 'wss://' : 'ws://';
    const host = window.location.hostname;
    const port = window.location.port || '80';
    const wsUrl = protocol + host + (port ? ':' + port : '') + '/ws';
    
    console.log('🔌 Connecting to WebSocket at:', wsUrl);
    
    let ws;
    let reconnectInterval;
    let reconnectAttempts = 0;
    const MAX_RECONNECT_ATTEMPTS = 10;
    
    function connectWebSocket() {
      try {
        ws = new WebSocket(wsUrl);
        
        ws.onopen = function() {
          console.log('✅ WebSocket connected successfully');
          reconnectAttempts = 0;
          document.getElementById('status').textContent = 'CONNECTED';
          document.getElementById('status').className = 'status-badge status-connected';
          
          if (reconnectInterval) {
            clearInterval(reconnectInterval);
          }
          reconnectInterval = setInterval(() => {
            if (ws.readyState === WebSocket.OPEN) {
              ws.send('ping');
            }
          }, 5000);
        };
        
        ws.onclose = function(event) {
          console.log('❌ WebSocket disconnected. Code:', event.code, 'Reason:', event.reason || 'No reason given');
          document.getElementById('status').textContent = 'DISCONNECTED';
          document.getElementById('status').className = 'status-badge status-disconnected';
          
          if (reconnectInterval) {
            clearInterval(reconnectInterval);
          }
          
          if (reconnectAttempts < MAX_RECONNECT_ATTEMPTS) {
            reconnectAttempts++;
            console.log(`🔄 Reconnecting... Attempt ${reconnectAttempts}/${MAX_RECONNECT_ATTEMPTS}`);
            setTimeout(connectWebSocket, 2000);
          } else {
            console.error('❌ Max reconnection attempts reached. Please refresh the page.');
            addCommandOutput('❌ Connection lost. Please refresh the page.');
          }
        };
        
        ws.onerror = function(error) {
          console.error('⚠️ WebSocket error:', error);
          addCommandOutput('⚠️ WebSocket error occurred');
        };
        
        ws.onmessage = function(event) {
          try {
            const data = JSON.parse(event.data);
            if (data.response) {
              addCommandOutput(data.response);
            } else {
              updateTelemetry(data);
            }
          } catch (e) {
            console.log('📨 Non-JSON message:', event.data);
          }
        };
        
      } catch (error) {
        console.error('❌ Failed to create WebSocket:', error);
        addCommandOutput('❌ Failed to connect: ' + error.message);
      }
    }
    
    connectWebSocket();
    
    const canvas = document.getElementById('canvas');
    const ctx = canvas.getContext('2d');
    
    const SCALE = 1000;
    const ORIGIN_X = canvas.width / 2;
    const ORIGIN_Y = canvas.height / 2;

    function sendSetpoint() {
      let x = parseFloat(document.getElementById("inputX").value);
      let y = parseFloat(document.getElementById("inputY").value);
      let theta = parseFloat(document.getElementById("inputTheta").value);

      if (isNaN(x) || isNaN(y) || isNaN(theta)) {
        addCommandOutput("❌ Invalid input values");
        return;
      }

      let msg = {
        cmd: "setpoint",
        x: x,
        y: y,
        theta: theta
      };

      ws.send(JSON.stringify(msg));
      addCommandOutput(`✅ Setpoint sent: X=${x}, Y=${y}, θ=${theta}°`);
    }

    function sendCommand() {
      const input = document.getElementById("commandInput");
      const command = input.value.trim();
      
      if (command === "") {
        addCommandOutput("❌ Please enter a command");
        return;
      }

      const msg = {
        cmd: "command",
        text: command
      };

      ws.send(JSON.stringify(msg));
      addCommandOutput(`📤 Sent: ${command}`);
      input.value = "";
    }

    function addCommandOutput(text) {
      const output = document.getElementById("commandOutput");
      const div = document.createElement("div");
      div.textContent = text;
      output.appendChild(div);
      output.scrollTop = output.scrollHeight;
      
      while (output.children.length > 10) {
        output.removeChild(output.firstChild);
      }
    }
    
    function updateTelemetry(data) {
      document.getElementById('pos-x').textContent = data.x.toFixed(5);
      document.getElementById('pos-y').textContent = data.y.toFixed(5);
      document.getElementById('pos-theta').textContent = data.theta.toFixed(1);
      
      if (data.enc_left !== undefined) {
        document.getElementById("enc-left").textContent = data.enc_left;
      }
      if (data.enc_right !== undefined) {
        document.getElementById("enc-right").textContent = data.enc_right;
      }
      
      if (data.set_L !== undefined) {
        document.getElementById("vel-left").textContent = data.set_L.toFixed(3);
      }
      if (data.set_R !== undefined) {
        document.getElementById("vel-right").textContent = data.set_R.toFixed(3);
      }
      
      if (data.rawL !== undefined) {
        document.getElementById("raw-l").textContent = data.rawL.toFixed(0);
      }
      if (data.rawR !== undefined) {
        document.getElementById("raw-r").textContent = data.rawR.toFixed(0);
      }
      
      document.getElementById('target-x').textContent = data.x_s.toFixed(3);
      document.getElementById('target-y').textContent = data.y_s.toFixed(3);
      document.getElementById('target-theta').textContent = data.theta_s.toFixed(1);
      
      document.getElementById('nav-state').textContent = data.state_name;
      
      const stateDescriptions = {
        'ROTATE_TO_GOAL': '🔄 Rotating to face the target position',
        'DRIVE_TO_GOAL': '🚗 Driving towards target position',
        'ROTATE_TO_FINAL': '🔄 Rotating to match target heading angle',
        'GOAL_REACHED': '✅ At goal position - holding steady'
      };
      document.getElementById('state-desc').textContent = stateDescriptions[data.state_name] || 'Unknown state';
      
      const stateElement = document.getElementById('nav-state');
      if (data.state_name === 'GOAL_REACHED') {
        stateElement.style.color = '#51cf66';
      } else if (data.state_name === 'DRIVE_TO_GOAL') {
        stateElement.style.color = '#ffd700';
      } else {
        stateElement.style.color = '#fff';
      }
      
      const distError = data.error_dist;
      const angleError = Math.abs(data.error_angle);
      
      document.getElementById('error-dist').textContent = distError.toFixed(4);
      document.getElementById('error-dist-cm').textContent = (distError * 100).toFixed(1);
      document.getElementById('error-angle').textContent = data.error_angle.toFixed(1);
      
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
      
      updatePWM('left', data.pwm_left);
      updatePWM('right', data.pwm_right);
      
      document.getElementById('set-l').textContent = data.set_L.toFixed(3);
      document.getElementById('set-r').textContent = data.set_R.toFixed(3);
      
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
      ctx.fillStyle = 'rgba(0, 0, 0, 0.2)';
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      
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
      
      const robotX = ORIGIN_X + data.x * SCALE;
      const robotY = ORIGIN_Y - data.y * SCALE;
      
      const targetX = ORIGIN_X + data.x_s * SCALE;
      const targetY = ORIGIN_Y - data.y_s * SCALE;
      
      ctx.fillStyle = '#ff6b6b';
      ctx.shadowBlur = 20;
      ctx.shadowColor = '#ff6b6b';
      ctx.beginPath();
      ctx.arc(targetX, targetY, 12, 0, 2 * Math.PI);
      ctx.fill();
      ctx.shadowBlur = 0;
      
      const targetHeadingRad = data.theta_s * Math.PI / 180;
      ctx.strokeStyle = '#ff6b6b';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(targetX, targetY);
      ctx.lineTo(targetX + Math.cos(targetHeadingRad) * 25, 
                 targetY - Math.sin(targetHeadingRad) * 25);
      ctx.stroke();
      
      ctx.fillStyle = '#4caf50';
      ctx.shadowBlur = 20;
      ctx.shadowColor = '#4caf50';
      ctx.beginPath();
      ctx.arc(robotX, robotY, 15, 0, 2 * Math.PI);
      ctx.fill();
      ctx.shadowBlur = 0;
      
      const headingRad = data.theta * Math.PI / 180;
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(robotX, robotY);
      ctx.lineTo(robotX + Math.cos(headingRad) * 30, 
                 robotY - Math.sin(headingRad) * 30);
      ctx.stroke();
      
      ctx.strokeStyle = 'rgba(255, 255, 0, 0.5)';
      ctx.lineWidth = 2;
      ctx.setLineDash([5, 5]);
      ctx.beginPath();
      ctx.moveTo(robotX, robotY);
      ctx.lineTo(targetX, targetY);
      ctx.stroke();
      ctx.setLineDash([]);
    }
    
    drawRobot({
      x: 0, y: 0, theta: 90,
      x_s: 0, y_s: 0, theta_s: 0
    });
  </script>
</body>
</html>
    )rawliteral");
  });
  
  // Setup WebSocket handlers
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.begin();
  
  Serial.println("🌐 Web server started!");
  Serial.println("📱 Connect to the IP address shown above");

  // ====== Initialize OTOS Sensor ======
  Serial.println("🔧 Initializing OTOS sensor...");
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
  
  Serial.println("🤖 Robot Navigation Starting...");

  // ====== Setup Motors ======
  setupMotors();

  // ====== Setup Encoders ======
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encLeft.attachHalfQuad(ENC_L_A, ENC_L_B);
  encRight.attachHalfQuad(ENC_R_A, ENC_R_B);
  
  encLeft.clearCount();
  encRight.clearCount();

  Serial.printf("🎯 Initial Target: x=%.2f, y=%.2f, theta=%.2f\n", x_s, y_s, theta_s);
  Serial.println("⏱️  Starting tasks in 2 seconds...");
  delay(2000);

  // ====== Start FreeRTOS Tasks ======
  xTaskCreatePinnedToCore(encoderTask, "Encoders", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(pidTask, "PID", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(navigationTask, "Nav", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(SerialCommandTask, "Serial", 4096, NULL, 1, NULL, 1);

  Serial.println("✅ All tasks started!");
  Serial.println("📊 Telemetry streaming at 10 Hz");
  Serial.println("💾 Free heap: " + String(ESP.getFreeHeap()) + " bytes");
}

// Replace your empty loop() with this:
void loop() {
  // Clean up disconnected WebSocket clients
  ws.cleanupClients();
  delay(10);
}
        //SETPOINT: S x y theta
        //Position: P x y theta
        //Constants: C Kprho Kpalpha Kptheta
        //PID Cons: D Kp Ki Kd
        //Tanh constant: T Scons
//use ws.textAll instead of Serial.println