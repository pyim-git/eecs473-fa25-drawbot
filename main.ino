#define Serial Serial0
// ==== LIBRARIES =====
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Encoder.h>
#include "WifiClient.h"
#include "gantry.h"
#include "pid.h"
#include <string>
#include <math.h>
#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"

// ====== PIN DEFINITIONS ======
#define CUSTOM_SDA 3  
#define CUSTOM_SCL 4
#define ENC_R_A 18
#define ENC_R_B 8
#define ENC_L_A 40
#define ENC_L_B 39
#define BAT 38
#define IN1_R 16
#define IN2_R 17
#define IN3_L 41
#define IN4_L 42
// ====== ROBOT PARAMETERS ======
#define WHEEL_RADIUS 0.0358      // meters
#define WHEEL_BASE   0.246063    // meters
#define TICKS_PER_REV 8400.0
#define LOOP_DT_MS 20

// ====== ENCODER DIRECTION ======
#define ENC_L_DIR 1
#define ENC_R_DIR -1

// ====== GCODE STRUCTURE ======
struct GcodeCommand {
  char command[8];
  float nextX, nextY;
  bool hasX, hasY;
  int color;
}; 

// ====== STEPPER STATE ======
struct Stepper {
  float currentX, currentY;
  float nextX, nextY;
  bool markerDown;
  int color;
}; 

Stepper stepper;

// ====== HARDWARE ======
ESP32Encoder encLeft, encRight;
QwiicOTOS myOtos;
GANTRY gantry;

// ====== PID VARIABLES ======
double set_L = 0.0, input_L = 0.0, output_L = 0.0;
double set_R = 0.0, input_R = 0.0, output_R = 0.0;
volatile double Kp_rho   = 50.0;
volatile double Kp_alpha = 20.0;
volatile double Kp_theta = 80.0;
volatile double Kp = 500.0;
volatile double Ki = 0.01;
volatile double Kd = 0.01;
extern double set_L, input_L, output_L;
extern double set_R, input_R, output_R;
extern volatile bool isRotatingInPlace;
extern volatile int currentNavState;
volatile int currentNavState = 0;
volatile double S_param = 50.0;

// ====== ODOMETRY ======
extern volatile double x = 0, y = 0, theta = M_PI / 2.0;
extern volatile double prevx = 0, calculated_velocity = 0;
extern volatile double x_s = 0.1, y_s = 0.0, theta_s = 0.0;
volatile bool isRotatingInPlace = false;

// ====== FILTERING ======
static double filteredL = 0, filteredR = 0;

// ====== CAMERA DATA ======
float camX = 0.0, camY = 0.0;
volatile bool cam_Valid = false;

// ====== FREERTOS HANDLES ======
SemaphoreHandle_t ExecuteGearSemaphore;
SemaphoreHandle_t ExecuteStepperSemaphore;
SemaphoreHandle_t poseMutex;
SemaphoreHandle_t printMutex;
SemaphoreHandle_t CameraUpdatedSemaphore;
QueueHandle_t queStepper;
QueueHandle_t queGear;
TaskHandle_t taskWeb;

// ====== GCODE BUFFER ======
String gcodeBuffer = "";
volatile bool gcodeReady = false;

// ====== WIFI & WEBSOCKET ======
const char* ssid = "shruti";
const char* password = "shruti05";
AsyncWebSocket ws("/ws");
AsyncWebServer server(80);
bool ledState = false;

// ====== FORWARD DECLARATIONS ======
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len);
             
// ====== UTILITY FUNCTIONS ======
double wrapAngle(double angle) {
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

float getBatteryPercentage() {
  return (analogRead(BAT) / 4095.0) * 100;
}

void ensureMarkerUp() {
  if (stepper.markerDown) {
    gantry.liftMarker();
    vTaskDelay(pdMS_TO_TICKS(500));
    stepper.markerDown = false;
  }
}

void ensureMarkerDown() {
  if (!stepper.markerDown) {
    gantry.lowerMarker();
    vTaskDelay(pdMS_TO_TICKS(500));
    stepper.markerDown = true;
  }
}

// ====== WEBSOCKET HANDLERS ======
void notifyClients() {
  ws.textAll(ledState ? "LED_ON" : "LED_OFF");
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    String msg = (char*)data;
    
    if (msg.startsWith("START_CONNECTION")) {
      ledState = true;
      digitalWrite(LED_BUILTIN, HIGH);
      notifyClients();
    } 
    else if (msg.startsWith("END_CONNECTION")) {
      ledState = false;
      digitalWrite(LED_BUILTIN, LOW);
      notifyClients();
    }
    else if (msg.startsWith("RobotPos")) {
      int xIdx = msg.indexOf("x=");
      int yIdx = msg.indexOf("y=");
      
      if (xIdx >= 0) {
        int end = msg.indexOf(' ', xIdx + 2);
        camX = msg.substring(xIdx + 2, end == -1 ? msg.length() : end).toFloat();
      }
      if (yIdx >= 0) {
        int end = msg.indexOf(' ', yIdx + 2);
        camY = msg.substring(yIdx + 2, end == -1 ? msg.length() : end).toFloat();
      }
      
      cam_Valid = true;
      xSemaphoreGive(CameraUpdatedSemaphore);
    }
    else if (msg.startsWith("STEPPER_GCODE_END")) {
      gcodeReady = true;
      Serial.println("All G-code received. Ready to execute!");
    }
    else {
      gcodeBuffer += msg;
      Serial.println("Received: " + msg);
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
            AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected\n", client->id());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  } else if (type == WS_EVT_DATA) {
    handleWebSocketMessage(arg, data, len);
  }
}

// ====== GCODE PARSING ======
struct GcodeCommand parseGcodeLine(const String& line) {
  struct GcodeCommand cmd;
  memset(&cmd, 0, sizeof(cmd));
  
  if (line.startsWith("G0")) strcpy(cmd.command, "G0");
  else if (line.startsWith("G1")) strcpy(cmd.command, "G1");
  else if (line.startsWith("C ")) {
    strcpy(cmd.command, "C");
    cmd.color = line.substring(2).toInt();
    return cmd;
  }
  else if (line.startsWith("D")) {
    strcpy(cmd.command, "D");
    cmd.nextY = line.substring(1).toFloat();
    cmd.hasY = true;
    return cmd;
  }
  else if (line.startsWith("B")) {
    strcpy(cmd.command, "B");
    return cmd;
  }
  
  int xIdx = line.indexOf('X');
  if (xIdx >= 0) {
    cmd.nextX = line.substring(xIdx + 1).toFloat();
    cmd.hasX = true;
  }
  
  int yIdx = line.indexOf('Y');
  if (yIdx >= 0) {
    cmd.nextY = line.substring(yIdx + 1).toFloat();
    cmd.hasY = true;
  }
  
  return cmd;
}

void TaskGcodeParse(void *pvParameters) {
  static bool isGearGcode = true;
  
  for (;;) {
    int newlinePos = gcodeBuffer.indexOf('\n');
    if (newlinePos < 0) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    
    String line = gcodeBuffer.substring(0, newlinePos);
    line.trim();
    
    if (line.length() == 0) continue;
    
    if (line == "GEAR_GCODE_END") {
      isGearGcode = false;
      Serial.println("Switched to stepper G-code");
      continue;
    }
    
    struct GcodeCommand cmd = parseGcodeLine(line);
    QueueHandle_t targetQueue = isGearGcode ? queGear : queStepper;
    if (targetQueue == NULL) {
        Serial.println("Queue failed to create!");
    }
    if (xQueueSend(targetQueue, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
      Serial.println("Queue full! Dropped: " + line);
    }
    else {
      gcodeBuffer.remove(0, newlinePos + 1); //queued, remove from buffer
      Serial.println("Queued: " + line);
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ====== GEAR EXECUTION ======
void executeGearCommand(const GcodeCommand& cmd) {
  if (strcmp(cmd.command, "G0") == 0) {
    ensureMarkerUp();
  }
  else if (strcmp(cmd.command, "G1") == 0) {
    ensureMarkerDown();
  }
  else if (strcmp(cmd.command, "C") == 0) {
    ensureMarkerUp();
    gantry.switchMarker(stepper.color, cmd.color);
    stepper.color = cmd.color;
    struct GcodeCommand dummy;
    xQueueReceive(queGear, &dummy, 0);
  }
  else if (strcmp(cmd.command, "B") == 0) {
    xSemaphoreTake(ExecuteStepperSemaphore, 0);
    ensureMarkerUp();
    struct GcodeCommand dummy;
    xQueueReceive(queGear, &dummy, 0);
  }
}

void TaskGcodeExec(void *pvParameters) {
  while (!gcodeReady) vTaskDelay(pdMS_TO_TICKS(100));
  
  struct GcodeCommand cmd;
  
  for (;;) {
    xSemaphoreTake(ExecuteGearSemaphore, portMAX_DELAY);
    
    if (xQueuePeek(queGear, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
      xSemaphoreGive(ExecuteGearSemaphore);
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    
    executeGearCommand(cmd);
    
    if (cmd.hasX && cmd.hasY) {
      xSemaphoreTake(poseMutex, portMAX_DELAY);
      x_s = cmd.nextX;
      y_s = cmd.nextY;
      xSemaphoreGive(poseMutex);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    xSemaphoreGive(ExecuteGearSemaphore);
  }
}

// ====== STEPPER EXECUTION ======
void TaskStepperGcodeExec(void *pvParameters) {
  while (!gcodeReady) vTaskDelay(pdMS_TO_TICKS(100));
  
  struct GcodeCommand cmd;
  
  for (;;) {
    xSemaphoreTake(ExecuteStepperSemaphore, portMAX_DELAY);
    
    if (xQueuePeek(queStepper, &cmd, pdMS_TO_TICKS(100)) == pdPASS) {
      Serial.printf("Stepper: %s X:%.2f Y:%.2f\n", cmd.command, cmd.nextX, cmd.nextY);
      
      if (cmd.hasX && cmd.hasY) {
        stepper.currentX = stepper.nextX;
        stepper.currentY = stepper.nextY;
        stepper.nextX = cmd.nextX;
        stepper.nextY = cmd.nextY;
      }
    }
    
    xSemaphoreGive(ExecuteStepperSemaphore);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ====== SERIAL COMMANDS ======
void SerialCommandTask(void *pv) {
  while (true) {
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      
      if (cmd.length() > 0 && (cmd.charAt(0) == 'S' || cmd.charAt(0) == 's')) {
        double newX, newY, thetaDeg;
        if (sscanf(cmd.c_str() + 1, "%lf %lf %lf", &newX, &newY, &thetaDeg) == 3) {
          x_s = newX;
          y_s = newY;
          theta_s = wrapAngle(thetaDeg * M_PI / 180.0);
          Serial.printf("✅ Setpoint: X=%.3f Y=%.3f Θ=%.2f°\n", x_s, y_s, thetaDeg);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ====== ODOMETRY ======
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
    // double rawTheta = myPosition.h * M_PI/180;
    // double rawTheta =  M_PI/180;
    double rawTheta = theta;

    
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

// ====== PID CONTROLLER ======
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

// ====== STEPPER CONTROLLER ======
void stepperTask(void *pvParameters) {
  for (;;) {
    if (stepper.nextX != stepper.currentX) {
      float stepper_velocity = calculated_velocity * 
        (stepper.nextY - stepper.currentY) / (stepper.nextX - stepper.currentX);
      
      gantry.move(stepper.nextY, stepper_velocity);
      
      if (gantry.position == stepper.nextY) {
        struct GcodeCommand cmd;
        xQueueReceive(queStepper, &cmd, pdMS_TO_TICKS(100));
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ====== VELOCITY CALCULATION ======
void velocityTask(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (true) {
    calculated_velocity = (x - prevx) * 200.0;  // 1000/5ms = 200
    prevx = x;
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(5));
  }
}

// ====== GOAL REACHED HANDLER ======
void handleGoalReached() {
  static unsigned long lastCameraCheck = 0;
  const unsigned long CAMERA_INTERVAL = 3000;
  
  xSemaphoreGive(ExecuteGearSemaphore);
  
  if (millis() - lastCameraCheck > CAMERA_INTERVAL) {
    lastCameraCheck = millis();
    
    ws.textAll("CAMERA_REQUEST");
    ws.textAll("BATTERY_STATUS " + String(getBatteryPercentage()) + "%");
    
    if (xSemaphoreTake(CameraUpdatedSemaphore, pdMS_TO_TICKS(5000)) == pdTRUE) {
      if (cam_Valid) {
        float distance = sqrt(pow(camX - x, 2) + pow(camY - y, 2));
        
        if (distance >= 0.10) {  // 10cm tolerance
          ensureMarkerUp();
          x = camX;
          y = camY;
          return;  // Re-navigate to corrected position
        }
      }
    }
  }
  
  xSemaphoreGive(ExecuteStepperSemaphore);
  struct GcodeCommand cmd;
  xQueueReceive(queGear, &cmd, pdMS_TO_TICKS(100));
}

// ====== NAVIGATION ======
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
                    // Serial.printf("[ROTATE_TO_GOAL] Heading: %.1f° | Pos drift: %.3fm | Can transition: %s\n", 
                    //              headingError * 180.0 / M_PI, rho, canChangeState ? "YES" : "NO");
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
        
        vTaskDelay(pdMS_TO_TICKS(LOOP_DT_MS * 2));
    }
}

// ====== WEB SERVER ======
void TaskWebServer(void *pvParameters) {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", "<h1>Robot Server</h1>");
  });
  server.begin();
  Serial.println("Web server started");
  
  for (;;) {
    ws.cleanupClients();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// ====== SETUP ======
void setup() {
  Serial.begin(115200);
  delay(100);

  gantry.init();
  Serial.println("Gantry initialized");
  
  Wire.begin(CUSTOM_SDA, CUSTOM_SCL);
  delay(1000);
  
  pinMode(LED_BUILTIN, OUTPUT);
  
  WifiClient wifi;
  Serial.print("ESP IP: ");
  Serial.println(wifi.connectWiFi(ssid, password));
  
  Serial.println("Initializing OTOS...");
  myOtos.begin();
  Serial.println("✅ OTOS initialized");
  
  myOtos.calibrateImu();
  myOtos.resetTracking();
  myOtos.setAngularScalar(0.9985437903);
  Wire.beginTransmission(0x0A);
  Wire.write(0x3A);
  Wire.write(0x5A);
  Wire.endTransmission();
  // Create synchronization objects
  poseMutex = xSemaphoreCreateMutex();
  printMutex = xSemaphoreCreateMutex();
  CameraUpdatedSemaphore = xSemaphoreCreateBinary();
  ExecuteGearSemaphore = xSemaphoreCreateBinary();
  ExecuteStepperSemaphore = xSemaphoreCreateBinary();
  // battery
  ws.textAll("BATTERY_STATUS " + String(getBatteryPercentage()) + "%");
  if (!poseMutex || !printMutex || !CameraUpdatedSemaphore) {
    Serial.println("❌ Semaphore creation failed!");
    while(1) delay(1000);
  }
  
  queStepper = xQueueCreate(50, sizeof(GcodeCommand));
  queGear = xQueueCreate(50, sizeof(GcodeCommand));
  
  if (!queStepper || !queGear) {
    Serial.println("❌ Queue creation failed!");
    while(1) delay(1000);
  }
  
  setupMotors();
  
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encLeft.attachHalfQuad(ENC_L_A, ENC_L_B);
  encRight.attachHalfQuad(ENC_R_A, ENC_R_B);
  encLeft.clearCount();
  encRight.clearCount();
  
  Serial.println("Starting tasks...");
  delay(2000);
  
  xTaskCreatePinnedToCore(encoderTask, "Encoders", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(pidTask, "PID", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(stepperTask, "Stepper", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(velocityTask, "Velocity", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(navigationTask, "Nav", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(SerialCommandTask, "Serial", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskGcodeParse, "GcodeParse", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskGcodeExec, "GcodeExec", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskStepperGcodeExec, "StepperExec", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskWebServer, "WebServer", 8192, NULL, 1, &taskWeb, 1);
  
  Serial.println("✅ All tasks started!");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}