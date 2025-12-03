#define Serial Serial0
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Encoder.h>
#include "WifiClient.h"
#include "gantry.h"
#include <string>
#include <math.h>
#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"

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

// ====== BAT PINS ======
#define BAT 38

// ====== ROBOT PARAMETERS ======
#define WHEEL_RADIUS 0.0358      // 3.58 cm
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


// ====== ENCODER DIRECTION ======
#define ENC_L_DIR 1   // Try -1 if left encoder backwards
#define ENC_R_DIR 1   // Try -1 if right encoder backwards

// ====== TEST MODE ======
#define ENCODER_TEST_MODE false  // SET TO false WHEN ENCODERS WORK
#define MOTOR_DIRECTION_TEST false  // SET TO true TO TEST MOTOR DIRECTIONS


// ====== CONTROL VARIABLES ======
ESP32Encoder encLeft, encRight;
QwiicOTOS myOtos;
GANTRY gantry;

// PID Variables - NO LIBRARY NEEDED
double set_L = 0.0, input_L = 0.0, output_L = 0.0;
double set_R = 0.0, input_R = 0.0, output_R = 0.0;

// Odometry
volatile double x = 0, y = 0, theta = M_PI / 2.0;
volatile double prevx = 0, calculated_velocity = 0;

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

// 1 - define freeRTOS settings
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif
#define SYSTEM_RUNNING_CORE 0

// ---- Camera postions and time status-----
float camX = 0.0, camY = 0.0;
static TickType_t LastUpdateTime = 0;
const TickType_t TIMEOUT_TICKS = pdMS_TO_TICKS(3000); 

// --- FreeRTOS handles ---
TaskHandle_t OdometryHandle;
TaskHandle_t ControlHandle;
TaskHandle_t taskGcodeParse;
TaskHandle_t taskGcodeExec;
TaskHandle_t taskWeb;
TaskHandle_t taskCamera;
TaskHandle_t taskStepper;
// Declare semaphores
SemaphoreHandle_t ExecuteGearSemaphore;
SemaphoreHandle_t ExecuteStepperSemaphore;
SemaphoreHandle_t ParseGCodeSemaphore;
SemaphoreHandle_t poseMutex;
SemaphoreHandle_t printMutex;
SemaphoreHandle_t CameraUpdatedSemaphore;  

// --- G-code Queue Structure ---
const byte numChars = 128;
struct GcodeCommand {
  char command[8];           // G0, G1, D#, M30
  float nextX, nextY;        // next coordinate destination
  bool hasX;                 // Flag if X is present
  bool hasY;                 // Flag if Y is present
  int color;                 // marker position for drawing (1,2,3)
};

QueueHandle_t queStepper;
QueueHandle_t queGear;

struct Stepper{
  float currentX, currentY;
  float nextX, nextY;
  bool markerDown;
  int color;
};

Stepper stepper;  // global stepper variable - gives gantry information

// --- G-code parsing variables ---
String gcodeBuffer = "";
bool isReceivingGcode = false;
volatile bool gcodeReady = false;

// --- WiFi Credentials ---
const char* ssid = "shruti";
const char* password = "shruti05"; // optionally may use mwireless wifi

// --- WebSocket variables ---
AsyncWebSocket ws("/ws");
AsyncWebServer server(80);
bool ledState = false;

// Forward declarations
void setMotorPWM(int leftPWM, int rightPWM);
void stopMotors();
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len);

// --------------------------------------------------
// WebSocket Functions
// --------------------------------------------------
void notifyClients() {
  ws.textAll(ledState ? "LED_ON" : "LED_OFF"); // add any info we want to send to the server in this format.
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    String msg = (char*)data;
    
    // Add the received line to the buffer
    gcodeBuffer += msg;
    
    // Check if this is a complete G-code command (ends with newline)
    if (msg.endsWith("\n") || msg.endsWith("\r\n")) {
      if (gcodeBuffer.length() > 0) {
        xSemaphoreTake(printMutex, portMAX_DELAY);
        Serial.println("Received: " + gcodeBuffer);
        xSemaphoreGive(printMutex);
      }
    }
    
    if (msg == "START_CONNECTION") {
      ledState = true;
      digitalWrite(LED_BUILTIN, HIGH);
      notifyClients();
    } else if (msg == "END_CONNECTION") {
      ledState = false;
      digitalWrite(LED_BUILTIN, LOW);
      notifyClients();
    }
    else if (msg.startsWith("RobotPos")) {

      // Parse x=
      int xIndex = msg.indexOf("x=");
      if (xIndex >= 0) {
          int start = xIndex + 2;
          int end = msg.indexOf(' ', start);
          String xStr = (end == -1) ? msg.substring(start)
                                    : msg.substring(start, end);
          camX = xStr.toFloat();
      }

      // Parse y=
      int yIndex = msg.indexOf("y=");
      if (yIndex >= 0) {
          int start = yIndex + 2;
          int end = msg.indexOf(' ', start);
          String yStr = (end == -1) ? msg.substring(start)
                                    : msg.substring(start, end);
          camY = yStr.toFloat();
      }

      // Signal camera updated
      xSemaphoreGive(CameraUpdatedSemaphore);
    }
    else if (msg == "STEPPER_GCODE_END") {
      gcodeReady = true;
      xSemaphoreTake(printMutex, portMAX_DELAY);
      Serial.println("All G-code received. Ready to execute!");
      xSemaphoreGive(printMutex);
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
            AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    xSemaphoreTake(printMutex, portMAX_DELAY);
    Serial.printf("WebSocket client #%u connected from %s\n", 
                  client->id(), client->remoteIP().toString().c_str());
    xSemaphoreGive(printMutex);
  } else if (type == WS_EVT_DISCONNECT) {
    xSemaphoreTake(printMutex, portMAX_DELAY);
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
    xSemaphoreGive(printMutex);
  } else if (type == WS_EVT_DATA) {
    handleWebSocketMessage(arg, data, len);
  }
}

// --------------------------------------------------
// G-code Parsing Task
// --------------------------------------------------
void TaskGcodeParse(void *pvParameters) {
  (void)pvParameters;
  
  static bool isGearGcode = true;
  
  for (;;) {
    // Check if websocket has sent new data
    while (gcodeBuffer.length() > 0 && gcodeBuffer.indexOf('\n') >= 0) {
      String current_line = gcodeBuffer.substring(0, gcodeBuffer.indexOf("\n"));
      gcodeBuffer = gcodeBuffer.substring(gcodeBuffer.indexOf("\n") + 1); // take the message we are processing out of gcodeBuffer
      
      if (current_line.length() > 0) {
        xSemaphoreTake(printMutex, portMAX_DELAY);
        Serial.println("Parsing: " + current_line);
        xSemaphoreGive(printMutex);
      } else {
        continue;
      }
      
      // switch to stepper gcode parsing
      if (current_line.startsWith("GEAR_GCODE_END")) {
        xSemaphoreTake(printMutex, portMAX_DELAY);
        Serial.println("Gear Gcode parsing complete. Begin parsing stepper commands");
        xSemaphoreGive(printMutex);
        isGearGcode = false;
        continue; 
      }

      // declare command variable
      GcodeCommand cmd;
      memset(&cmd, 0, sizeof(cmd));
      
      // Parse the line
      if (current_line.startsWith("G0") || current_line.startsWith("G1")) {
      
        // Extract command type
        if (current_line.startsWith("G0")) {
          strcpy(cmd.command, "G0");
        } else {
          strcpy(cmd.command, "G1");
        }
        
        // Parse X coordinate
        int xIndex = current_line.indexOf('X');
        if (xIndex != -1) {
          String xStr = current_line.substring(xIndex + 1);
          int spaceIndex = xStr.indexOf(' ');
          if (spaceIndex != -1) {
            xStr = xStr.substring(0, spaceIndex);
          }
          cmd.nextX = xStr.toFloat();
          cmd.hasX = true;
        }
        
        // Parse Y coordinate
        int yIndex = current_line.indexOf('Y');
        if (yIndex != -1) {
          String yStr = current_line.substring(yIndex + 1);
          int spaceIndex = yStr.indexOf(' ');
          if (spaceIndex != -1) {
            yStr = yStr.substring(0, spaceIndex);
          }
          cmd.nextY = yStr.toFloat();
          cmd.hasY = true;
        }
      }        
      // Handle Color command
      else if (current_line.startsWith("C")) {
        int spaceIndex = current_line.indexOf(' ');
        if (spaceIndex != -1) {
          String currentColor = current_line.substring(spaceIndex + 1);
          currentColor.trim();
          
          strcpy(cmd.command, "C");
          cmd.color = currentColor.toInt();

          xSemaphoreTake(printMutex, portMAX_DELAY);
          Serial.println("Color set to: " + currentColor);
          xSemaphoreGive(printMutex);
        }
      }
      else if (current_line.startsWith("D")) {
        // move down by # of points
        String numStr = current_line.substring(1);
        float points_down = numStr.toFloat();
        strcpy(cmd.command, "D");
        cmd.nextY = points_down; // nextY stores the relative points down
        cmd.hasY = true;
      }

      // Send to execution queue
      if (current_line.startsWith("D") || current_line.startsWith("G0") || current_line.startsWith("G1") || current_line.startsWith("Color")) {
        BaseType_t xStatus;
        if (isGearGcode) {
          xStatus = xQueueSendToBack(queGear, &cmd, pdMS_TO_TICKS(100));
        } else {
          xStatus = xQueueSendToBack(queStepper, &cmd, pdMS_TO_TICKS(100));
        }

        if (xStatus == pdPASS) {
          xSemaphoreTake(printMutex, portMAX_DELAY);
          Serial.printf("Queued: %s X:%.2f Y:%.2f\n", cmd.command, cmd.nextX, cmd.nextY);
          xSemaphoreGive(printMutex);
        } else {
          xSemaphoreTake(printMutex, portMAX_DELAY);
          Serial.println("Queue full! Command dropped.");
          xSemaphoreGive(printMutex);
        }
      }

    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// --------------------------------------------------
// Gear G-code Execution Task
// --------------------------------------------------
void TaskGcodeExec(void *pvParameters) {
  (void)pvParameters;
  while (!gcodeReady) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  
  GcodeCommand receivedGear_cmd;
  BaseType_t xGearStatus;
  const TickType_t xTicksToWait = pdMS_TO_TICKS(100);

  for (;;) {
    xSemaphoreTake(ExecuteGearSemaphore,portMAX_DELAY);
    
    xGearStatus = xQueuePeek(queGear, &receivedGear_cmd, xTicksToWait);
    
    if (xGearStatus == pdPASS) {  
      xSemaphoreTake(printMutex, portMAX_DELAY);
      // Serial.println("=== Executing G-code (gear motor commands) ===");
      // if (strcmp(receivedGear_cmd.command, "C") == 0)
      // {
      //   strcpy(currentColor,receivedGear_cmd.color);
      // }
      // else if (receivedGear_cmd.hasX && receivedGear_cmd.hasY) {
      //   Serial.printf("Robot travel to X: %.2f, Y: %.2f\n", receivedGear_cmd.nextX, receivedGear_cmd.nextY);  
      // }
      // Serial.printf("Color: %s\n", currentColor);
      // Serial.printf("Command: %s\n", receivedGear_cmd.command);
      // if (receivedGear_cmd.hasX && receivedGear_cmd.hasY) {
      //   Serial.printf("Robot travel to X: %.2f, Y: %.2f\n", receivedGear_cmd.nextX, receivedGear_cmd.nextY);  
      // }
      // Serial.println("========================");
      // xSemaphoreGive(printMutex);

      // ========= COMMANDS =========
      // G0: ensure marker is lifted
      if (strcmp(receivedGear_cmd.command, "G0") == 0) {
        if (stepper.markerDown){
          gantry.liftMarker();
          vTaskDelay(pdMS_TO_TICKS(500)); // wait for marker to lift
          stepper.markerDown = false;
        }
      } // if ..G0
      
      // G1: marker should be down
      else if (strcmp(receivedGear_cmd.command, "G1") == 0) {
        if (!stepper.markerDown)
        {
          gantry.lowerMarker();
          vTaskDelay(pdMS_TO_TICKS(500)); // wait for marker to lower
          stepper.markerDown = true;
        }
      } // if ..G1

      // C: switching out color
      else if (strcmp(receivedGear_cmd.command, "C") == 0) {
        if (stepper.markerDown)
        {
          gantry.liftMarker();
          vTaskDelay(pdMS_TO_TICKS(500)); // wait for marker to lift
          stepper.markerDown = false;
        } // if ..marker isn't lifted

        gantry.switchMarker(stepper.color, receivedGear_cmd.color); // switch markers
        stepper.color = receivedGear_cmd.color;
        xGearStatus = xQueueReceive(queGear, &receivedGear_cmd, xTicksToWait); // pop off color commands
      } // if ..C

      // Set target coordinates for the robot
      if (receivedGear_cmd.hasX && receivedGear_cmd.hasY) {
        xSemaphoreTake(poseMutex, portMAX_DELAY);
        x_s = receivedGear_cmd.nextX;  
        y_s = receivedGear_cmd.nextY;
        xSemaphoreGive(poseMutex);
        
        // Wait for robot to reach position
        vTaskDelay(pdMS_TO_TICKS(1000));
      } // set coordinates for robot if command executed
      else {
        // No command in queue, just wait
        vTaskDelay(pdMS_TO_TICKS(10));
      } // else ..no commands
    } // if ..xGearStatus == pdPASS
    xSemaphoreGive(ExecuteGearSemaphore);
  }
}

// --------------------------------------------------
// Stepper G-code Execution Task
// --------------------------------------------------
void TaskStepperGcodeExec(void *pvParameters) {
  (void)pvParameters;
  while (!gcodeReady) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  GcodeCommand receivedStepper_cmd;
  BaseType_t xStepperStatus;
  const TickType_t xTicksToWait = pdMS_TO_TICKS(100);

  for (;;) {
    xSemaphoreTake(ExecuteStepperSemaphore,portMAX_DELAY);

    xStepperStatus = xQueuePeek(queStepper, &receivedStepper_cmd, xTicksToWait);
    
    if (xStepperStatus == pdPASS) {
      xSemaphoreTake(printMutex, portMAX_DELAY);
      Serial.println("=== Executing G-code (stepper motor commands) ===");
      Serial.printf("Command: %s\n", receivedStepper_cmd.command);
      if (receivedStepper_cmd.hasX && receivedStepper_cmd.hasY) {
        // update current stepper position
        stepper.currentY = stepper.nextY;
        stepper.currentX = stepper.nextX;
        // update next stepper position
        stepper.nextX = receivedStepper_cmd.nextX;
        stepper.nextY = receivedStepper_cmd.nextY;
        Serial.printf("Stepper travel to X: %.2f, Y: %.2f\n", receivedStepper_cmd.nextX, receivedStepper_cmd.nextY);  
      }
      Serial.println("========================");
      xSemaphoreGive(printMutex);
    } // if xStepperStatus == pdPass
    xSemaphoreGive(ExecuteStepperSemaphore);
  }
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
} // ..testMotorDirections()

// ====== MOTOR SETUP ======
void setupMotors() {
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(IN3_R, OUTPUT);
  pinMode(IN4_R, OUTPUT);

  ledcAttach(ENA_L, 5000, 8);
  ledcAttach(ENB_R, 5000, 8);
} // ..setupMotors()

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
} // ..remapPID()

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
} // ..SerialCommandTask()

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
} // ..testEncoders()

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
} // encoderTask()

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
   vTaskDelay(pdMS_TO_TICKS(LOOP_DT_MS)
  );
  }
} // ..pidTask()

// ====== STEPPER CONTROLLER ======
void stepperTask(void *pvParameters) {
  (void)pvParameters;
  const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
  for (;;) {
    // checks if stepper queue is empty or full
    UBaseType_t messagesInQueue = uxQueueMessagesWaiting(queStepper);

    // calculate stepper velocity - slope * calc velocity
    float stepper_velocity = calculated_velocity*(stepper.nextY-stepper.currentY)/(stepper.nextX-stepper.currentX);
    
    gantry.move(stepper.nextY, stepper_velocity);
    
    // if the target y is reached
    if ((gantry.position == stepper.nextY))
    {
      // pop off the queue
      GcodeCommand receivedStepper_cmd;
      BaseType_t xStepperStatus = xQueueReceive(queStepper, &receivedStepper_cmd, xTicksToWait);
    } // if ..target x reached  
    vTaskDelay(pdMS_TO_TICKS(10)); // 10 ms, arbitrary
  } // for ..;;
} // ..stepperTask()

// ====== VELOCITY CALCULATION =====
void velocityTask (void *pvParameters) {
  (void)pvParameters;
  const TickType_t period = 5 / portTICK_PERIOD_MS; // 5 ms
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (true) {
      calculated_velocity = (x - prevx)*1000/0.005;
      prevx = x;
      vTaskDelayUntil(&lastWakeTime, period);
  }
} // ... velocityTask()

// ====== NAVIGATION TASK (WITH BACKWARD MOTION) ======
void navigationTask(void *pv) {
    enum State {
        DECIDE_MOTION,      // NEW: deciding what to do next
        DRIVE_TO_GOAL,      // Drive forward or backward to goal
        ROTATE_TO_FINAL,    // Rotate to final orientation
        GOAL_REACHED        // Hold position
    };
    
    // block gear motor commands from executing while gear motor is navigating to target position
    xSemaphoreTake(ExecuteGearSemaphore,portMAX_DELAY);
    
    State currentState = DECIDE_MOTION;
    const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
    GcodeCommand receivedGear_cmd;
    
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
                
                xSemaphoreGive(ExecuteGearSemaphore);
                
                Serial.printf("[STATE 4: GOAL_REACHED] ρ=%.4f (tol %.4f) dθ=%.4f (tol %.4f) | Holding...\n", 
                             rho, goal_tolerance * 3.0, dtheta, angle_tolerance * 3.0);

                // --------------------------------------------------
                // Camera Validates if Goal is Checked
                // --------------------------------------------------
                // requires 3 seconds timeout in between camera updates
                TickType_t currentTime = xTaskGetTickCount();

                // robot arrived at point and timeout is met
                if (currentTime - LastUpdateTime >= TIMEOUT_TICKS) {
                  xSemaphoreTake(printMutex, portMAX_DELAY);
                  Serial.println("Requesting camera check");
                  xSemaphoreGive(printMutex);
                  // sends camera request to webserver
                  ws.textAll("CAMERA_REQUEST");

                  if (xSemaphoreTake(CameraUpdatedSemaphore, pdMS_TO_TICKS(5000)) == pdTRUE) {
                    xSemaphoreTake(printMutex, portMAX_DELAY);
                    Serial.printf("Update camX: %.2f and camY  %.2f\n", camX, camY);
                    xSemaphoreGive(printMutex);
                    LastUpdateTime = xTaskGetTickCount();

                    // check if within tolerance
                    float distance = sqrt(pow((camX - x), 2) + pow((camY - y), 2));

                    if (distance >= 10)
                    {
                      x = camX;
                      y = camY; 
                    }
                    else
                    {
                      // pop the command from the queue
                      xSemaphoreGive(ExecuteGearSemaphore);
                      GcodeCommand receivedGear_cmd;
                      BaseType_t xGearStatus = xQueueReceive(queGear, &receivedGear_cmd, xTicksToWait);
                    }
                
                  }
                }
                
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
} // ..navigationTask()

// ====== SETUP ======
void setup() {
  Serial.begin(115200);
  delay(100);

  // Initialize gantry
  gantry.init();
  Serial.print("Initialization Done!");

  Wire.begin(CUSTOM_SDA, CUSTOM_SCL);
  delay(1000);

  pinMode(LED_BUILTIN, OUTPUT);
  
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


  // Initialize semaphores and queues
  poseMutex = xSemaphoreCreateMutex();
  printMutex = xSemaphoreCreateMutex();
  CameraUpdatedSemaphore = xSemaphoreCreateBinary();
  ExecuteGearSemaphore = xSemaphoreCreateBinary();
  ExecuteStepperSemaphore = xSemaphoreCreateBinary();
  ParseGCodeSemaphore = xSemaphoreCreateBinary();
  
  queStepper = xQueueCreate(10, sizeof(GcodeCommand));
  queGear = xQueueCreate(10, sizeof(GcodeCommand));

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
  xTaskCreatePinnedToCore(stepperTask, "Stepper", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(velocityTask, "CalculateVelocity", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(navigationTask, "Nav", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(SerialCommandTask, "Serial", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskGcodeParse, "GcodeParse", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskGcodeExec, "GcodeExec", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskStepperGcodeExec, "StepperGcodeExec", 4096, NULL, 1, NULL, 0);


  Serial.println("✅ All tasks started!");
} // ..setup()

void loop() {
  // All work done in FreeRTOS tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
} // ..loop