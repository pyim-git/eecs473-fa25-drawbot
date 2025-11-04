#define Serial Serial0
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <PID_v1.h>
#include <ESP32Encoder.h>
#include <WifiClient.h>
#include <string>


// 1 - define freeRTOS settings
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif
#define SYSTEM_RUNNING_CORE 0

// --- Robot geometry ---
const float R = 0.05;   // Wheel radius (m)
const float L = 0.15;   // Distance between wheels (m)
const int ticksPerRev = 16800;

// --- Encoder pins ---
#define ENC_L_A 11
#define ENC_L_B 12
#define ENC_R_A 13
#define ENC_R_B 14

// --- Motor pins ---
#define ENA 5
#define IN1 6
#define IN2 7
#define ENB 8
#define IN3 9
#define IN4 10

// --- Global states (shared) ---
volatile long encL = 0, encR = 0;
float x = 0, y = 0, theta = 0;
volatile float velL = 0, velR = 0;

// --- Navigation target ---
float x_t = 1.0, y_t = 0.0;

// --- PID control variables ---
double inputL, inputR, outputL, outputR;
double setL, setR;

PID pidL(&inputL, &outputL, &setL, 20, 5, 0.1, DIRECT);
PID pidR(&inputR, &outputR, &setR, 20, 5, 0.1, DIRECT);

// --- FreeRTOS handles ---
TaskHandle_t OdometryHandle;
TaskHandle_t ControlHandle;
TaskHandle_t taskGcodeParse;
TaskHandle_t taskGcodeExec;
TaskHandle_t taskWeb;
SemaphoreHandle_t poseMutex;
SemaphoreHandle_t printMutex;

// --- G-code Queue Structure ---
const byte numChars = 128;
struct GcodeCommand {
  char command[8];      // G0, G1, M3, M5, etc.
  float X;              // X coordinate
  float Y;              // Y coordinate
  bool hasX;            // Flag if X is present
  bool hasY;            // Flag if Y is present
  char color[16];       // Color for drawing
};

QueueHandle_t queGcode;

// --- G-code parsing variables ---
String gcodeBuffer = "";
bool isReceivingGcode = false;
String currentColor = "blue";
volatile bool gcodeReady = false;

// --- Create Encoder objects ---
ESP32Encoder encLeft;
ESP32Encoder encRight;

// --- WiFi Credentials ---
const char* ssid = "shruti";
const char* password = "shruti05";

// --- WebSocket variables ---
// WifiClient webserver;
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
  ws.textAll(ledState ? "LED_ON" : "LED_OFF");
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
      //gcodeBuffer.trim();
      
      // Send to parsing task via queue
      if (gcodeBuffer.length() > 0) {
        // For now, just store in a global for the parser task to pick up
        // The parser task will handle it
        xSemaphoreTake(printMutex, portMAX_DELAY);
        Serial.println("Received: " + gcodeBuffer);
        xSemaphoreGive(printMutex);
      }
      
      //gcodeBuffer = ""; // Clear buffer for next command
    }
    
    // Legacy commands for testing
    if (msg == "START_CONNECTION") {
      ledState = true;
      digitalWrite(LED_BUILTIN, HIGH);
      notifyClients();
    } else if (msg == "END_CONNECTION") {
      ledState = false;
      digitalWrite(LED_BUILTIN, LOW);
      notifyClients();
    }
    else if (msg == "GCODE_END") {
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
  
  static String receivedLine = "";
  static bool newLine = false;
  
  for (;;) {
    // Check if websocket has sent new data
    if (gcodeBuffer.length() > 0) {
      //receivedLine = gcodeBuffer;
      //gcodeBuffer = "";
      newLine = true;
    }
    
    while (newLine) {
      //receivedLine.trim();
      
      xSemaphoreTake(printMutex, portMAX_DELAY);
      String currentLine = gcodeBuffer.substring(0,gcodeBuffer.indexOf("\n"));
      Serial.println("Parsing: " + currentLine);
      xSemaphoreGive(printMutex);
      
      // Parse the line
      if (gcodeBuffer.startsWith("G0") || gcodeBuffer.startsWith("G1")) {
        GcodeCommand cmd;
        memset(&cmd, 0, sizeof(cmd));
        
        // Extract command type
        if (gcodeBuffer.startsWith("G0")) {
          strcpy(cmd.command, "G0");
        } else {
          strcpy(cmd.command, "G1");
        }
        
        // Parse X coordinate
        int xIndex = gcodeBuffer.indexOf('X');
        if (xIndex != -1) {
          String xStr = gcodeBuffer.substring(xIndex + 1);
          // Find next space or end
          int spaceIndex = xStr.indexOf(' ');
          if (spaceIndex != -1) {
            xStr = xStr.substring(0, spaceIndex);
          }
          cmd.X = xStr.toFloat();
          cmd.hasX = true;
        }
        
        // Parse Y coordinate
        int yIndex = gcodeBuffer.indexOf('Y');
        if (yIndex != -1) {
          String yStr = gcodeBuffer.substring(yIndex + 1);
          // Find next space or end
          int spaceIndex = yStr.indexOf(' ');
          if (spaceIndex != -1) {
            yStr = yStr.substring(0, spaceIndex);
          }
          cmd.Y = yStr.toFloat();
          cmd.hasY = true;
        }
        
        // Add current color
        strcpy(cmd.color, currentColor.c_str());
        
        // Send to execution queue
        BaseType_t xStatus = xQueueSendToBack(queGcode, &cmd, pdMS_TO_TICKS(100));
        
        if (xStatus == pdPASS) {
          xSemaphoreTake(printMutex, portMAX_DELAY);
          Serial.printf("Queued: %s X:%.2f Y:%.2f\n", cmd.command, cmd.X, cmd.Y);
          xSemaphoreGive(printMutex);
        } else {
          xSemaphoreTake(printMutex, portMAX_DELAY);
          Serial.println("Queue full! Command dropped.");
          xSemaphoreGive(printMutex);
        }
      }
      // Handle Color command
      else if (gcodeBuffer.startsWith("Color")) {
        int spaceIndex = gcodeBuffer.indexOf(' ');
        if (spaceIndex != -1) {
          currentColor = gcodeBuffer.substring(spaceIndex + 1, gcodeBuffer.indexOf("\n"));
          currentColor.trim();
          
          xSemaphoreTake(printMutex, portMAX_DELAY);
          Serial.println("Color set to: " + currentColor);
          xSemaphoreGive(printMutex);
        }
      }
      // Handle M3 (pen down) and M5 (pen up)
      else if (gcodeBuffer.startsWith("M3") || gcodeBuffer.startsWith("M5")) {
        xSemaphoreTake(printMutex, portMAX_DELAY);
        Serial.println("Command: " + gcodeBuffer);
        xSemaphoreGive(printMutex);
      }
      gcodeBuffer = gcodeBuffer.substring(gcodeBuffer.indexOf("\n")+1);
      newLine = gcodeBuffer.indexOf("\n") >= 0;
      if (!newLine) {
        gcodeBuffer = "";
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(10)); // Check every 10ms
  }
}

// --------------------------------------------------
// G-code Execution Task
// --------------------------------------------------
void TaskGcodeExec(void *pvParameters) {
  (void)pvParameters;
  while (!gcodeReady) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  GcodeCommand receivedCmd;
  BaseType_t xStatus;
  const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
  
  for (;;) {
    xStatus = xQueueReceive(queGcode, &receivedCmd, xTicksToWait);
    
    if (xStatus == pdPASS) {
      xSemaphoreTake(printMutex, portMAX_DELAY);
      Serial.println("=== Executing G-code ===");
      Serial.printf("Command: %s\n", receivedCmd.command);
      
      if (receivedCmd.hasX) {
        Serial.printf("X: %.2f\n", receivedCmd.X);
      }
      if (receivedCmd.hasY) {
        Serial.printf("Y: %.2f\n", receivedCmd.Y);
      }
      Serial.printf("Color: %s\n", receivedCmd.color);
      Serial.println("========================");
      xSemaphoreGive(printMutex);
      
      // Set target coordinates for the robot
      if (receivedCmd.hasX && receivedCmd.hasY) {
        xSemaphoreTake(poseMutex, portMAX_DELAY);
        x_t = receivedCmd.X / 1000.0; // Convert pixels to meters (scale as needed)
        y_t = receivedCmd.Y / 1000.0;
        xSemaphoreGive(poseMutex);
        
        // Wait for robot to reach position (simple approach)
        // You might want to implement a more sophisticated waiting mechanism
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
    } else {
      // No command in queue, just wait
    }
  }
}

// --------------------------------------------------
// CORE 0: ODOMETRY TASK
// --------------------------------------------------
void OdometryTask(void *pvParameters) {
  while (!gcodeReady) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  static long prevL = 0, prevR = 0;
  static unsigned long lastTime = millis();

  for (;;) {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    
    long nL = encLeft.getCount();
    long nR = encRight.getCount();
    long dL = nL - prevL;
    long dR = nR - prevR;

    float dsL = (2 * PI * R / ticksPerRev) * dL;
    float dsR = (2 * PI * R / ticksPerRev) * dR;

    if (dt > 0.001) {
      velL = dsL / dt;
      velR = dsR / dt;
    } else {
      velL = 0;
      velR = 0;
    }

    prevL = nL; 
    prevR = nR;
    lastTime = currentTime;

    float ds = (dsL + dsR) / 2.0;
    float dtheta = (dsR - dsL) / L;

    xSemaphoreTake(poseMutex, portMAX_DELAY);
    x += ds * cos(theta + dtheta / 2);
    y += ds * sin(theta + dtheta / 2);
    theta += dtheta;
    xSemaphoreGive(poseMutex);

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// --------------------------------------------------
// CORE 1: CONTROL TASK
// --------------------------------------------------
void ControlTask(void *pvParameters) {
  while (!gcodeReady) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  for (;;) {
    float localX, localY, localTheta;

    xSemaphoreTake(poseMutex, portMAX_DELAY);
    localX = x; 
    localY = y; 
    localTheta = theta;
    xSemaphoreGive(poseMutex);

    float ex = x_t - localX;
    float ey = y_t - localY;
    float rho = sqrt(ex*ex + ey*ey);
    float alpha = atan2(ey, ex) - localTheta;

    while (alpha > PI) alpha -= 2 * PI;
    while (alpha < -PI) alpha += 2 * PI;

    float K_rho = 1.5, K_alpha = 4.0;
    float v = K_rho * rho;
    float w = K_alpha * alpha;

    setL = v - (L / 2.0) * w;
    setR = v + (L / 2.0) * w;

    inputL = velL;
    inputR = velR;

    pidL.Compute();
    pidR.Compute();

    setMotorPWM(outputL, outputR);

    if (rho < 0.05) stopMotors();

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// --------------------------------------------------
// WebSocket Server Task
// --------------------------------------------------
void TaskWebServer(void *pvParameters) {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", "<h1>G-code Robot Server</h1><p>Connect via WebSocket at ws://[IP]/ws</p>");
  });

  server.begin();
  
  xSemaphoreTake(printMutex, portMAX_DELAY);
  Serial.println("Web server started");
  xSemaphoreGive(printMutex);
  
  for (;;) {
    ws.cleanupClients();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// --------------------------------------------------
// Helper functions
// --------------------------------------------------
void setMotorPWM(int leftPWM, int rightPWM) {
  if (leftPWM >= 0) { 
    digitalWrite(IN1, HIGH); 
    digitalWrite(IN2, LOW); 
  } else { 
    digitalWrite(IN1, LOW); 
    digitalWrite(IN2, HIGH); 
    leftPWM = -leftPWM; 
  }

  if (rightPWM >= 0) { 
    digitalWrite(IN3, HIGH); 
    digitalWrite(IN4, LOW); 
  } else { 
    digitalWrite(IN3, LOW); 
    digitalWrite(IN4, HIGH); 
    rightPWM = -rightPWM; 
  }

  ledcWriteChannel(0, constrain(leftPWM, 0, 255));
  ledcWriteChannel(1, constrain(rightPWM, 0, 255));
}

void stopMotors() {
  ledcWriteChannel(0, 0);
  ledcWriteChannel(1, 0);
}

// --------------------------------------------------
// Setup
// --------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }  // wait for Serial to open (especially useful on USB-C)
  delay(1000);
  
  Serial.println("=== G-code Robot Starting ===");
  
  // Connect to WiFi
  WifiClient wifi;
  Serial.print("ESP IP Address: ");
  Serial.println(wifi.connectWiFi("shruti", "shruti05"));
  
  // Setup encoders
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encLeft.attachFullQuad(ENC_L_A, ENC_L_B);
  encRight.attachFullQuad(ENC_R_A, ENC_R_B);
  encLeft.clearCount();
  encRight.clearCount();
  
  // Setup motors
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); 
  pinMode(IN4, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  ledcAttachChannel(ENA, 5000, 8, 0);
  ledcAttachChannel(ENB, 5000, 8, 1);
  
  pidL.SetOutputLimits(-255, 255);
  pidR.SetOutputLimits(-255, 255);
  pidL.SetMode(AUTOMATIC);
  pidR.SetMode(AUTOMATIC);

  // Create mutexes and queues
  poseMutex = xSemaphoreCreateMutex();
  printMutex = xSemaphoreCreateMutex();
  queGcode = xQueueCreate(50, sizeof(GcodeCommand)); // Queue for 50 commands
  
  if (poseMutex != NULL) Serial.println("Pose mutex created");
  if (printMutex != NULL) Serial.println("Print mutex created");
  if (queGcode != NULL) Serial.println("G-code queue created");

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(
    OdometryTask, "Odometry", 4096, NULL, 2, &OdometryHandle, SYSTEM_RUNNING_CORE);
  
  xTaskCreatePinnedToCore(
    ControlTask, "Control", 4096, NULL, 2, &ControlHandle, SYSTEM_RUNNING_CORE);
  
  xTaskCreatePinnedToCore(
    TaskGcodeParse, "GcodeParse", 4096, NULL, 3, &taskGcodeParse, ARDUINO_RUNNING_CORE);
  
  xTaskCreatePinnedToCore(
    TaskGcodeExec, "GcodeExec", 4096, NULL, 3, &taskGcodeExec, ARDUINO_RUNNING_CORE);
  
  xTaskCreatePinnedToCore(
    TaskWebServer, "WebServer", 8192, NULL, 1, &taskWeb, ARDUINO_RUNNING_CORE);

  Serial.println("=== SETUP COMPLETE ===");
}

// --------------------------------------------------
// Main Loop
// --------------------------------------------------
void loop() {
  // Debug output
  xSemaphoreTake(poseMutex, portMAX_DELAY);
  float px = x, py = y, ptheta = theta;
  xSemaphoreGive(poseMutex);
  
  xSemaphoreTake(printMutex, portMAX_DELAY);
  Serial.println("---- Position ----");
  Serial.printf("X: %.3f, Y: %.3f, Theta: %.3f\n", px, py, ptheta);
  Serial.printf("Target: X: %.3f, Y: %.3f\n", x_t, y_t);
  xSemaphoreGive(printMutex);
  
  vTaskDelay(pdMS_TO_TICKS(2000));
}