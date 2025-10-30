#include <PID_v1.h>
#include <Arduino.h>
#include <ESP32Encoder.h>

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
SemaphoreHandle_t poseMutex;

// --- Create Encoder objects ---
ESP32Encoder encLeft;
ESP32Encoder encRight;



void setup() {
  Serial.begin(115200);

  // Setup encoders
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(ENC_L_A), onEncLA, RISING);
  // attachInterrupt(digitalPinToInterrupt(ENC_R_A), onEncRA, RISING);
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encLeft.attachFullQuad(ENC_L_A, ENC_L_B);
  encRight.attachFullQuad(ENC_R_A, ENC_R_B);
  encLeft.clearCount();
  encRight.clearCount();
  // Setup motors
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // ledcAttach(ENA, 5000, 8,0); // ENA pin, 5kHz, 8-bit
  // ledcAttach(ENB, 5000, 8,1); // ENB pin, 5kHz, 8-bit
  ledcAttachChannel(ENA, 5000, 8, 0); // pin, freq, resolution, channel
  ledcAttachChannel(ENB, 5000, 8, 1); // pin, freq, resolution, channel
  
  pidL.SetOutputLimits(-255, 255);
  pidR.SetOutputLimits(-255, 255);
  pidL.SetMode(AUTOMATIC);
  pidR.SetMode(AUTOMATIC);

  // Mutex for shared pose data
  poseMutex = xSemaphoreCreateMutex();

  // Create FreeRTOS tasks pinned to cores
  xTaskCreatePinnedToCore(OdometryTask, "Odometry", 4096, NULL, 1, &OdometryHandle, 0);
  xTaskCreatePinnedToCore(ControlTask, "Control", 8192, NULL, 1, &ControlHandle, 1);
  Serial.println("=== SETUP COMPLETE ==="); 
  Serial.println("Tasks created successfully");

}

// --------------------------------------------------
// CORE 0: ODOMETRY TASK
// --------------------------------------------------
void OdometryTask(void *pvParameters) {
  static long prevL = 0, prevR = 0;
  static unsigned long lastTime = millis();

  for (;;) {

    // Serial.print("Left Enc: ");
    // Serial.println(prevL);

    // Serial.print("Right Enc: ");
    // Serial.println(prevR);

    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; //Ms->S
    
    
    long nL =  encLeft.getCount();
    long nR = encRight.getCount();
    long dL = nL - prevL;
    long dR = nR - prevR;


    float dsL = (2 * PI * R / ticksPerRev) * dL;
    float dsR = (2 * PI * R / ticksPerRev) * dR;


    if (dt > 0.001) {  // Only update if dt > 1ms
      velL = dsL / dt;
      velR = dsR / dt;
    } else {
      velL = 0;
      velR = 0;
    }

    

    prevL = nL; prevR = nR;
    lastTime = currentTime;



    float ds = (dsL + dsR) / 2.0;
    float dtheta = (dsR - dsL) / L;



    xSemaphoreTake(poseMutex, portMAX_DELAY);
    x += ds * cos(theta + dtheta / 2);
    y += ds * sin(theta + dtheta / 2);
    theta += dtheta;
    xSemaphoreGive(poseMutex);

    vTaskDelay(pdMS_TO_TICKS(20));  // 50 Hz
  }
}

// --------------------------------------------------
// CORE 1: CONTROL TASK
// --------------------------------------------------
void ControlTask(void *pvParameters) {
  for (;;) {
    float localX, localY, localTheta;

    // Copy pose safely
    xSemaphoreTake(poseMutex, portMAX_DELAY);
    localX = x; localY = y; localTheta = theta;
    xSemaphoreGive(poseMutex);

    // --- Compute go-to-goal control ---
    float ex = x_t - localX;
    float ey = y_t - localY;
    float rho = sqrt(ex*ex + ey*ey);
    float alpha = atan2(ey, ex) - localTheta;


    while (alpha > PI) alpha -= 2 * PI;
    while (alpha < -PI) alpha += 2 * PI;

    float K_rho = 1.5, K_alpha = 4.0;
    float v = K_rho * rho;
    float w = K_alpha * alpha;

    // // --- Convert to wheel velocities ---
    setL = v - (L / 2.0) * w;
    setR = v + (L / 2.0) * w;

    // --- Estimate actual speeds (simple for now) ---
    // Use MEASURED velocities for PID input
    inputL = velL;  // <-- NOW using actual measured velocity!
    inputR = velR;  // <-- NOW using actual measured velocity!

    pidL.Compute();
    pidR.Compute();

    Serial.println("----ERROR----");

    Serial.println(ex);
    Serial.println(ey);
    Serial.println(alpha);



    Serial.println("-------------");


    setMotorPWM(outputL, outputR);

    if (rho < 0.05) stopMotors();

    vTaskDelay(pdMS_TO_TICKS(10)); // 50 Hz loop
  }
}

// --------------------------------------------------
// Helper functions
// --------------------------------------------------
void setMotorPWM(int leftPWM, int rightPWM) {
  // Serial.println("----PWM----");
  // Serial.println(leftPWM);
  // Serial.println(rightPWM);
  // Serial.println("------------");

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
  Serial.println("----PWM----");
  Serial.println(constrain(leftPWM, 0, 255));
  Serial.println(constrain(rightPWM, 0, 255));
  Serial.println("---------");

  ledcWriteChannel(0, constrain(leftPWM, 0, 255)); // Use channel 0 for ENA
  ledcWriteChannel(1, constrain(rightPWM, 0, 255)); // Use channel 1 for ENB

}

void stopMotors() {
  Serial.println("STOP");

  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
}


void loop() {
  // Empty - all work done in FreeRTOS tasks
  Serial.println("----LOOP DEBUG----");

  Serial.println(x);
  Serial.println(y);
  Serial.println(theta);


  Serial.println("-------------");

  vTaskDelay(pdMS_TO_TICKS(1000));

}



