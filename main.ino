// #include <WifiClient.h>
// #include <MotorController.h>
// #include <ESPAsyncWebServer.h>
// #include <AsyncTCP.h>

// WifiClient webserver;
// MotorController mc;
// AsyncWebSocket ws("/ws");
// AsyncWebServer server(80);
// //forward declarations.
// void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
//              AwsEventType type, void *arg, uint8_t *data, size_t len);

// bool ledState = false;
// void notifyClients() {
//   ws.textAll(ledState ? "LED_ON" : "LED_OFF");
// }

// void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
//   AwsFrameInfo *info = (AwsFrameInfo*)arg;
//   pinMode(LED_BUILTIN, OUTPUT);
//   if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
//     data[len] = 0;
//     String msg = (char*)data;

//     if (msg == "START_CONNECTION") {
//       ledState = true;
//       digitalWrite(LED_BUILTIN, HIGH);
//     } else if (msg == "END_CONNECTION") {
//       ledState = false;
//       digitalWrite(LED_BUILTIN, LOW);
//     } else if (msg = "FORWARD") {
//       mc.forward();
//     } else if (msg = "BACKWARD") {
//       mc.backward();
//     } else if (msg = "STOP") {
//       mc.stop();
//     }
//     notifyClients();
//   }
// }

// void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
//             AwsEventType type, void *arg, uint8_t *data, size_t len) {
//   if (type == WS_EVT_CONNECT) {
//     Serial.println("WebSocket client connected");
//   } else if (type == WS_EVT_DISCONNECT) {
//     Serial.println("WebSocket client disconnected");
//   } else if (type == WS_EVT_DATA) {
//     handleWebSocketMessage(arg, data, len);
//   }
// }

// void setup() {
//   mc.init(11, 12, 35, 38, 37, 36);
//   webserver.connectWiFi("shruti", "shruti05");
//   ws.onEvent(onEvent);
//   server.addHandler(&ws);
//   server.begin();
// }

// void loop() {
//   //ws.cleanupClients();
// }