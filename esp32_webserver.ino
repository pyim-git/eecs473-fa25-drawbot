#define Serial Serial0

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
//ESP IP Address: 172.20.10.10
// Replace with your WiFi credentials
const char* ssid = "shruti";
const char* password = "shruti05";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

bool ledState = false;

void notifyClients() {
  ws.textAll(ledState ? "LED_ON" : "LED_OFF");
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    String msg = (char*)data;

    if (msg == "LED_ON") {
      ledState = true;
      digitalWrite(LED_BUILTIN, HIGH);
    } else if (msg == "LED_OFF") {
      ledState = false;
      digitalWrite(LED_BUILTIN, LOW);
    }
    notifyClients();
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("WebSocket client connected");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("WebSocket client disconnected");
  } else if (type == WS_EVT_DATA) {
    handleWebSocketMessage(arg, data, len);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("hi??");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  ws.onEvent(onEvent);
  server.addHandler(&ws);

  // Serve a test webpage
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", R"rawliteral(
      <!DOCTYPE html>
      <html>
      <body>
        <h2>ESP32 WebSocket Control</h2>
        <button onclick="sendMsg('LED_ON')">Turn ON</button>
        <button onclick="sendMsg('LED_OFF')">Turn OFF</button>
        <p id="status">Status: Unknown</p>
        <script>
          var ws = new WebSocket('ws://' + location.host + '/ws');
          ws.onmessage = function(event) {
            document.getElementById('status').innerText = 'Status: ' + event.data;
          };
          function sendMsg(msg) {
            ws.send(msg);
          }
        </script>
      </body>
      </html>
    )rawliteral");
  });

  server.begin();
}

void loop() {
  ws.cleanupClients();
}
