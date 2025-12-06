// ESP IP Address: 172.20.10.10 on hotspot
// ESP IP Address: xxxx on MWireless

#define Serial Serial0
#include <WiFi.h>
#include "esp_wpa2.h"
#include <esp_wifi.h>
#include <time.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "WifiClient.h"

// //forward declarations.
// void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
//              AwsEventType type, void *arg, uint8_t *data, size_t len);
WifiClient::WifiClient(){}
uint32_t WifiClient::connectWiFi(const char* username, const char* password, const char* certificate) {
  delay(10);

  if(strlen(certificate)!=0) { //enterprise wifi
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    Serial.print("MAC >> "); 
    Serial.println(WiFi.macAddress()); 
    Serial.printf("Connecting to WiFi: %s\n", username);
    esp_wifi_sta_wpa2_ent_set_ca_cert((uint8_t *)certificate, strlen(certificate)+1);
    esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)username, strlen(username));
    esp_wifi_sta_wpa2_ent_set_username((uint8_t *)username, strlen(username));
    esp_wifi_sta_wpa2_ent_set_password((uint8_t *)password, strlen(password));
    esp_wifi_sta_wpa2_ent_enable();

    WiFi.begin("eduroam");
  }
  else { //hotspot
    WiFi.begin(username, password);
  }

  //retry counter
  uint8_t counter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    counter++;
    if (counter >= 60) {  // 30s timeout
      ESP.restart();
    }
  }

  //success
  Serial.println(F(" connected!"));
  Serial.print(F("IP address set: "));
  Serial.println(WiFi.localIP()); //ESP Local IP Address.

  // Init and sync NTP
  configTime(0, 0, ntpServer);
  setenv("TZ", timezoneEST, 1);
  time_t now = 0;
  return WiFi.localIP();
}