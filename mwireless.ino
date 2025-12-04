// #include <WiFi.h>
// #include "esp_wpa2.h"
// #include <esp_wifi.h>
// #include <time.h>
// #define Serial Serial0

// const char *ntpServer = "pool.ntp.org";
// const char *timezoneEST = "EST5EDT,M3.2.0/2,M11.1.0";

// uint8_t counter = 0;
// unsigned long previousMillisWiFi = 0;
// char timeStringBuff[50];

// void printLocalTime(bool printToSerial = false) {
//   struct tm timeinfo;
//   if (!getLocalTime(&timeinfo)) {
//     Serial.println(F("NTP sync failed"));
//     return;
//   }
//   strftime(timeStringBuff, sizeof(timeStringBuff), "%A, %B %d %Y %H:%M:%S", &timeinfo);
//   if (printToSerial) Serial.println(timeStringBuff);
// }

// void setup() {
//   Serial.begin(115200);
//   delay(10);
//   Serial.println();

//   WiFi.disconnect(true);
//   WiFi.mode(WIFI_STA);

//   Serial.print("MAC >> ");
//   Serial.println(WiFi.macAddress());
//   Serial.printf("Connecting to WiFi: %s\n", ssid);

//   // WPA2 Enterprise setup (NEW METHOD)
//   esp_wifi_sta_wpa2_ent_set_ca_cert((uint8_t *)incommon_ca, strlen(incommon_ca) + 1);
//   esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
//   esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
//   esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));

//   // In new ESP-IDF, no config struct needed
//   esp_wifi_sta_wpa2_ent_enable();

//   WiFi.begin(ssid);

//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//     counter++;
//     if (counter >= 60) {  // 30s timeout
//       ESP.restart();
//     }
//   }
//   Serial.println(F(" connected!"));
//   Serial.print(F("IP address set: "));
//   Serial.println(WiFi.localIP());

//   // Init and sync NTP
//   configTime(0, 0, ntpServer);
//   setenv("TZ", timezoneEST, 1);

//   time_t now = 0;
//   Serial.print("Obtaining NTP time: ");
//   while (now < 1510592825) { // sanity check ~2017
//     Serial.print(".");
//     delay(500);
//     time(&now);
//   }
//   Serial.print(F(" success!\nGot Time: "));
//   printLocalTime(true);
//   Serial.println(F("NTP time received!"));
// }

// void loop() {
//   unsigned long currentMillis = millis();

//   if (WiFi.status() == WL_CONNECTED) {
//     counter = 0;
//     if (currentMillis - previousMillisWiFi >= 15 * 1000) {
//       printLocalTime(true);
//       previousMillisWiFi = currentMillis;
//       Serial.print(F("Wifi is still connected with IP: "));
//       Serial.println(WiFi.localIP());
//     }
//   } else {
//     WiFi.begin(ssid);
//     Serial.printf("Reconnecting to WiFi: %s ", ssid);
//     while (WiFi.status() != WL_CONNECTED) {
//       delay(500);
//       Serial.print(".");
//       counter++;
//       if (counter >= 60) {
//         ESP.restart();
//       }
//     }
//     Serial.println(F(" connected!"));
//   }
// }
