#ifndef WIFICLIENT_H
#define WIFICLIENT_H

class WifiClient {
private: 
    const char* ntpServer = "pool.ntp.org";
    const char* timezoneEST = "EST5EDT,M3.2.0/2,M11.1.0";
    unsigned long previousMillisWiFi = 0;
    char timeStringBuff[50];

public: 
    WifiClient();
    uint32_t connectWiFi(char* username, char* password, char* certificate = "");
};

#endif  // WIFICLIENT_H
