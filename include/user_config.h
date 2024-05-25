#ifndef IOHC_USER_H
#define IOHC_USER_H

#include <board-config.h>

#define WIFI_SSID ""
#define WIFI_PASSWD ""
inline const char *wifi_ssid = "";
inline const char *wifi_passwd = "";

#define MQTT_SERVER "192.168.1.40"
#define MQTT_USER "user"
#define MQTT_PASSWD "passwd"
inline const char *mqtt_server = "192.168.1.40";
inline const char *mqtt_user = "user";
inline const char *mqtt_password = "passwd";

#define HTTP_LISTEN_PORT    80
#define HTTP_USERNAME       "admin"
#define HTTP_PASSWORD       "admin"
#if defined(ESP8266)
        #define SERIALSPEED         460800
#elif defined(ESP32)
#define SERIALSPEED         115200 //921600
#endif

#endif
