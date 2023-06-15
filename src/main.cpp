#include <ESP8266WiFi.h>
#include "ESPAsyncTCP.h"
#include <ESPAsyncWebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266SSDP.h>


#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration - Async branch
#include <SX1276Helpers.h>
#include <board-config.h>
#include <user_config.h>
#include <globals.h>
#include <WebServerHelpers.h>


const char* http_username = HTTP_USERNAME;
const char* http_password = HTTP_PASSWORD;
WiFiManager wm;

AsyncWebServer server(HTTP_LISTEN_PORT);
AsyncWebSocket ws("/ws"); // access at ws://[esp ip]/ws
AsyncEventSource events("/events"); // event source (Server-Sent events)

// Test frame: IOHC - Variable length - without CRC
uint8_t out[] = {0xf6,0x00,0x00,0x00,0x3f,0x8a,0xd4,0x2e,0x00,0x01,0x61,0xd2,0x00,0x00,0x00,0x0f,0x35,0x3b,0xe0,0xd9,0x0d,0x60,0x49, 0xbd,0x69,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
// Receiving buffer
Radio::payload  payload;

void setup() {
    Serial.begin(SERIALSPEED);
//    LOG(printf_P, PSTR("\n\nsetup: free heap  : %d\n"), ESP.getFreeHeap());

    pinMode(LED_BUILTIN, OUTPUT); // we are goning to blink this LED
    digitalWrite( LED_BUILTIN, true);
    WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP 
    wm.autoConnect();
    
    // WiFi Manager setup
    // Reset settings - wipe credentials for testing
    //wm.resetSettings();
    wm.setConfigPortalBlocking(false);
    wm.setConfigPortalTimeout(120);
    // automatically connect using saved credentials if they exist
    // If connection fails it starts an access point with the specified name
    if(wm.autoConnect("AutoConnectAP")){
        LOG(printf_P, PSTR("Connected :)"));
        // Show the current IP Address
        LOG(printf_P, PSTR("IP got: %s\n"), WiFi.localIP().toString());
    }
    else {
        LOG(printf_P("Configportal running\n"));
    }

    // Start MDNS
    if (! MDNS.begin("IO-Homecontrol_gateway")) 
        Serial.println(F("Error setting up MDNS responder"));

    MDNS.addService("http", "tcp", HTTP_LISTEN_PORT);
    testmDNS((char *)"http");
    // Start SSDP
    SSDP.setSchemaURL("description.xml");
    SSDP.setHTTPPort(HTTP_LISTEN_PORT);
    SSDP.setName("Velux remote gateway");
    SSDP.setSerialNumber(ESP.getChipId());
    SSDP.setURL("/");
    SSDP.setDeviceType("upnp:rootdevice");
    SSDP.begin();
    //server.on("/description.xml", HTTP_GET, [](AsyncWebServerRequest *request) { SSDP.schema((Print&)std::ref(request->client())); });

    // Web Server
    // attach AsyncWebSocket
    ws.onEvent(onEvent);
    server.addHandler(&ws);

    // attach AsyncEventSource
    server.addHandler(&events);

    /*
    To be completed
    
    // respond to GET requests on URL /heap
    server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(ESP.getFreeHeap()));
    });

    // upload a file to /upload
    server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request){
    request->send(200);
    }, onUpload);

    // send a file when /index is requested
    server.on("/index", HTTP_ANY, [](AsyncWebServerRequest *request){
//    request->send(SPIFFS, "/index.htm");
    });

    // HTTP basic authentication
    server.on("/login", HTTP_GET, [](AsyncWebServerRequest *request){
    if(!request->authenticate(http_username, http_password))
        return request->requestAuthentication();
    request->send(200, "text/plain", "Login Success!");
    });

    // Simple Firmware Update Form
    server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", "<form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>");
    });
    server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request){
    shouldReboot = !Update.hasError();
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", shouldReboot?"OK":"FAIL");
    response->addHeader("Connection", "close");
    request->send(response);
    },[](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
    if(!index){
        Serial.printf("Update Start: %s\n", filename.c_str());
        Update.runAsync(true);
        if(!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000)){
        Update.printError(Serial);
        }
    }
    if(!Update.hasError()){
        if(Update.write(data, len) != len){
        Update.printError(Serial);
        }
    }
    if(final){
        if(Update.end(true)){
        Serial.printf("Update Success: %uB\n", index+len);
        } else {
        Update.printError(Serial);
        }
    }
    });
    
    */

    // attach filesystem root at URL /fs
//    server.serveStatic("/fs", SPIFFS, "/");

    // Catch-All Handlers
    // Any request that can not find a Handler that canHandle it
    // ends in the callbacks below.
    server.onNotFound(onRequest);
    server.onRequestBody(onBody);
    server.begin();


    // Radio section
    Radio::init();  // Set SPI, Reset Radio chip and setup interrupts

    Radio::setCarrier(Radio::Carrier::Frequency, 868925000);
    Radio::setCarrier(Radio::Carrier::Deviation, 19200);
    Radio::setCarrier(Radio::Carrier::Bitrate, 38400);
    Radio::setCarrier(Radio::Carrier::Bandwidth, 250);
    Radio::setCarrier(Radio::Carrier::Modulation, Radio::Modulation::FSK);

    if (Radio::iAmAReceiver)
        Radio::initRx();
    else
        Radio::initTx();
    Radio::dump();  // Dump registers after setting up

    Cmd::init();    // Initialize Serial commands reception and handlers
    Cmd::addHandler((char *)"dump", (char *)"Dump SX1276 registers", [](Tokens*cmd)->void {Radio::dump();});
    Cmd::addHandler((char *)"rx", (char *)"Run as a receiver (startup default)", [](Tokens*cmd)->void {Radio::iAmAReceiver=true; Radio::initRx();});
    Cmd::addHandler((char *)"tx", (char *)"Run as a transmitter", [](Tokens*cmd)->void {Radio::iAmAReceiver=false; Radio::setStandby();});
    Cmd::addHandler((char *)"send", (char *)"Send packet", [](Tokens*cmd)->void {Radio::initTx(); Radio::writeBytes(REG_FIFO, out, sizeof(out));});
}


void loop() {
    if (Radio::iAmAReceiver)
    {    
        while (Radio::dataAvail) 
        {
            payload.buffer[Radio::bufferIndex++] = Radio::readByte(REG_FIFO);
//            Serial.printf("%2.2x", payload.buffer[Radio::bufferIndex-1]);
        }


        if (Radio::packetEnd) // packet received
        {
            Radio::setStandby();
            Serial.printf("\nPacket received - ");
            for (uint8_t idx=0; idx<Radio::bufferIndex; ++idx)
                Serial.printf("%2.2x", payload.buffer[idx]);

// To be verified as bits values are incorrect
//            Serial.printf("\nlen: %u, rel: %1x, source: %6.6x, target: %6.6x\n", payload.control.framelength, payload.control.relationship,
//                payload.nodeid.source, payload.nodeid.target);
            Radio::clearFlags();
            Radio::packetEnd = false;
            Radio::bufferIndex = 0;
            Radio::initRx();
            return;
        }

        if (Radio::inStdbyOrSleep())
        {
            Serial.printf("\nPacket received\n");
            Radio::initRx();
        }

        return;
    }
    else
        if (Radio::packetEnd)
        {
            Radio::setStandby();
            Serial.printf("Packet sent\n");
            return;
        }

    wm.process();
    MDNS.update();

    return;
/*


    digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN)^1);



    if(shouldReboot){
    Serial.println("Rebooting...");
    delay(100);
    ESP.restart();
    }
    static char temp[128];
    sprintf(temp, "Seconds since boot: %lu", millis()/1000);
    events.send(temp, "time"); //send event "time"
*/
}
