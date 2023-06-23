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


//#define USE_US_TIMER
//#include "osapi.h"
//#include <ets_sys.h>

//void system_timer_reinit();


const char* http_username = HTTP_USERNAME;
const char* http_password = HTTP_PASSWORD;
WiFiManager wm;

AsyncWebServer server(HTTP_LISTEN_PORT);
AsyncWebSocket ws("/ws"); // access at ws://[esp ip]/ws
AsyncEventSource events("/events"); // event source (Server-Sent events)

// Test frame: IOHC - Variable length - without CRC
//uint8_t out[] = {0xf6,0x20,0x00,0x00,0x3f,0x8a,0xd4,0x2e,0x00,0x01,0x61,0x00,0x00,0x00,0x00,0x10,0x30,0xd9,0xb5,0x78,0x0e,0x5d,0x03};   // Pure packet, without CRC that will be added in silicon
uint8_t out[] = {0xd4, 0x30, 0x00, 0x00, 0x3b, 0x5c, 0xd6, 0x8f, 0x2a, 0x93, 0x32, 0xd6, 0x18, 0xde, 0x2a, 0x0f, 0xa6, 0x25, 0x0e, 0x2c, 0x7e}; //Discovery

// Receiving buffer
//Radio::payload  payload;
bool verbosity = false;
void txUserBuffer(Tokens*cmd);

void setup() {
//    Timers::init_us();
//    system_timer_reinit();
    INIT_US;

    Serial.begin(SERIALSPEED);
//    LOG(printf_P, PSTR("\n\nsetup: free heap  : %d\n"), ESP.getFreeHeap());

    pinMode(RX_LED, OUTPUT); // we are goning to blink this LED
    digitalWrite( RX_LED, true);
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
    Cmd::addHandler((char *)"send", (char *)"Send packet", [](Tokens*cmd)->void {digitalWrite(RX_LED, digitalRead(RX_LED)^1); Radio::writeBytes(REG_FIFO, out, sizeof(out)); Radio::initTx();});
    Cmd::addHandler((char *)"sendbuf", (char *)"Send packet from cmd line", txUserBuffer);
    Cmd::addHandler((char *)"verbose", (char *)"Toggle verbose output on packets receiving", [](Tokens*cmd)->void {verbosity=!verbosity;});
}


void loop() {
    if (Radio::iAmAReceiver)
    {
        Radio::stateMachine();
        if (Radio::packetEnd) // packet received
        {
            if (verbosity)
            {
                Serial.printf(" - Len: %2.2u, mode: %1xW, first: %s, last: %s", Radio::payload.control.framelength, Radio::payload.control.mode?1:2, Radio::payload.control.first?"T":"F", Radio::payload.control.last?"T":"F");
                Serial.printf(" - bea: %u, rtd: %u, lp: %u, ack: %u, proto: %u", Radio::payload.control.use_beacon, Radio::payload.control.routed, Radio::payload.control.low_p, Radio::payload.control.ack, Radio::payload.control.prot_v);
                Serial.printf(" from: %2.2x%2.2x%2.2x, to: %2.2x%2.2x%2.2x, cmd: %2.2x", 
                    Radio::payload.nodeid.source[0], Radio::payload.nodeid.source[1], Radio::payload.nodeid.source[2],
                    Radio::payload.nodeid.target[0], Radio::payload.nodeid.target[1], Radio::payload.nodeid.target[2],
                    Radio::payload.message.cmd );

            }
            Serial.printf("\t - ");
            for (uint8_t idx=0; idx<Radio::bufferIndex; ++idx)
                Serial.printf("%2.2x", Radio::payload.buffer[idx]);
            Serial.printf("\n");

            Radio::clearFlags();
            Radio::packetEnd = false;
            Radio::bufferIndex = 0;
            return;
        }
        return;
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
            Serial.printf("Sent!\n");
            Radio::packetEnd = false;
        }

/*
    if (!Radio::iAmAReceiver)
        if (Radio::packetEnd)
        {
            Radio::setStandby();
            digitalWrite(RX_LED, digitalRead(RX_LED)^1);
            Serial.printf("Packet sent\n");
            return;
        }
*/

    wm.process();
    MDNS.update();

    return;
/*

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


void txUserBuffer(Tokens*cmd)
{
    if (cmd->size() < 2)
    {
        Serial.printf("No packet to be sent!\n");
        return;
    }
    else
    {
        digitalWrite(RX_LED, digitalRead(RX_LED)^1);

        for (unsigned int i = 0; i < cmd->at(1).length(); i += 2) {
            std::string byteString = cmd->at(1).substr(i, 2);
            char byte = (char) strtol(byteString.c_str(), NULL, 16);
            Serial.printf("%2.2x", byte);
            Radio::writeByte(REG_FIFO, byte);
        }
        Radio::initTx();
        digitalWrite(RX_LED, digitalRead(RX_LED)^1);
        Serial.printf(" ");
    }
}