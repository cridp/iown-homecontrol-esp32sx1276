#include <ESP8266WiFi.h>
#include "ESPAsyncTCP.h"
#include <ESPAsyncWebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266SSDP.h>


#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration - Async branch
#include <SX1276Helpers.h>
#include <board-config.h>
#include <user_config.h>
#include <Receiver.h>
#include <Transmitter.h>
#include <globals.h>
#include <WebServerHelpers.h>


const char* http_username = HTTP_USERNAME;
const char* http_password = HTTP_PASSWORD;
WiFiManager wm;

AsyncWebServer server(HTTP_LISTEN_PORT);
AsyncWebSocket ws("/ws"); // access at ws://[esp ip]/ws
AsyncEventSource events("/events"); // event source (Server-Sent events)


// Receiving buffer
//Radio::payload  payload;
bool verbosity = true;
void txUserBuffer(Tokens*cmd);
Radio::iohcPacket *iohc;
unsigned long relTime;


uint32_t frequencies[MAX_FREQS] = FREQS2SCAN;
Radio::Receiver *rcvr = nullptr;
Radio::Transmitter *tstr = nullptr;

bool msgRcvd(Radio::iohcPacket *iohc);
//bool msgSent(Radio::iohcPacket *iohc);


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


    Cmd::init();    // Initialize Serial commands reception and handlers
    Cmd::addHandler((char *)"dump", (char *)"Dump SX1276 registers", [](Tokens*cmd)->void {Radio::dump();});
    Cmd::addHandler((char *)"rx", (char *)"Run as a receiver (startup default)", [](Tokens*cmd)->void {if (rcvr) {delete rcvr; rcvr = nullptr;} if (tstr) {delete tstr; tstr = nullptr;} rcvr = new Radio::Receiver(MAX_FREQS, frequencies, SCAN_INTERVAL_US, msgRcvd); rcvr->Start();});
    Cmd::addHandler((char *)"tx", (char *)"Run as a transmitter", [](Tokens*cmd)->void {if (rcvr) {delete rcvr; rcvr = nullptr;} if (tstr) {delete tstr; tstr = nullptr;} tstr = new Radio::Transmitter(msgRcvd); tstr->Start();});
    Cmd::addHandler((char *)"send", (char *)"Send packet from cmd line", [](Tokens*cmd)->void {if (rcvr) {delete rcvr; rcvr = nullptr;} if (tstr) {delete tstr; tstr = nullptr;} tstr = new Radio::Transmitter(msgRcvd); tstr->Start(); txUserBuffer(cmd);});
    Cmd::addHandler((char *)"verbose", (char *)"Toggle verbose output on packets receiving", [](Tokens*cmd)->void {verbosity=!verbosity;});


    // Radio section
    Radio::init();  // Set SPI, Reset Radio chip and setup interrupts

    rcvr = new Radio::Receiver(MAX_FREQS, frequencies, SCAN_INTERVAL_US, msgRcvd);
    rcvr->Start();
    Radio::dump();  // Dump registers after setting up
    relTime = millis();
}


void loop() {
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

bool msgRcvd(Radio::iohcPacket *iohc)
{
    if ((iohc->millis - relTime) > 3000)
    {
        Serial.printf("\n");
        relTime = iohc->millis;
    }

    if (verbosity)
    {
        Serial.printf(" - Len: %2.2u, mode: %1xW, first: %s, last: %s", iohc->packet.control.framelength, iohc->packet.control.mode?1:2, iohc->packet.control.first?"T":"F", iohc->packet.control.last?"T":"F");
        Serial.printf(" - b: %u, r: %u, lp: %u, ack: %u, prt: %u", iohc->packet.control.use_beacon, iohc->packet.control.routed, iohc->packet.control.low_p, iohc->packet.control.ack, iohc->packet.control.prot_v);
        Serial.printf(" from: %2.2x%2.2x%2.2x, to: %2.2x%2.2x%2.2x, cmd: %2.2x, freq: %6.3fM, s+%6.3f", 
            iohc->packet.nodeid.source[0], iohc->packet.nodeid.source[1], iohc->packet.nodeid.source[2],
            iohc->packet.nodeid.target[0], iohc->packet.nodeid.target[1], iohc->packet.nodeid.target[2],
            iohc->packet.message.cmd, (float)(iohc->frequency)/1000000, (float)(iohc->millis - relTime)/1000);

    }
    Serial.printf(" - ");
    for (uint8_t idx=0; idx<iohc->buffer_length; ++idx)
        Serial.printf("%2.2x",iohc->packet.buffer[idx]);
    Serial.printf("\n");

    relTime = iohc->millis;
    return true;
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
        iohc = new Radio::iohcPacket();
        if (cmd->size() == 3)
            iohc->frequency = frequencies[atoi(cmd->at(2).c_str())-1];
        else
            iohc->frequency = frequencies[2];

        digitalWrite(RX_LED, digitalRead(RX_LED)^1);
        for (unsigned int i = 0; i < cmd->at(1).length(); i += 2) {
            std::string byteString = cmd->at(1).substr(i, 2);
            char byte = (char) strtol(byteString.c_str(), NULL, 16);
            iohc->packet.buffer[iohc->buffer_length++] = byte;
        }
        digitalWrite(RX_LED, digitalRead(RX_LED)^1);
        tstr->Send(iohc);
    }
}