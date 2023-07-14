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
#include <iohcCryptoHelpers.h>
#include <iohcRadio.h>


const char* http_username = HTTP_USERNAME;
const char* http_password = HTTP_PASSWORD;
WiFiManager wm;

AsyncWebServer server(HTTP_LISTEN_PORT);
AsyncWebSocket ws("/ws"); // access at ws://[esp ip]/ws
AsyncEventSource events("/events"); // event source (Server-Sent events)


// Receiving buffer
bool verbosity = true;
void txUserBuffer(Tokens*cmd);
void testKey(void);
uint8_t keyCap[16] = {0};
uint8_t source_originator[3] = {0};

unsigned long relTime;

//#define MAXPACKETS  199
IOHC::iohcRadio *radioInstance;
IOHC::iohcPacket *radioPackets[IOHC_INBOUND_MAX_PACKETS];
IOHC::iohcPacket *packets2send[IOHC_OUTBOUND_MAX_PACKETS];
uint8_t nextPacket = 0;

uint32_t frequencies[MAX_FREQS] = FREQS2SCAN;
//Radio::Receiver *rcvr = nullptr;
//Radio::Transmitter *tstr = nullptr;

bool msgRcvd(IOHC::iohcPacket *iohc);
bool msgArchive(IOHC::iohcPacket *iohc);
//bool msgSent(IOHC::iohcPacket *iohc);


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
    Cmd::addHandler((char *)"dump", (char *)"Dump SX1276 registers", [](Tokens*cmd)->void {Radio::dump(); Serial.printf("*%d packets in memory\n", nextPacket);});
    Cmd::addHandler((char *)"list", (char *)"List received packets", [](Tokens*cmd)->void {for (uint8_t i=0; i<nextPacket; i++) msgRcvd(radioPackets[i]);});
    Cmd::addHandler((char *)"erase", (char *)"Erase received packets", [](Tokens*cmd)->void {for (uint8_t i=0; i<nextPacket; i++) free(radioPackets[i]); nextPacket = 0;});
    Cmd::addHandler((char *)"send", (char *)"Send packet from cmd line", [](Tokens*cmd)->void {txUserBuffer(cmd);});
    Cmd::addHandler((char *)"verbose", (char *)"Toggle verbose output on packets receiving", [](Tokens*cmd)->void {verbosity=!verbosity;});
    Cmd::addHandler((char *)"key", (char *)"Test keys generation", [](Tokens*cmd)->void {testKey();});


    // Radio section
    Radio::initHardware();  // Set SPI, Reset Radio chip and setup interrupts
/*
    rcvr = new Radio::Receiver(MAX_FREQS, frequencies, SCAN_INTERVAL_US, msgRcvd);
    rcvr->Start();
*/
    radioInstance = IOHC::iohcRadio::getInstance();
//    radioInstance->start(MAX_FREQS, frequencies, SCAN_INTERVAL_US, msgRcvd, msgRcvd);
    radioInstance->start(MAX_FREQS, frequencies, SCAN_INTERVAL_US, msgArchive, msgRcvd);

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

bool msgRcvd(IOHC::iohcPacket *iohc)
{
    unsigned char _dir;
    if ((iohc->millis - relTime) > 3000)
    {
        Serial.printf("\n");
        relTime = iohc->millis;
        for (uint8_t i=0; i<3; i++)
            source_originator[i] = iohc->payload.packet.header.source[i];
    }
    if (!memcmp(source_originator, iohc->payload.packet.header.source, 3))
        _dir = '>';
    else
        _dir = '<';


    if (verbosity)
    {
        Serial.printf("Len: %2.2u, mode: %1xW, first: %s, last: %s,", iohc->payload.packet.header.framelength, iohc->payload.packet.header.mode?1:2, iohc->payload.packet.header.first?"T":"F", iohc->payload.packet.header.last?"T":"F");
        Serial.printf(" b: %u, r: %u, lp: %u, ack: %u, prt: %u", iohc->payload.packet.header.use_beacon, iohc->payload.packet.header.routed, iohc->payload.packet.header.low_p, iohc->payload.packet.header.ack, iohc->payload.packet.header.prot_v);
        Serial.printf(" from: %2.2x%2.2x%2.2x, to: %2.2x%2.2x%2.2x, cmd: %2.2x, fr: %6.3fM, s+%5.3f",
            iohc->payload.packet.header.source[0], iohc->payload.packet.header.source[1], iohc->payload.packet.header.source[2],
            iohc->payload.packet.header.target[0], iohc->payload.packet.header.target[1], iohc->payload.packet.header.target[2],
            iohc->payload.packet.header.cmd, (float)(iohc->frequency)/1000000, (float)(iohc->millis - relTime)/1000);
        Serial.printf(" %c ", _dir);
    }
    for (uint8_t idx=0; idx<iohc->buffer_length; ++idx)
        Serial.printf("%2.2x",iohc->payload.buffer[idx]);
    Serial.printf("\n");

/*
*/
    switch (iohc->payload.packet.header.cmd)
    {
    case 0x30:
        encrypt_1W_key((const uint8_t *)iohc->payload.packet.header.source, (uint8_t *)iohc->payload.packet.msg.p0x30.enc_key);
        Serial.printf("Controller key in clear: ");
        for (uint8_t idx=0; idx < 16; idx ++)
        {
            keyCap[idx] = iohc->payload.packet.msg.p0x30.enc_key[idx];
            Serial.printf("%2.2x", keyCap[idx]);
        }
        Serial.printf("\n");
        break;
    
    case 0x39:
        if (keyCap[0] == 0)
            break;
        uint8_t hmac[6];
        std::vector<uint8_t> frame(&iohc->payload.packet.header.cmd, &iohc->payload.packet.header.cmd+2);
        create_1W_hmac(hmac, (uint8_t *)iohc->payload.packet.msg.p0x39.sequence, keyCap, frame);
        Serial.printf("hmac: ");
        for (uint8_t idx=0; idx < 6; idx ++)
            Serial.printf("%2.2x", hmac[idx]);
        Serial.printf("\n");
        break;
    }

    relTime = iohc->millis;
    return true;
}

bool msgArchive(IOHC::iohcPacket *iohc)
{
    radioPackets[nextPacket] = (IOHC::iohcPacket *)malloc(sizeof(IOHC::iohcPacket));
    if (!radioPackets[nextPacket])
    {
        Serial.printf("*** Malloc failed!\n");
        return false;
    }

    radioPackets[nextPacket]->buffer_length = iohc->buffer_length;
    radioPackets[nextPacket]->frequency = iohc->frequency;
    radioPackets[nextPacket]->millis = iohc->millis;
    radioPackets[nextPacket]->rssi = iohc->rssi;

    for (uint8_t i=0; i<iohc->buffer_length; i++)
        radioPackets[nextPacket]->payload.buffer[i] = iohc->payload.buffer[i];

    nextPacket += 1;
    Serial.printf("-> %d\r", nextPacket);
    if (nextPacket >= IOHC_INBOUND_MAX_PACKETS)
    {
        nextPacket = IOHC_INBOUND_MAX_PACKETS -1;
        Serial.printf("*** Not enough buffers available. Please erase current ones\n");
        return false;
    }

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
        packets2send[0] = (IOHC::iohcPacket *)malloc(sizeof(IOHC::iohcPacket));
        if (cmd->size() == 3)
            packets2send[0]->frequency = frequencies[atoi(cmd->at(2).c_str())-1];
        else
            packets2send[0]->frequency = 0;

        digitalWrite(RX_LED, digitalRead(RX_LED)^1);
        packets2send[0]->buffer_length = hexStringToBytes(cmd->at(1), packets2send[0]->payload.buffer);
        packets2send[0]->millis = 35;
        packets2send[0]->repeat = 4;

        digitalWrite(RX_LED, digitalRead(RX_LED)^1);
        radioInstance->send(packets2send);
    }
}


void testKey()
{
    std::string controller_key = "d94a00399a46b5aba67f3809b68ecc52";
    std::string node_address = "8ad42e";
    std::string dest_address = "00013f";
    std::string sequence = "1934";

    char bcontroller[16];
    char bnode[3];
    char bdest[3];
    uint8_t bseq[2] = {0x19, 0x37};
    char output[16];


    hexStringToBytes(controller_key, bcontroller);
    hexStringToBytes(node_address, bnode);
    hexStringToBytes(dest_address, bdest);
    
    Serial.printf("Node address: ");
    for (uint8_t idx=0; idx < 3; idx ++)
        Serial.printf("%2.2x", bnode[idx]);
    Serial.printf("\n");
    Serial.printf("Destination address: ");
    for (uint8_t idx=0; idx < 3; idx ++)
        Serial.printf("%2.2x", bdest[idx]);
    Serial.printf("\n");
    Serial.printf("Controller key encrypted: ");
    for (uint8_t idx=0; idx < sizeof(bcontroller); idx ++)
        Serial.printf("%2.2x", bcontroller[idx]);
    Serial.printf("\n");

    std::vector<uint8_t> node(bnode, bnode+3);
    std::vector<uint8_t> controller(bcontroller, bcontroller+16);
    std::vector<uint8_t> ret;

    encrypt_1W_key((const uint8_t *)bnode, (uint8_t *)bcontroller);
    Serial.printf("Controller key in clear: ");
    for (uint8_t idx=0; idx < 16; idx ++)
        Serial.printf("%2.2x", bcontroller[idx]);
    Serial.printf("\n");

    uint8_t bframe[2] = {0x2e, 0x00};
    std::vector<uint8_t> seq(bseq, bseq+2);
    std::vector<uint8_t> frame(bframe, bframe+2);
    hexStringToBytes(controller_key, bcontroller);
    create_1W_hmac((uint8_t *)output, bseq, (uint8_t *)bcontroller, frame);

    Serial.printf("hmac: ");
    for (uint8_t idx=0; idx < 6; idx ++)
        Serial.printf("%2.2x", output[idx]);
    Serial.printf("\n");
    Serial.printf("Packet to send: f120");
    for (uint8_t idx=0; idx < 3; idx ++)
        Serial.printf("%2.2x", bdest[idx]);
    for (uint8_t idx=0; idx < 3; idx ++)
        Serial.printf("%2.2x", bnode[idx]);
    for (uint8_t idx=0; idx < 2; idx ++)
        Serial.printf("%2.2x", bframe[idx]);
    for (uint8_t idx=0; idx < 2; idx ++)
        Serial.printf("%2.2x", bseq[idx]);
    for (uint8_t idx=0; idx < 6; idx ++)
        Serial.printf("%2.2x", output[idx]);
    Serial.printf("\n\n");
}