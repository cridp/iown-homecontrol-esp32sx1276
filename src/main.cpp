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
#include <iohcSystemTable.h>
#include <iohcRemote1W.h>
#include <fileSystemHelpers.h>


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
IOHC::iohcSystemTable *sysTable;
IOHC::iohcRemote1W *remote1W;
//IOHC::iohcObject *dev;

uint32_t frequencies[MAX_FREQS] = FREQS2SCAN;

bool msgRcvd(IOHC::iohcPacket *iohc);
bool msgArchive(IOHC::iohcPacket *iohc);
void discovery(void);


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

    // Mount LittleFS filesystem
    LittleFSConfig lcfg;
    lcfg.setAutoFormat(false);
    LittleFS.setConfig(lcfg);
    LittleFS.begin();

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
//    request->send(LittleFS, "/index.htm");
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
//    server.serveStatic("/fs", LittleFS, "/");

    // Catch-All Handlers
    // Any request that can not find a Handler that canHandle it
    // ends in the callbacks below.
    server.onNotFound(onRequest);
    server.onRequestBody(onBody);
    server.begin();

    sysTable = IOHC::iohcSystemTable::getInstance();
    remote1W = IOHC::iohcRemote1W::getInstance();
    radioInstance = IOHC::iohcRadio::getInstance();

    Cmd::init();    // Initialize Serial commands reception and handlers
    Cmd::addHandler((char *)"dump", (char *)"Dump SX1276 registers", [](Tokens*cmd)->void {Radio::dump(); Serial.printf("*%d packets in memory\t", nextPacket); Serial.printf("*%d devices discovered\n\n", sysTable->size());});
    Cmd::addHandler((char *)"list", (char *)"List received packets", [](Tokens*cmd)->void {for (uint8_t i=0; i<nextPacket; i++) msgRcvd(radioPackets[i]); sysTable->dump();});
    Cmd::addHandler((char *)"save", (char *)"Saves Objects table", [](Tokens*cmd)->void {sysTable->save(true);});
    Cmd::addHandler((char *)"erase", (char *)"Erase received packets", [](Tokens*cmd)->void {for (uint8_t i=0; i<nextPacket; i++) free(radioPackets[i]); nextPacket = 0;});
    Cmd::addHandler((char *)"send", (char *)"Send packet from cmd line", [](Tokens*cmd)->void {txUserBuffer(cmd);});
    Cmd::addHandler((char *)"discovery", (char *)"Send discovery on air", [](Tokens*cmd)->void {discovery();});
    Cmd::addHandler((char *)"verbose", (char *)"Toggle verbose output on packets list", [](Tokens*cmd)->void {verbosity=!verbosity;});
    Cmd::addHandler((char *)"ls", (char *)"List filesystem", [](Tokens*cmd)->void {listFS();});
    Cmd::addHandler((char *)"cat", (char *)"Print file content", [](Tokens*cmd)->void {cat(cmd->at(1).c_str());});
    Cmd::addHandler((char *)"rm", (char *)"Remove file", [](Tokens*cmd)->void {rm(cmd->at(1).c_str());});
    Cmd::addHandler((char *)"key", (char *)"Test keys generation", [](Tokens*cmd)->void {testKey();});
    Cmd::addHandler((char *)"pair", (char *)"1W put device in pair mode", [](Tokens*cmd)->void {remote1W->cmd(IOHC::RemoteButton::Pair);});
    Cmd::addHandler((char *)"add", (char *)"1W add controller to device", [](Tokens*cmd)->void {remote1W->cmd(IOHC::RemoteButton::Add);});
    Cmd::addHandler((char *)"remove", (char *)"1W remove controller from device", [](Tokens*cmd)->void {remote1W->cmd(IOHC::RemoteButton::Remove);});
    Cmd::addHandler((char *)"open", (char *)"1W open device", [](Tokens*cmd)->void {remote1W->cmd(IOHC::RemoteButton::Open);});
    Cmd::addHandler((char *)"close", (char *)"1W close device", [](Tokens*cmd)->void {remote1W->cmd(IOHC::RemoteButton::Close);});
    Cmd::addHandler((char *)"stop", (char *)"1W stop device", [](Tokens*cmd)->void {remote1W->cmd(IOHC::RemoteButton::Stop);});
    Cmd::addHandler((char *)"vent", (char *)"1W vent device", [](Tokens*cmd)->void {remote1W->cmd(IOHC::RemoteButton::Vent);});
    Cmd::addHandler((char *)"force", (char *)"1W force device open", [](Tokens*cmd)->void {remote1W->cmd(IOHC::RemoteButton::ForceOpen);});


    radioInstance->start(MAX_FREQS, frequencies, 0, msgArchive, msgRcvd);
    relTime = millis();

    Serial.printf("------------------------------------------------------\n");
    Serial.printf("------------------------------------------------------\n");
    Serial.printf(" Startup completed. type help to see what you can do!\n");
    Serial.printf("------------------------------------------------------\n");
    Serial.printf("------------------------------------------------------\n");
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
    if (verbosity)
    {
        switch (iohc->payload.packet.header.cmd)
        {
            case 0x2b:
            {
                sysTable->addObject(iohc->payload.packet.header.source, iohc->payload.packet.msg.p0x2b.backbone,
                    iohc->payload.packet.msg.p0x2b.actuator, iohc->payload.packet.msg.p0x2b.manufacturer, iohc->payload.packet.msg.p0x2b.info);
                break;
            }

            case 0x30:
            {
                for (uint8_t idx=0; idx < 16; idx ++)
                    keyCap[idx] = iohc->payload.packet.msg.p0x30.enc_key[idx];

                iohcUtils::encrypt_1W_key((const uint8_t *)iohc->payload.packet.header.source, (uint8_t *)keyCap);
                Serial.printf("Controller key in clear: ");
                for (uint8_t idx=0; idx < 16; idx ++)
                    Serial.printf("%2.2x", keyCap[idx]);
                Serial.printf("\n");
                break;
            }
            
            case 0x39:
            {
                if (keyCap[0] == 0)
                    break;
                uint8_t hmac[6];
                std::vector<uint8_t> frame(&iohc->payload.packet.header.cmd, &iohc->payload.packet.header.cmd+2);
                iohcUtils::create_1W_hmac(hmac, iohc->payload.packet.msg.p0x39.sequence, keyCap, frame);
                Serial.printf("hmac: ");
                for (uint8_t idx=0; idx < 6; idx ++)
                    Serial.printf("%2.2x", hmac[idx]);
                Serial.printf("\n");
                break;
            }
        }
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
        if (!packets2send[0])
            packets2send[0] = (IOHC::iohcPacket *)malloc(sizeof(IOHC::iohcPacket));

        if (cmd->size() == 3)
            packets2send[0]->frequency = frequencies[atoi(cmd->at(2).c_str())-1];
        else
            packets2send[0]->frequency = 0;

        digitalWrite(RX_LED, digitalRead(RX_LED)^1);
        packets2send[0]->buffer_length = hexStringToBytes(cmd->at(1), packets2send[0]->payload.buffer);;
        packets2send[0]->millis = 35;
        packets2send[0]->repeat = 1;
        packets2send[1] = nullptr;

        digitalWrite(RX_LED, digitalRead(RX_LED)^1);
        radioInstance->send(packets2send);
        
        // Do not deallocate buffers as send is asynchronous
//        free(packets2send[0]);
//        packets2send[0] = nullptr;
    }
}

void discovery()
{
    if (!packets2send[0])
        packets2send[0] = (IOHC::iohcPacket *)malloc(sizeof(IOHC::iohcPacket));

    std::string discovery = "d43000003b5cd68f2a578ebc37334d6e2f50a4dfa9";
    packets2send[0]->frequency = 0;

    digitalWrite(RX_LED, digitalRead(RX_LED)^1);
    packets2send[0]->buffer_length = hexStringToBytes(discovery, packets2send[0]->payload.buffer);;
    packets2send[0]->millis = 1750;
    packets2send[0]->repeat = 4;
    packets2send[0]->lock = false;
    packets2send[1] = nullptr;

    digitalWrite(RX_LED, digitalRead(RX_LED)^1);
    radioInstance->send(packets2send);
}

void testKey()
{
    std::string controller_key = "d94a00399a46b5aba67f3809b68ecc52";    // In clear
    std::string node_address = "8ad42e";
    std::string dest_address = "00003f";
//    std::string sequence = "1934";

    uint8_t bcontroller[16];
    uint8_t bnode[3];
    uint8_t bdest[3];
    uint8_t bseq[2] = {0x1a, 0x40}; // <-- Set Sequance number here
    uint8_t output[16];


    hexStringToBytes(controller_key, bcontroller);
    hexStringToBytes(node_address, bnode);
    hexStringToBytes(dest_address, bdest);

    std::string tmp = "f60000003f8ad42e000161d20000001a4731661e6275d2";
    uint8_t btmp[32];
    uint8_t bLen = hexStringToBytes(tmp, btmp);
    std::vector<uint8_t> buffer(btmp, btmp + bLen);
    uint16_t crc = iohcUtils::radioPacketComputeCrc(btmp, bLen);
    Serial.printf("--> %s (%d) crc %2.2x%2.2x <--\t\t", tmp.c_str(), bLen, crc&0x00ff, crc>>8);
    crc = iohcUtils::radioPacketComputeCrc(buffer);
    Serial.printf("--> alt crc %2.2x%2.2x <--\n\n", crc&0x00ff, crc>>8);


    Serial.printf("Node address: ");
    for (uint8_t idx=0; idx < 3; idx ++)
        Serial.printf("%2.2x", bnode[idx]);
    Serial.printf("\n");
    Serial.printf("Dest address: ");
    for (uint8_t idx=0; idx < 3; idx ++)
        Serial.printf("%2.2x", bdest[idx]);
    Serial.printf("\n");
    Serial.printf("Controller key in clear:  ");
    for (uint8_t idx=0; idx < sizeof(bcontroller); idx ++)
        Serial.printf("%2.2x", bcontroller[idx]);
    Serial.printf("\n");

    std::vector<uint8_t> node(bnode, bnode+3);
    std::vector<uint8_t> controller(bcontroller, bcontroller+16);
    std::vector<uint8_t> ret;

    iohcUtils::encrypt_1W_key(bnode, bcontroller);
    Serial.printf("Controller key encrypted: ");
    for (uint8_t idx=0; idx < 16; idx ++)
        Serial.printf("%2.2x", bcontroller[idx]);
    Serial.printf("\n");


    uint16_t test = (bseq[0]<<8)+bseq[1];
    
    for (uint8_t i=0; i<0x20; i++)
    {
        test += 1;
        bseq[1] = test & 0x00ff;
        bseq[0] = test >> 8;

/*
Main parameter: 
0x0000    100% Open
0xd200    Current (used as stop)
0xC800    0% Open
0xD803    Secured ventilation
*/
    //  00 01 61 c800 00 00 1a05
    //    uint8_t bframe[9] = {0x00, 0x01, 0x61, 0xc8, 0x00, 0x02, 0x00, 0x1a, 0x07};
        std::vector<uint8_t> seq(bseq, bseq+2);
        uint8_t bframe1[7] = {0x00, 0x01, 0x61, 0x00, 0x00, 0x02, 0x00}; // <-- Set packet here, excluding sequence number, then adjust length
        std::vector<uint8_t> frame1(bframe1, bframe1+7); // <-- adjust packet length
        hexStringToBytes(controller_key, bcontroller);
        iohcUtils::create_1W_hmac(output, bseq, bcontroller, frame1);

        Serial.printf("Open: f620");
        for (uint8_t idx=0; idx < 3; idx ++)
            Serial.printf("%2.2x", bdest[idx]);
        for (uint8_t idx=0; idx < 3; idx ++)
            Serial.printf("%2.2x", bnode[idx]);
        for (uint8_t idx=0; idx < 7; idx ++) // <-- adjust packet length
            Serial.printf("%2.2x", bframe1[idx]);
        for (uint8_t idx=0; idx < 2; idx ++)
            Serial.printf("%2.2x", bseq[idx]);
        for (uint8_t idx=0; idx < 6; idx ++)
            Serial.printf("%2.2x", output[idx]);
        Serial.printf("\t\t");

        uint8_t bframe2[7] = {0x00, 0x01, 0x61, 0xC8, 0x00, 0x02, 0x00}; // <-- Set packet here, excluding sequence number, then adjust length
        std::vector<uint8_t> frame2(bframe2, bframe2+7); // <-- adjust packet length
        hexStringToBytes(controller_key, bcontroller);
        iohcUtils::create_1W_hmac(output, bseq, bcontroller, frame2);

        Serial.printf("Close: f620");
        for (uint8_t idx=0; idx < 3; idx ++)
            Serial.printf("%2.2x", bdest[idx]);
        for (uint8_t idx=0; idx < 3; idx ++)
            Serial.printf("%2.2x", bnode[idx]);
        for (uint8_t idx=0; idx < 7; idx ++) // <-- adjust packet length
            Serial.printf("%2.2x", bframe2[idx]);
        for (uint8_t idx=0; idx < 2; idx ++)
            Serial.printf("%2.2x", bseq[idx]);
        for (uint8_t idx=0; idx < 6; idx ++)
            Serial.printf("%2.2x", output[idx]);
        Serial.printf("\n");
    }
}