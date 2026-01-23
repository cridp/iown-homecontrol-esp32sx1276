/*
   Copyright (c) 2024-2026. CRIDP https://github.com/cridp

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

           http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */

#include <board-config.h>
#include <fileSystemHelpers.h>
#include <interact.h>
#include <iohcCozyDevice2W.h>
#include <iohcCryptoHelpers.h>
#include <iohcOther2W.h>
#include <iohcRadio.h>
#include <iohcRemote1W.h>
#include <iohcRemoteMap.h>
#include <iohcSystemTable.h>
#include <user_config.h>
#include <ArduinoJson.h>
#include <nvs_helpers.h>
#include <wifi_helper.h>
#include "LittleFS.h"

#if defined(MQTT)
#include <mqtt_handler.h>
#endif
#if defined(WEBSERVER)
#include <web_server_handler.h>
#endif
#if defined(SSD1306_DISPLAY)
#include <oled_display.h>
#endif
#if defined(UECHO)
#include <echonet_handler.hpp>
#endif

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

void txUserBuffer(Tokens *cmd);
void testKey();
void scanDump();
bool publishMsg(IOHC::iohcPacket *iohc);
bool msgRcvd(IOHC::iohcPacket *receivedPacket);
bool msgArchive(IOHC::iohcPacket *iohc);

uint8_t keyCap[16] = {};

IOHC::iohcRadio *radioInstance;
IOHC::iohcPacket *radioPackets[IOHC_INBOUND_MAX_PACKETS];

std::vector<IOHC::iohcPacket *> packets2send{};

uint8_t nextPacket = 0;

IOHC::iohcRemote1W *remote1W;
IOHC::iohcCozyDevice2W *cozyDevice2W;
IOHC::iohcOther2W *otherDevice2W;
// IOHC::iohcSystemTable *sysTable;
// IOHC::iohcRemoteMap *remoteMap;

// Longueur de préambule pour réveiller un appareil distant en veille (ex: gateway)
constexpr uint16_t PREAMBLE_LENGTH_WAKEUP = 0x0068; // 104 symboles
constexpr uint16_t PREAMBLE_LENGTH_DEFAULT = 0x0034; // 52 symboles

uint32_t frequencies[] = FREQS2SCAN;

using namespace IOHC;

void setup() {
    esp_log_level_set("*", ESP_LOG_VERBOSE);    // Or VERBOSE for ESP_LOGV
    Serial.begin(115200);    //Start serial connection for debug and manual input
    pinMode(RX_LED, OUTPUT); // Blink this LED
    digitalWrite(RX_LED, digitalRead(RX_LED) ^ 1);
    Cmd::createCommands();
    Cmd::kbd_tick.attach_ms(500, Cmd::cmdFuncHandler);

#if defined(SSD1306_DISPLAY)
    initDisplay(); // Init OLED display
#endif

    nvs_init();
    // Initialize network services
    // initWifi();
    // Wait for WiFi to be connected before starting services
    // while (WiFi.status() != WL_CONNECTED) {
        // vTaskDelay(100 / portTICK_PERIOD_MS);
    // }

    // set WiFi power mode
    // esp_wifi_set_ps(WIFI_PS_NONE);

#if defined(MQTT)
    initMqtt();
#endif
#if defined(WEBSERVER)
    setupWebServer();
#endif
#if defined(UECHO)
    setup_uEcho();
#endif

    // Mount LittleFS filesystem
    if(!LittleFS.begin()){
        ets_printf("An Error has occurred while mounting LittleFS\n");
        // Handle error appropriately, maybe by halting or indicating failure
        return;
    }
    ets_printf("LittleFS mounted successfully\n");

    radioInstance = IOHC::iohcRadio::getInstance();
#if !defined(MQTT)
// If MQTT is not defined, txCallback is null
    radioInstance->start(MAX_FREQS, frequencies, 0, msgRcvd, nullptr); //msgArchive); //, msgRcvd);

#else
    radioInstance->start(MAX_FREQS, frequencies, 0, msgRcvd, publishMsg);
#endif

    // sysTable = IOHC::iohcSystemTable::getInstance();
    // remoteMap = IOHC::iohcRemoteMap::getInstance();

    remote1W = IOHC::iohcRemote1W::getInstance();
    cozyDevice2W = IOHC::iohcCozyDevice2W::getInstance();
    otherDevice2W = IOHC::iohcOther2W::getInstance();

    ets_printf("Startup completed. type help to see what you can do!\n");
    digitalWrite(RX_LED, 0);

    // Verification logic to match Python implementation for deriving system_key
    // challenge used from 0x3C received after sending 0x31
    // Challenge utilisé (CMD 31/3C): f4bf794c01bd
    // encrypted_key = bytes.fromhex("72c552ce5183f740c806e30ad8d3733c")
    // expected_key = bytes.fromhex("777cb0cf4fdc591d4c8173637e1b7013")
    std::vector<uint8_t> challengeAsked = {0xf4, 0xbf, 0x79, 0x4c, 0x01, 0xbd};
    std::vector<uint8_t> encrypted_key = {0x72, 0xc5, 0x52, 0xce, 0x51, 0x83, 0xf7, 0x40, 0xc8, 0x06, 0xe3, 0x0a, 0xd8, 0xd3, 0x73, 0x3c};
    
    std::vector<uint8_t> IVdata = {IOHC::iohcDevice::ASK_CHALLENGE_0x31};

    // 1. Construct IV and encrypt it with transfer_key to get the keystream
    std::vector<uint8_t> keystream = iohcCrypto::encrypt_2W_payload(IVdata, challengeAsked, iohcCrypto::transfert_key);

    // 2. XOR the encrypted_key with the keystream to get the final system_key
    std::vector<uint8_t> calculated_system_key = encrypted_key; // copy
    for (size_t i = 0; i < calculated_system_key.size(); i++) {
        calculated_system_key[i] ^= keystream[i];
    }

    ets_printf("Calculated 2W SYSTEM_KEY (KEEP IT PRIVATE): ");
    for (unsigned char idx: calculated_system_key)
        ets_printf("%2.2X", idx);
    ets_printf("\n");
    ets_printf("Expected 2W SYSTEM_KEY: 777CB0CF4FDC591D4C8173637E1B7013\n");
}

/**
* @brief Creates a common iohcPacket with the given data to send.
* @param packet * The packet you want to forge
* @param toSend The data that will be added to the packet
*/
void forgePacket(iohcPacket* packet, const std::vector<uint8_t> &toSend) {
    IOHC::packetStamp = esp_timer_get_time();

    // Common Flags
    packet->payload.packet.header.CtrlByte1.asStruct.Protocol = 0;
    packet->payload.packet.header.CtrlByte1.asStruct.StartFrame = 1;
    packet->payload.packet.header.CtrlByte1.asStruct.EndFrame = 0;
    packet->payload.packet.header.CtrlByte1.asByte += toSend.size();
    memcpy(packet->payload.buffer + 9, toSend.data(), toSend.size());
    packet->buffer_length = toSend.size() + 9;
}

bool msgRcvd(IOHC::iohcPacket *receivedPacket) {
    // radioInstance->maybeCheckHeap("ENTER IN msgRcvd");
    JsonDocument doc;
    doc["type"] = "Unk";
    if (Cmd::scanMode) {
        otherDevice2W->memorizeOther2W = {};
        otherDevice2W->mapValid[IOHC::lastCmd] = receivedPacket->cmd();
    }
    otherDevice2W->stopDiscover = false;

    switch (receivedPacket->cmd()) {
        case iohcDevice::DISCOVER_0x28: {
            if (!Cmd::pairMode) break;

            // Node type and subtype (2 bytes): type on 10 bits and subtype on the remainer
            // Node type = (field >> 6) & 1023
            // Node subtype = field & 63
            // Node address (3 bytes)
            // Manufacturer ID (1 byte)
            // Multiinfo (1 byte)
            // Timestamp (2 bytes)

            // 0x0b OverKiz 0x0c Atlantic
            // std::vector<uint8_t> toSend = {0xff, 0xc0, 0xba, 0x11, 0xad, 0x0b, 0xcc, 0x00, 0x00};
            // Sunblind dyna 0x344e1f fabricant 0x02 dc9c88 04 00 34 4E 1F 02 DC D5 7B
            std::vector<uint8_t> toSend = {0x04, 0x00, 0x34, 0x4E, 0x1F, 0x02, 0xDC, 0x9C, 0x88};

            // auto* packet = new iohcPacket();
            iohcPacket response;

            forgePacket(&response, toSend);

            response.payload.packet.header.cmd = IOHC::iohcDevice::DISCOVER_ANSWER_0x29;

            /* Swap */
            memcpy(response.payload.packet.header.source, cozyDevice2W->gateway, 3);
            memcpy(response.payload.packet.header.target, receivedPacket->payload.packet.header.source, 3);

            response.repeatTime = 50;

            response.frequency = receivedPacket->frequency;
            radioInstance->sendPriority(&response);

            break;
        }
        case iohcDevice::DISCOVER_ANSWER_0x29: {
            otherDevice2W->stopDiscover = true;
            // ets_printf("2W Device want to be paired Waiting for 0x2C\n");
            //
            // std::vector<uint8_t> deviceAsked;
            // deviceAsked.assign(iohc->payload.buffer + 9, iohc->payload.buffer + 18);
            // for (unsigned char i: deviceAsked) {
            //     ets_printf("%02X ", i);
            // }
            // ets_printf("\n");
            //
            if (!Cmd::pairMode) break;

            // printf("Sending 0x38 \n");
            ets_printf("Sending DISCOVER_ACTUATOR_0x2C \n");

            // std::vector<uint8_t> toSend = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06}; // 38
            std::vector<uint8_t> toSend = {}; // SEND_DISCOVER_ACTUATOR_0x2C

            // auto* packet = new iohcPacket();
            iohcPacket response;

            forgePacket(&response, toSend);

            // packet->payload.packet.header.cmd = 0x38;
            response.payload.packet.header.cmd = iohcDevice::DISCOVER_ACTUATOR_0x2C;

            /* Swap */
            memcpy(response.payload.packet.header.source, receivedPacket->payload.packet.header.target, 3);
            memcpy(response.payload.packet.header.target, receivedPacket->payload.packet.header.source, 3);

            response.frequency = receivedPacket->frequency;
            radioInstance->sendPriority(&response);

            break;
        }
        case iohcDevice::DISCOVER_REMOTE_ANSWER_0x2B: {
            otherDevice2W->stopDiscover = true;
            // sysTable->addObject(iohc->payload.packet.header.source, iohc->payload.packet.msg.p0x2b.backbone,
            //                     iohc->payload.packet.msg.p0x2b.actuator, iohc->payload.packet.msg.p0x2b.manufacturer,
            //                     iohc->payload.packet.msg.p0x2b.info);
            break;
        }
        case iohcDevice::DISCOVER_ACTUATOR_0x2C: {
            if (!Cmd::pairMode) break;

            std::vector<uint8_t> toSend = {};

            iohcPacket response;

            forgePacket(&response, toSend);

            response.payload.packet.header.cmd = IOHC::iohcDevice::DISCOVER_ACTUATOR_ACK_0x2D;

            /* Swap */
            memcpy(response.payload.packet.header.source, receivedPacket->payload.packet.header.target, 3);
            memcpy(response.payload.packet.header.target, receivedPacket->payload.packet.header.source, 3);

            response.repeatTime = 50;
            response.frequency = receivedPacket->frequency;

            radioInstance->sendPriority(&response);

            break;
        }
        case iohcDevice::LAUNCH_KEY_TRANSFERT_0x38: {
            std::vector<uint8_t> key_transfert;
            key_transfert.assign(receivedPacket->payload.buffer + 9, receivedPacket->payload.buffer + 15);

            for (unsigned char i: key_transfert) {
                ets_printf("%02X ", i);
            }
            ets_printf("\n");

            std::vector<uint8_t> data = {IOHC::iohcDevice::ASK_CHALLENGE_0x31}; //0x38
            std::vector<uint8_t> initial_value = iohcCrypto::constructInitialValue(data, key_transfert.data(), nullptr);
            ets_printf("2) Initial value used for key encryption: ");
            for (unsigned char i: initial_value) {
                ets_printf("%02X ", i);
            }
            ets_printf("\n");

            std::vector<uint8_t> encrypted_key = iohcCrypto::encrypt_2W_payload(data, key_transfert, iohcCrypto::transfert_key);
            //  XORing transfert_key
            for (int i = 0; i < 16; i++) {
                encrypted_key[i] = encrypted_key[i] ^ iohcCrypto::transfert_key[i];
            }
            ets_printf("2) Encrypted 2-way key to be sent with SEND_KEY_TRANSFERT_0x32: ");
            for (unsigned char i: encrypted_key) {
                ets_printf("%02X ", i);
            }
            ets_printf("\n");
            if (!Cmd::pairMode) break;

            iohcPacket response;
            forgePacket(&response, encrypted_key);

            response.payload.packet.header.cmd = IOHC::iohcDevice::KEY_TRANSFERT_0x32;

            /* Swap */
            memcpy(response.payload.packet.header.source, receivedPacket->payload.packet.header.target, 3);
            memcpy(response.payload.packet.header.target, receivedPacket->payload.packet.header.source, 3);

            // response.frequency = receivedPacket->frequency;
            radioInstance->sendPriority(&response);
            break;
        }
        case iohcDevice::WRITE_PRIVATE_0x20:  {
            // cozyDevice2W->memorizeSend.memorizedCmd = receivedPacket->payload.packet.header.cmd;
            // IOHC::lastCmd = receivedPacket->payload.packet.header.cmd;
            break;
        }
        case iohcDevice::PRIVATE_ACK_0x21: {
#if defined (MQTT)
            // Answer of 0x20, publish the confirmed command
            // doc["type"] = "Cozy";
            // doc["from"] = bytesToHexString(iohc->payload.packet.header.target, 3);
            // doc["to"] = bytesToHexString(iohc->payload.packet.header.source, 3);
            // doc["cmd"] = to_hex_str(iohc->payload.packet.header.cmd).c_str();
            // doc["_data"] = bytesToHexString(iohc->payload.buffer + 9, iohc->buffer_length - 9);
            // std::string message;
            // size_t messageSize = serializeJson(doc, message);
            // mqttClient.publish("iown/Frame", 0, false, message.c_str(), messageSize);
#endif
            break;
        }
        case iohcDevice::CHALLENGE_REQUEST_0x3C: {
            // Answer only to our fake gateway, not to others real devices
            if (true) {
            // if (cozyDevice2W->isFake(receivedPacket->payload.packet.header.source, receivedPacket->payload.packet.header.target)) {
                doc["type"] = "Gateway";

                // if (Cmd::scanMode) {
                //     otherDevice2W->mapValid[IOHC::lastSendCmd] = iohcDevice::CHALLENGE_REQUEST_0x3C;
                //     break;
                // }

                // IVdata is the challenge with commandId put on start
                std::vector<uint8_t> challengeAsked = receivedPacket->data();

                iohcPacket response;
                std::vector<uint8_t> toSend;

                if (IOHC::lastCmd == IOHC::iohcDevice::ASK_CHALLENGE_0x31) {
                    response.payload.packet.header.cmd = IOHC::iohcDevice::KEY_TRANSFERT_0x32;
                    std::vector<uint8_t> IVdata = {IOHC::iohcDevice::ASK_CHALLENGE_0x31};
                    toSend = iohcCrypto::encrypt_2W_payload(IVdata, challengeAsked, iohcCrypto::transfert_key);
                    for (size_t i = 0; i < toSend.size(); i++)
                        toSend[i] ^= iohcCrypto::transfert_key[i];
                } else {
                    response.payload.packet.header.cmd = IOHC::iohcDevice::CHALLENGE_ANSWER_0x3D;
                    std::vector<uint8_t> IVdata = IOHC::lastData;
                    IVdata.insert(IVdata.begin(), IOHC::lastCmd);
                    // The challenge response for regular commands seems to use the transfert_key, not the system_key.
                    // This is unusual but matches the behavior of the original implementation.
                    toSend = iohcCrypto::encrypt_2W_payload(IVdata, challengeAsked, iohcCrypto::transfert_key);
                    toSend.resize(6);
                }

                forgePacket(&response, toSend);

                cozyDevice2W->memorizeSend.memorizedCmd = response.payload.packet.header.cmd;
                cozyDevice2W->memorizeSend.memorizedData = toSend;

                /* Swap */
                memcpy(response.payload.packet.header.source, receivedPacket->payload.packet.header.target, 3);
                memcpy(response.payload.packet.header.target, receivedPacket->payload.packet.header.source, 3);

                response.payload.packet.header.CtrlByte1.asStruct.StartFrame = 0;

                // For remote devices, a longer preamble is needed
                // to allow their AFC (Automatic Frequency Control) to stabilize.
                // Radio::setPreambleLength(PREAMBLE_LENGTH_WAKEUP);
                // Timing is critical. A specific delay is necessary for the gateway to accept the response.
                // The window appears to be between 9 and 12 ms.
                response.repeatTime = 12;

                radioInstance->sendPriority(&response);

                // Restaurer la longueur de préambule par défaut pour les communications suivantes.
                // Radio::setPreambleLength(PREAMBLE_LENGTH_DEFAULT);
            }
            break;
        }
        case iohcDevice::GET_NAME_0x50: {
            if (cozyDevice2W->isFake(receivedPacket->payload.packet.header.source, receivedPacket->payload.packet.header.target)) {
            // MY_GATEWAY 4d595f47415445574159
            std::vector<uint8_t> toSend = {0x4d, 0x59, 0x5f, 0x47, 0x41, 0x54, 0x45, 0x57, 0x41, 0x59};
            toSend.resize(16);

            iohcPacket response;

            forgePacket(&response, toSend);

            response.payload.packet.header.cmd = 0x51;

            /* Swap */
            memcpy(response.payload.packet.header.source, cozyDevice2W->gateway, 3);
            memcpy(response.payload.packet.header.target, receivedPacket->payload.packet.header.source, 3);

            response.repeatTime = 50;
            response.frequency = receivedPacket->frequency;

            radioInstance->sendPriority(&response);

            }
            break;
        }
        case iohcDevice::GET_NAME_ANSWER_0x51: {
            // in iohcPacket
            break;
        }
        case 0x01: if (!receivedPacket->is1W()) {IOHC::lastCmd = 0x01; break;}
        case 0X00: if (!receivedPacket->is1W()) {IOHC::lastCmd = 0x00;}
        case 0x03: {
            if (receivedPacket->is1W() && receivedPacket->cmd() == 0x00) {
                // doc["type"] = "1W";
                uint16_t main = (receivedPacket->payload.packet.msg.p0x00_14.main[0] << 8) | receivedPacket->payload.packet.msg.p0x00_14.main[1];
                auto action = "unknown";
                switch (main) {
                    case 0x0000: action = "OPEN"; break;
                    case 0xC800: action = "CLOSE"; break;
                    case 0xD200: action = "STOP"; break;
                    case 0xD803: action = "VENT"; break;
                    case 0x6400: action = "FORCE"; break;
                    default: break;
                }
                doc["action"] = action;
#if defined(SSD1306_DISPLAY)
                display1WAction(receivedPacket->payload.packet.header.source, action, "RX");
#endif
#if defined(MQTT)
                if (const auto *map = remoteMap->find(receivedPacket->payload.packet.header.source)) {
                    const auto &remotes = iohcRemote1W::getInstance()->getRemotes();
                    for (const auto &desc : map->devices) {
                        auto rit = std::find_if(remotes.begin(), remotes.end(), [&](const auto &r){
                            return r.description == desc;
                        });
                        if (rit != remotes.end()) {
                            std::string id = bytesToHexString(rit->node, sizeof(rit->node));

                            std::string topic = "iown/" + id + "/state";
                            mqttClient.publish(topic.c_str(), 0, false, action);

                        }
                    }
                }
#endif
            } else if (!receivedPacket->is1W() && receivedPacket->cmd() == 0x03){
                doc["type"] = "Other";
                IOHC::lastCmd = 0x03;
                IOHC::lastData = receivedPacket->data();
                // For the fun....
                iohcPacket response;

//                response.payload.packet.header.cmd = 0x3C;
//                std::vector<uint8_t> toSend = {0xab, 0xba, 0xab, 0xba, 0xab, 0xba};
                response.payload.packet.header.cmd = 0x04;
                std::vector<uint8_t> toSend = {0x05 , 0x00, 0xc8, 0x00 , 0xc8, 0x00, 0x00, 0x00 , 0x32, 0x99, 0xd8 , 0x01, 0x00, 0x00};

                memcpy(toSend.data() + 8, receivedPacket->payload.packet.header.source, 3);
                forgePacket(&response, toSend);

                /* Swap */
                memcpy(response.payload.packet.header.source, receivedPacket->payload.packet.header.target, 3);
                memcpy(response.payload.packet.header.target, receivedPacket->payload.packet.header.source, 3);

                response.payload.packet.header.CtrlByte1.asStruct.StartFrame = 0;
                response.payload.packet.header.CtrlByte1.asStruct.EndFrame = 1;

                response.repeatTime = 10;
                radioInstance->sendSingle(&response);
            }
            break;
        }
        case 0x04:
        case 0x0D:
        case iohcDevice::DISCOVER_ACTUATOR_ACK_0x2D:
        case 0x4B:
        case 0x55:
        case 0x57:
        case 0x59: {
            // if (Cmd::scanMode) {
            //     otherDevice2W->memorizeOther2W = {};
            //     // printf(" Answer %X Cmd %X ", iohc->payload.packet.header.cmd, IOHC::lastSendCmd);
            //     otherDevice2W->mapValid[IOHC::lastSendCmd] = iohc->payload.packet.header.cmd;
            // }
        break;
    }
        case iohcDevice::STATUS_0xFE: {
            if (Cmd::scanMode) {
                otherDevice2W->memorizeOther2W = {};
                otherDevice2W->mapValid[IOHC::lastCmd] = receivedPacket->cmd();// payload.buffer[9];
            }
            break;
        }
        case 0x30: {
            for (uint8_t idx = 0; idx < 16; idx++)
                keyCap[idx] = receivedPacket->payload.packet.msg.p0x30.enc_key[idx];

            iohcCrypto::encrypt_1W_key(receivedPacket->payload.packet.header.source, keyCap);
            ets_printf("CLEAR KEY: ");
            for (unsigned char idx: keyCap)
                ets_printf("%2.2X", idx);
            ets_printf("\n");
            break;
        }
        case 0X2E: {
            // ets_printf("In 1W Learning mode\n");
            break;
        }
        case 0x39: {
            if (keyCap[0] == 0) break;
            uint8_t hmac[16];
            std::vector<uint8_t> frame(&receivedPacket->payload.packet.header.cmd, &receivedPacket->payload.packet.header.cmd + 2);
            // frame = {0x39, 0x00}; //
            iohcCrypto::create_1W_hmac(hmac, receivedPacket->payload.packet.msg.p0x39.sequence, keyCap, frame);
            ets_printf("MAC: ");
            for (uint8_t idx = 0; idx < 6; idx++)
                ets_printf("%2.2X", hmac[idx]);
            ets_printf("\n");
            break;
        }
        case iohcDevice::CHALLENGE_ANSWER_0x3D: // TODO save
        {
            // This is where we verify if a received challenge answer is valid.
//            ets_printf("Received a 0x3D challenge answer. Verifying...\n");
            
            // 1. Get the encrypted data from the received packet.
//            std::vector<uint8_t> encrypted_data = receivedPacket->data();
            
            // 2. Decrypt it using the system key.
//            std::vector<uint8_t> decrypted_payload = iohcCrypto::decrypt_2W_payload(encrypted_data, iohcCrypto::transfert_key);

            // 3. Reconstruct the expected IV based on the last command we sent.
//            std::vector<uint8_t> iv_data_base = cozyDevice2W->memorizeSend.memorizedData;
//            iv_data_base.insert(iv_data_base.begin(), cozyDevice2W->memorizeSend.memorizedCmd);
            // We can't know the challenge used by the other remote, so we only construct the first part of the IV.
//            std::vector<uint8_t> expected_iv = iohcCrypto::constructInitialValue(iv_data_base, std::vector<uint8_t>(6, 0).data(), nullptr);

//            ets_printf("Decrypted IV from 0x3D:       ");
//            for (int i = 0; i < 8; i++) ets_printf("%02X ", decrypted_payload[i]);
//            ets_printf("\n");

//            ets_printf("Expected IV base (from us): ");
//            for (int i = 0; i < 8; i++) ets_printf("%02X ", expected_iv[i]);
//            ets_printf("\n");
            break;
        }
        case 0x2A:
        case 0x48:
        case 0x49:
        case 0x4A:
        case 0X05:
        case 0x19:
        case 0x0C: break;
        case 0x31:
        case 0x32:
        case 0x33:
        case 0x46:
        case 0x47:
        case 0x54:
        case 0x56:
        case 0x37: break;
        default:
            ets_printf("Received Unknown command %02X ", receivedPacket->cmd());
            return false;
            break;
    }
    if (receivedPacket->is1W()) {
        doc["type"] = "1W";
    } else {
        doc["type"] = "2W";
    }

#if defined(MQTT)
    publishMsg(receivedPacket);
#endif
    // radioInstance->maybeCheckHeap("EXIT FROM msgRcvd");
    return true;
}
#if defined(MQTT)
/**
 * The function creates a JSON message from an `iohcPacket` object and publishes it using
 * MQTT if enabled.
 * 
 * @param iohc The `iohc` parameter is a pointer to an object of type `IOHC::iohcPacket`. The function
 * `publishMsg` takes this pointer as input and processes the data within the `iohc` object to create a
 * JSON message and publish it using MQTT if the conditions are met.
 * 
 * @return The function `publishMsg` is returning `false`.
 */
bool publishMsg(IOHC::iohcPacket *iohc) {
    //                if(iohc->payload.packet.header.cmd == 0x20 || iohc->payload.packet.header.cmd == 0x00) {
    JsonDocument doc;

    doc["type"] = "Cozy";
    doc["from"] = bytesToHexString(iohc->payload.packet.header.target, 3);
    doc["to"] = bytesToHexString(iohc->payload.packet.header.source, 3);
    doc["cmd"] = to_hex_str(iohc->payload.packet.header.cmd).c_str();
    doc["_data"] = bytesToHexString(iohc->payload.buffer + 9, iohc->buffer_length - 9);
    if (remoteMap) {
        if (const auto *map = remoteMap->find(iohc->payload.packet.header.source)) {
            doc["remote"] = map->name;
        }
    }

    if (iohc->payload.packet.header.CtrlByte1.asStruct.Protocol == 1 &&
        iohc->payload.packet.header.cmd == 0x00) {
        uint16_t main =
                (iohc->payload.packet.msg.p0x00_14.main[0] << 8) |
                iohc->payload.packet.msg.p0x00_14.main[1];
        const char *action = "unknown";
        switch (main) {
            case 0x0000: action = "open"; break;
            case 0xC800: action = "close"; break;
            case 0xD200: action = "stop"; break;
            case 0xD803: action = "vent"; break;
            case 0x6400: action = "force"; break;
            default: break;
        }
        doc["type"] = "1W";
        doc["action"] = action;
    }

    std::string message;
    size_t messageSize = serializeJson(doc, message);
    mqttClient.publish("iown/Frame", 1, false, message.c_str(), messageSize);
    mqttClient.publish("homeassistant/sensor/iohc_frame/state", 0, false, message.c_str(), messageSize);
    return false;
}
#endif

/**
 * @deprecated
 * The function copies data from one `iohcPacket` object to another and stores it in an
 * array, returning true if successful and false if there are not enough buffers available.
 * 
 * @param iohc Pointer to an object of type
 * `IOHC::iohcPacket`. This object contains information such as buffer length, frequency, RSSI
 * (Received Signal Strength Indication), and payload data. The function `msgArchive` is
 * 
 * @return `true` if the operation is successful
 * and `false` if there is a failure condition detected during the execution of the function.
 */
// bool msgArchive(IOHC::iohcPacket *iohc) {
//     radioPackets[nextPacket] = new IOHC::iohcPacket;
//     if (!radioPackets[nextPacket]) {
//         ets_printf("*** Malloc failed!\n");
//         return false;
//     }
//
//     radioPackets[nextPacket]->buffer_length = iohc->buffer_length;
//     radioPackets[nextPacket]->frequency = iohc->frequency;
//     //    radioPackets[nextPacket]->stamp = iohc->stamp;
//     radioPackets[nextPacket]->rssi = iohc->rssi;
//
//     for (uint8_t i = 0; i < iohc->buffer_length; i++)
//         radioPackets[nextPacket]->payload.buffer[i] = iohc->payload.buffer[i];
//
//     nextPacket += 1;
//     Serial.printf("-> %d\r", nextPacket);
//     if (nextPacket >= IOHC_INBOUND_MAX_PACKETS) {
//         nextPacket = IOHC_INBOUND_MAX_PACKETS - 1;
//         Serial.printf("*** Not enough buffers available. Please erase current ones\n");
//         return false;
//     }
//
//     return true;
// }

/**
 * @deprecated
 * The function `txUserBuffer` sends a packet using a radio instance based on the input command and
 * frequency.
 * 
 * @param cmd The `cmd` parameter is a pointer to a `Tokens` object. It seems like the `Tokens` class
 * has a method `size()` that returns the size of the object, and an `at()` method that retrieves a
 * specific element at a given index. The function `txUserBuffer`
 * 
 * @return nothing (void).
 */
// void txUserBuffer(Tokens *cmd) {
//     if (cmd->size() < 2) {
//         ets_printf("No packet to be sent!\n");
//         return;
//     }
//     digitalWrite(RX_LED, digitalRead(RX_LED) ^ 1);
//     if (!packets2send[0])
//         packets2send[0] = new IOHC::iohcPacket;
//
//     if (cmd->size() == 3)
//         packets2send[0]->frequency = frequencies[atoi(cmd->at(2).c_str()) - 1];
//     else
//         packets2send[0]->frequency = 0;
//
//     packets2send[0]->buffer_length = hexStringToBytes(cmd->at(1), packets2send[0]->payload.buffer);
//     packets2send[0]->repeatTime = 35;
//     packets2send[0]->repeat = 1;
//     // packets2send[1] = nullptr; // Not needed with std::vector and push_back
//
//     radioInstance->send(packets2send);
//     digitalWrite(RX_LED, digitalRead(RX_LED) ^ 1);
// }

void loop() {
    // loopWebServer(); // For ESPAsyncWebServer, this is typically not needed.
    // checkWifiConnection();
}
