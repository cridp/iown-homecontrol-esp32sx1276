#include <iohcRemote1W.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <iohcCryptoHelpers.h>


namespace IOHC {
    iohcRemote1W* iohcRemote1W::_iohcRemote1W = nullptr;

    iohcRemote1W::iohcRemote1W() = default;

    iohcRemote1W* iohcRemote1W::getInstance() {
        if (!_iohcRemote1W) {
            _iohcRemote1W = new iohcRemote1W();
            _iohcRemote1W->load();
        }
        return _iohcRemote1W;
    }

    void iohcRemote1W::init(iohcPacket* packet, size_t typn) {
        //           for (size_t typn=0; typn<_type.size(); typn++) { // Pre-allocate packets vector; one packet for each remote type loaded
        // Pas besoin de new ici, std::array gère automatiquement la mémoire
        // Vous pouvez directement accéder à l'élément avec l'opérateur [].
        //            auto& packet = packets2send[typn];
        //                packets2send[typn] = new iohcPacket; //(iohcPacket *) malloc(sizeof(iohcPacket));
        //            else
        //                memset((void *)packets2send[typn]->payload.buffer, 0x00, sizeof(packets2send[typn]->payload.buffer));
        // Common Flags
        packet->payload.packet.header.CtrlByte1.asStruct.MsgLen = sizeof(_header) - 1;
        packet->payload.packet.header.CtrlByte1.asStruct.Protocol = 1;
        packet->payload.packet.header.CtrlByte1.asStruct.StartFrame = 1;
        packet->payload.packet.header.CtrlByte1.asStruct.EndFrame = 1;
        packet->payload.packet.header.CtrlByte2.asByte = 0;
        packet->payload.packet.header.CtrlByte2.asStruct.LPM = 1;
        // Broadcast Target
        uint16_t bcast = (_type.at(typn) << 6) + 0b111111;
        packet->payload.packet.header.target[0] = 0x00;
        packet->payload.packet.header.target[1] = bcast >> 8;
        packet->payload.packet.header.target[2] = bcast & 0x00ff;

        packet->frequency = CHANNEL2;
        packet->repeatTime = 50;
        packet->repeat = 4;
        packet->lock = true;
        // }
    }

    std::vector<uint8_t> frame;

    void iohcRemote1W::cmd(RemoteButton cmd) {
        // Emulates remote button press
        switch (cmd) {
            case RemoteButton::Pair: // 0x2e: 0x1120 + target broadcast + source + 0x2e00 + sequence + hmac
                IOHC::relStamp = esp_timer_get_time();
                for (size_t typn = 0; typn < _type.size(); typn++) {
                    digitalWrite(RX_LED, digitalRead(RX_LED) ^ 1);

                    auto&packet = packets2send[typn];
                    _iohcRemote1W->init(packet, typn);
                    // Packet length
                    packet->payload.packet.header.CtrlByte1.asStruct.MsgLen += sizeof(_p0x2e);

                    // Source (me)
                    for (size_t i = 0; i < sizeof(address); i++)
                        packet->payload.packet.header.source[i] = _node[i];

                    //Command
                    packet->payload.packet.header.cmd = 0x2e;
                    // Data
                    packet->payload.packet.msg.p0x2e.data = 0x00;
                    // Sequence
                    packet->payload.packet.msg.p0x2e.sequence[0] = _sequence >> 8;
                    packet->payload.packet.msg.p0x2e.sequence[1] = _sequence & 0x00ff;
                    _sequence += 1;
                    // hmac
                    frame = std::vector(&packet->payload.packet.header.cmd, &packet->payload.packet.header.cmd + 2);
                    uint8_t hmac[6];
                    iohcCrypto::create_1W_hmac(hmac, packet->payload.packet.msg.p0x2e.sequence, _key, frame);

                    for (uint8_t i = 0; i < 6; i++)
                        packet->payload.packet.msg.p0x2e.hmac[i] = hmac[i];

                    packet->buffer_length = packet->payload.packet.header.CtrlByte1.asStruct.MsgLen + 1;

                    // if (typn) packet->payload.packet.header.CtrlByte2.asStruct.LPM = 0; //TODO only first is LPM
                    //                    packets2send[typn+1] = nullptr;
                    //                    packet->decode(); //  KLI 1W
                    digitalWrite(RX_LED, digitalRead(RX_LED) ^ 1);
                }
                    _radioInstance->send(packets2send);
                    break;

            case RemoteButton::Remove: // 0x39: 0x1c00 + target broadcast + source + 0x3900 + sequence + hmac
                IOHC::relStamp = esp_timer_get_time();
                for (size_t typn = 0; typn < _type.size(); typn++) {
                    digitalWrite(RX_LED, digitalRead(RX_LED) ^ 1);

                    auto&packet = packets2send[typn];
                    _iohcRemote1W->init(packet, typn);
                    // Packet length
                    //                    packet->payload.packet.header.CtrlByte1.asStruct.MsgLen = sizeof(_header) - 1;
                    packet->payload.packet.header.CtrlByte1.asStruct.MsgLen += sizeof(_p0x2e);

                    // Source (me)
                    for (size_t i = 0; i < sizeof(address); i++)
                        packet->payload.packet.header.source[i] = _node[i];

                    //Command
                    packet->payload.packet.header.cmd = 0x39;
                    // Data
                    packet->payload.packet.msg.p0x2e.data = 0x00;
                    // Sequence
                    packet->payload.packet.msg.p0x2e.sequence[0] = _sequence >> 8;
                    packet->payload.packet.msg.p0x2e.sequence[1] = _sequence & 0x00ff;
                    _sequence += 1;
                    // hmac
                    uint8_t hmac[6];
                    frame = std::vector(&packet->payload.packet.header.cmd, &packet->payload.packet.header.cmd + 2);
                    iohcCrypto::create_1W_hmac(hmac, packets2send[typn]->payload.packet.msg.p0x2e.sequence, _key,
                                               frame);
                    for (uint8_t i = 0; i < 6; i++)
                        packet->payload.packet.msg.p0x2e.hmac[i] = hmac[i];

                    packet->buffer_length = packet->payload.packet.header.CtrlByte1.asStruct.MsgLen + 1;

                    //                  packet->decode(); //  KLI 1W
                    digitalWrite(RX_LED, digitalRead(RX_LED) ^ 1);
                }
                    _radioInstance->send(packets2send);
                    //printf("\n");
                    break;

            case RemoteButton::Add: // 0x30: 0x1100 + target broadcast + source + 0x3000 + ???
                IOHC::relStamp = esp_timer_get_time();
                for (size_t typn = 0; typn < _type.size(); typn++) {
                    digitalWrite(RX_LED, digitalRead(RX_LED) ^ 1);

                    auto&packet = packets2send[typn];
                    _iohcRemote1W->init(packet, typn);
                    // Packet length
                    packet->payload.packet.header.CtrlByte1.asStruct.MsgLen += sizeof(_p0x30);

                    // Source (me)
                    for (size_t i = 0; i < sizeof(address); i++)
                        packet->payload.packet.header.source[i] = _node[i];
                    //Command
                    packet->payload.packet.header.cmd = 0x30;

                    // Encrypted key
                    uint8_t encKey[16];
                    memcpy(encKey, _key, 16);
                    iohcCrypto::encrypt_1W_key(_node, encKey);
                    memcpy(packet->payload.packet.msg.p0x30.enc_key, encKey, 16);

                    // Manufacturer
                    packet->payload.packet.msg.p0x30.man_id = _manufacturer;
                    // Data
                    packet->payload.packet.msg.p0x30.data = 0x01;
                    // Sequence
                    packet->payload.packet.msg.p0x30.sequence[0] = _sequence >> 8;
                    packet->payload.packet.msg.p0x30.sequence[1] = _sequence & 0x00ff;
                    _sequence += 1;

                    packet->buffer_length = packet->payload.packet.header.CtrlByte1.asStruct.MsgLen + 1;

                    //                    packet->decode();
                    digitalWrite(RX_LED, digitalRead(RX_LED) ^ 1);
                }
                    _radioInstance->send(packets2send);
                    //                packets2send[0]->decode(); //  KLI 1W
                    //printf("\n");
                    break;
            case RemoteButton::testKey: {
                std::string controller_key = "d94a00399a46b5aba67f3809b68ecc52"; // In clear
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
                std::vector buffer(btmp, btmp + bLen);
                uint16_t crc = iohcCrypto::radioPacketComputeCrc(btmp, bLen);
                Serial.printf("--> %s (%d) crc %2.2x%2.2x <--\t\t", tmp.c_str(), bLen, crc & 0x00ff, crc >> 8);
                crc = iohcCrypto::radioPacketComputeCrc(buffer);
                Serial.printf("--> alt crc %2.2x%2.2x <--\n\n", crc & 0x00ff, crc >> 8);


                Serial.printf("Node address: ");
                for (unsigned char idx: bnode)
                    Serial.printf("%2.2x", idx);
                Serial.printf("\n");
                Serial.printf("Dest address: ");
                for (unsigned char idx: bdest)
                    Serial.printf("%2.2x", idx);
                Serial.printf("\n");
                Serial.printf("Controller key in clear:  ");
                for (unsigned char idx: bcontroller)
                    Serial.printf("%2.2x", idx);
                Serial.printf("\n");

                std::vector node(bnode, bnode + 3);
                std::vector controller(bcontroller, bcontroller + 16);
                std::vector<uint8_t> ret;

                iohcCrypto::encrypt_1W_key(bnode, bcontroller);
                Serial.printf("Controller key encrypted: ");
                for (unsigned char idx: bcontroller)
                    Serial.printf("%2.2x", idx);
                Serial.printf("\n");


                uint16_t test = (bseq[0] << 8) + bseq[1];

                for (uint8_t i = 0; i < 0x20; i++) {
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
                    std::vector seq(bseq, bseq + 2);
                    uint8_t bframe1[7] = {0x00, 0x01, 0x61, 0x00, 0x00, 0x02, 0x00};
                    // <-- Set packet here, excluding sequence number, then adjust length
                    std::vector frame1(bframe1, bframe1 + 7); // <-- adjust packet length
                    hexStringToBytes(controller_key, bcontroller);
                    iohcCrypto::create_1W_hmac(output, bseq, bcontroller, frame1);

                    Serial.printf("Open: f620");
                    for (unsigned char idx: bdest)
                        Serial.printf("%2.2x", idx);
                    for (unsigned char idx: bnode)
                        Serial.printf("%2.2x", idx);
                    for (unsigned char idx: bframe1) // <-- adjust packet length
                        Serial.printf("%2.2x", idx);
                    for (unsigned char idx: bseq)
                        Serial.printf("%2.2x", idx);
                    for (uint8_t idx = 0; idx < 6; idx++)
                        Serial.printf("%2.2x", output[idx]);
                    Serial.printf("\t\t");

                    uint8_t bframe2[7] = {0x00, 0x01, 0x61, 0xC8, 0x00, 0x02, 0x00};
                    // <-- Set packet here, excluding sequence number, then adjust length
                    std::vector frame2(bframe2, bframe2 + 7); // <-- adjust packet length
                    hexStringToBytes(controller_key, bcontroller);
                    iohcCrypto::create_1W_hmac(output, bseq, bcontroller, frame2);

                    Serial.printf("Close: f620");
                    for (unsigned char idx: bdest)
                        Serial.printf("%2.2x", idx);
                    for (unsigned char idx: bnode)
                        Serial.printf("%2.2x", idx);
                    for (unsigned char idx: bframe2) // <-- adjust packet length
                        Serial.printf("%2.2x", idx);
                    for (unsigned char idx: bseq)
                        Serial.printf("%2.2x", idx);
                    for (uint8_t idx = 0; idx < 6; idx++)
                        Serial.printf("%2.2x", output[idx]);
                    Serial.printf("\n");
                }
                break;
            }
            default:              
                IOHC::relStamp = esp_timer_get_time(); // 0x00: 0x1600 + target broadcast + source + 0x00 + Originator + ACEI + Main Param + FP1 + FP2 + sequence + hmac
                for (size_t typn = 0; typn < _type.size(); typn++) {
                    auto&packet = packets2send[typn];
                    _iohcRemote1W->init(packet, typn);
                    // Packet length
                    packet->payload.packet.header.CtrlByte1.asStruct.MsgLen += sizeof(_p0x00);
                    // Source (me)
                    for (size_t i = 0; i < sizeof(address); i++)
                        packet->payload.packet.header.source[i] = _node[i];
                    //Command
                    packet->payload.packet.header.cmd = 0x00;
                    packet->payload.packet.msg.p0x00.origin = 0x01; // Command Source Originator is: User
                    //Acei packet->payload.packet.msg.p0x00.acei;
                    setAcei(packet->payload.packet.msg.p0x00.acei, 0x43); //0xE7); //0x61);
                    switch (cmd) {
                        // Switch for Main Parameter of cmd 0x00: Open/Close/Stop/Ventilation
                        case RemoteButton::Open:
                            packet->payload.packet.msg.p0x00.main[0] = 0x00;
                            packet->payload.packet.msg.p0x00.main[1] = 0x00;
                            break;
                        case RemoteButton::Close:
                            packet->payload.packet.msg.p0x00.main[0] = 0xc8;
                            packet->payload.packet.msg.p0x00.main[1] = 0x00;
                            break;
                        case RemoteButton::Stop:
                            packet->payload.packet.msg.p0x00.main[0] = 0xd2;
                            packet->payload.packet.msg.p0x00.main[1] = 0x00;
                            break;
                        case RemoteButton::Vent:
                            packet->payload.packet.msg.p0x00.main[0] = 0xd8;
                            packet->payload.packet.msg.p0x00.main[1] = 0x03;
                            break;
                        case RemoteButton::ForceOpen:
                            packet->payload.packet.msg.p0x00.main[0] = 0x64;
                            packet->payload.packet.msg.p0x00.main[1] = 0x00;
                            break;
                        default: // If reaching default here, then cmd is not recognized, then return
                            return;
                    }
                    packet->payload.packet.msg.p0x00.fp1 = 0xAA;
                    packet->payload.packet.msg.p0x00.fp2 = 0x00;
                    // Sequence
                    packet->payload.packet.msg.p0x00.sequence[0] = _sequence >> 8;
                    packet->payload.packet.msg.p0x00.sequence[1] = _sequence & 0x00ff;
                    _sequence += 1;
                    // hmac
                    uint8_t hmac[6];
                    frame = std::vector(&packet->payload.packet.header.cmd, &packet->payload.packet.header.cmd + 7);
                    iohcCrypto::create_1W_hmac(hmac, packet->payload.packet.msg.p0x00.sequence, _key, frame);
                    for (uint8_t i = 0; i < 6; i++)
                        packet->payload.packet.msg.p0x00.hmac[i] = hmac[i];

                    digitalWrite(RX_LED, digitalRead(RX_LED) ^ 1);
                    packet->buffer_length = packet->payload.packet.header.CtrlByte1.asStruct.MsgLen + 1;
                    //                   packet->decode();

                    digitalWrite(RX_LED, digitalRead(RX_LED) ^ 1);
                }
                _radioInstance->send(packets2send);
                break;
        }
        save(); // Save sequence number
    }

    bool iohcRemote1W::load() {
        _radioInstance = iohcRadio::getInstance();

        if (LittleFS.exists(IOHC_1W_REMOTE))
            Serial.printf("Loading 1W remote settings from %s\n", IOHC_1W_REMOTE);
        else {
            Serial.printf("*1W remote not available\n");
            return false;
        }

        fs::File f = LittleFS.open(IOHC_1W_REMOTE, "r");
        DynamicJsonDocument doc(256);
        deserializeJson(doc, f);
        f.close();

        // Iterate through the JSON object
        for (JsonPair kv: doc.as<JsonObject>()) {
            hexStringToBytes(kv.key().c_str(), _node);
            auto jobj = kv.value().as<JsonObject>();
            hexStringToBytes(jobj["key"].as<const char *>(), _key);
            uint8_t btmp[2];
            hexStringToBytes(jobj["sequence"].as<const char *>(), btmp);
            _sequence = (btmp[0] << 8) + btmp[1];
            JsonArray jarr = jobj["type"];

            // Iterate through the JSON  type array
            for (size_t i = 0; i < jarr.size(); i++) {
                _type.insert(_type.begin() + i, jarr[i].as<uint16_t>());
                // packets2send[i] = new iohcPacket;
                packets2send.push_back(new iohcPacket);
            }
            _manufacturer = jobj["manufacturer_id"].as<uint8_t>();
        }

        return true;
    }

    bool iohcRemote1W::save() {
        fs::File f = LittleFS.open(IOHC_1W_REMOTE, "w+");
        DynamicJsonDocument doc(256);

        JsonObject jobj = doc.createNestedObject(bytesToHexString(_node, sizeof(_node)));
        jobj["key"] = bytesToHexString(_key, sizeof(_key));
        uint8_t btmp[2];
        btmp[1] = _sequence & 0x00ff;
        btmp[0] = _sequence >> 8;
        jobj["sequence"] = bytesToHexString(btmp, sizeof(btmp));
        JsonArray jarr = jobj.createNestedArray("type");
        for (size_t i = 0; i < _type.size(); i++)
            if (_type[i])
                jarr.add(_type.at(i));
            else
                break;
        jobj["manufacturer_id"] = _manufacturer;

        serializeJson(doc, f);
        f.close();

        return true;
    }
}
