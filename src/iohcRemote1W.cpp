#include <iohcRemote1W.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <iohcCryptoHelpers.h>


namespace IOHC
{
    iohcRemote1W *iohcRemote1W::_iohcRemote1W = nullptr;

    iohcRemote1W::iohcRemote1W()
    {
        load();
    }

    iohcRemote1W *iohcRemote1W::getInstance()
    {
        if (!_iohcRemote1W)
            _iohcRemote1W = new iohcRemote1W();
        return _iohcRemote1W;
    }

    void iohcRemote1W::cmd(RemoteButton cmd)   // Emulates remote button press
    {
        for (uint8_t typn=0; typn<_type.size(); typn++) // Pre-allocate packets vector; one packet for each remote type loaded
        {
            if (!packets2send[typn])
                packets2send[typn] = (IOHC::iohcPacket *)malloc(sizeof(IOHC::iohcPacket));
            else
                memset((void *)packets2send[typn]->payload.buffer, 0x00, sizeof(packets2send[typn]->payload.buffer));
        }

        switch (cmd)
        {
            case RemoteButton::Pair:   // 0x2e: 0x1120 + target broadcast + source + 0x2e00 + sequence + hmac
                for (uint8_t typn=0; typn<_type.size(); typn++)
                {
                    // Packet length
                    packets2send[typn]->payload.packet.header.framelength = sizeof(_header)+sizeof(_p0x2e)-1;
                    // Flags
                    packets2send[typn]->payload.packet.header.mode = 1;
                    packets2send[typn]->payload.packet.header.first = 1;
                    packets2send[typn]->payload.packet.header.last = 1;

                    packets2send[typn]->payload.packet.header.prot_v = 0;
                    packets2send[typn]->payload.packet.header._unq1 = 0;
                    packets2send[typn]->payload.packet.header._unq2 = 0;
                    packets2send[typn]->payload.packet.header.ack = 0;
                    packets2send[typn]->payload.packet.header.low_p = 1;
                    packets2send[typn]->payload.packet.header.routed = 0;
                    packets2send[typn]->payload.packet.header.use_beacon = 0;
                    // Source (me)
                    for (uint8_t i=0; i<sizeof(address); i++)
                        packets2send[typn]->payload.packet.header.source[i] = _node[i];
                    // Broadcast Target
                    uint16_t bcast = (_type.at(typn)<<6) + 0b111111;
                    packets2send[typn]->payload.packet.header.target[0] = 0x00;
                    packets2send[typn]->payload.packet.header.target[1] = bcast >> 8;
                    packets2send[typn]->payload.packet.header.target[2] = bcast & 0x00ff;

                    //Command
                    packets2send[typn]->payload.packet.header.cmd = 0x2e;
                    // Data
                    packets2send[typn]->payload.packet.msg.p0x2e.data = 0x00;
                    // Sequence
                    packets2send[typn]->payload.packet.msg.p0x2e.sequence[0] = _sequence >> 8;
                    packets2send[typn]->payload.packet.msg.p0x2e.sequence[1] = _sequence & 0x00ff;
                    _sequence += 1;
                    // hmac
                    uint8_t hmac[6];
                    std::vector<uint8_t> frame(&packets2send[typn]->payload.packet.header.cmd, &packets2send[typn]->payload.packet.header.cmd+2);
                    iohcUtils::create_1W_hmac(hmac, packets2send[typn]->payload.packet.msg.p0x2e.sequence, _key, frame);
                    for (uint8_t i=0; i < 6; i ++)
                        packets2send[typn]->payload.packet.msg.p0x2e.hmac[i] = hmac[i];


                    Serial.printf("Setup: ");
                    for (uint8_t x=0; x<=packets2send[typn]->payload.packet.header.framelength; x++)
                        Serial.printf("%2.2x", packets2send[typn]->payload.buffer[x]);
                    Serial.printf("\n");


                    digitalWrite(RX_LED, digitalRead(RX_LED)^1);
                    packets2send[typn]->buffer_length = packets2send[typn]->payload.packet.header.framelength +1;
                    packets2send[typn]->frequency = 0;
                    packets2send[typn]->millis = 25;
                    packets2send[typn]->repeat = 4;
                    packets2send[typn]->lock = true;
                    packets2send[typn+1] = nullptr;

                    digitalWrite(RX_LED, digitalRead(RX_LED)^1);
                    _radioInstance->send(packets2send);
                }
                break;

            case RemoteButton::Remove:   // 0x39: 0x1c00 + target broadcast + source + 0x3900 + sequence + hmac
                for (uint8_t typn=0; typn<_type.size(); typn++)
                {
                    // Packet length
                    packets2send[typn]->payload.packet.header.framelength = sizeof(_header)+sizeof(_p0x2e)-1;
                    // Flags
                    packets2send[typn]->payload.packet.header.mode = 1;
                    packets2send[typn]->payload.packet.header.first = 1;
                    packets2send[typn]->payload.packet.header.last = 1;

                    packets2send[typn]->payload.packet.header.prot_v = 0;
                    packets2send[typn]->payload.packet.header._unq1 = 0;
                    packets2send[typn]->payload.packet.header._unq2 = 0;
                    packets2send[typn]->payload.packet.header.ack = 0;
                    packets2send[typn]->payload.packet.header.low_p = 0;
                    packets2send[typn]->payload.packet.header.routed = 0;
                    packets2send[typn]->payload.packet.header.use_beacon = 0;
                    // Source (me)
                    for (uint8_t i=0; i<sizeof(address); i++)
                        packets2send[typn]->payload.packet.header.source[i] = _node[i];
                    // Broadcast Target
                    uint16_t bcast = (_type.at(typn)<<6) + 0b111111;
                    packets2send[typn]->payload.packet.header.target[0] = 0x00;
                    packets2send[typn]->payload.packet.header.target[1] = bcast >> 8;
                    packets2send[typn]->payload.packet.header.target[2] = bcast & 0x00ff;

                    //Command
                    packets2send[typn]->payload.packet.header.cmd = 0x39;
                    // Data
                    packets2send[typn]->payload.packet.msg.p0x2e.data = 0x00;
                    // Sequence
                    packets2send[typn]->payload.packet.msg.p0x2e.sequence[0] = _sequence >> 8;
                    packets2send[typn]->payload.packet.msg.p0x2e.sequence[1] = _sequence & 0x00ff;
                    _sequence += 1;
                    // hmac
                    uint8_t hmac[6];
                    std::vector<uint8_t> frame(&packets2send[typn]->payload.packet.header.cmd, &packets2send[typn]->payload.packet.header.cmd+2);
                    iohcUtils::create_1W_hmac(hmac, packets2send[typn]->payload.packet.msg.p0x2e.sequence, _key, frame);
                    for (uint8_t i=0; i < 6; i ++)
                        packets2send[typn]->payload.packet.msg.p0x2e.hmac[i] = hmac[i];


                    Serial.printf("Remove: ");
                    for (uint8_t x=0; x<=packets2send[typn]->payload.packet.header.framelength; x++)
                        Serial.printf("%2.2x", packets2send[typn]->payload.buffer[x]);
                    Serial.printf("\n");


                    digitalWrite(RX_LED, digitalRead(RX_LED)^1);
                    packets2send[typn]->buffer_length = packets2send[typn]->payload.packet.header.framelength +1;
                    packets2send[typn]->frequency = 0;
                    packets2send[typn]->millis = 25;
                    packets2send[typn]->repeat = 4;
                    packets2send[typn]->lock = true;
                    packets2send[typn+1] = nullptr;

                    digitalWrite(RX_LED, digitalRead(RX_LED)^1);
                    _radioInstance->send(packets2send);
                }
                break;

            case RemoteButton::Add:   // 0x30: 0x1100 + target broadcast + source + 0x3000 + ???
                for (uint8_t typn=0; typn<_type.size(); typn++)
                {
                    // Packet length
                    packets2send[typn]->payload.packet.header.framelength = sizeof(_header)+sizeof(_p0x30)-1;
                    // Flags
                    packets2send[typn]->payload.packet.header.mode = 1;
                    packets2send[typn]->payload.packet.header.first = 1;
                    packets2send[typn]->payload.packet.header.last = 1;

                    packets2send[typn]->payload.packet.header.prot_v = 0;
                    packets2send[typn]->payload.packet.header._unq1 = 0;
                    packets2send[typn]->payload.packet.header._unq2 = 0;
                    packets2send[typn]->payload.packet.header.ack = 0;
                    packets2send[typn]->payload.packet.header.low_p = 0;
                    packets2send[typn]->payload.packet.header.routed = 0;
                    packets2send[typn]->payload.packet.header.use_beacon = 0;
                    // Source (me)
                    for (uint8_t i=0; i<sizeof(address); i++)
                        packets2send[typn]->payload.packet.header.source[i] = _node[i];
                    // Broadcast Target
                    uint16_t bcast = (_type.at(typn)<<6) + 0b111111;
                    packets2send[typn]->payload.packet.header.target[0] = 0x00;
                    packets2send[typn]->payload.packet.header.target[1] = bcast >> 8;
                    packets2send[typn]->payload.packet.header.target[2] = bcast & 0x00ff;

                    //Command
                    packets2send[typn]->payload.packet.header.cmd = 0x30;

                    // Encrypted key
                    uint8_t encKey[16];
                    memcpy(encKey, _key, 16);
                    iohcUtils::encrypt_1W_key(_node, encKey);
                    memcpy(packets2send[typn]->payload.packet.msg.p0x30.enc_key, encKey, 16);

                    // Manufacturer
                    packets2send[typn]->payload.packet.msg.p0x30.man_id = _manufacturer;
                    // Data
                    packets2send[typn]->payload.packet.msg.p0x30.data = 0x01;
                    // Sequence
                    packets2send[typn]->payload.packet.msg.p0x30.sequence[0] = _sequence >> 8;
                    packets2send[typn]->payload.packet.msg.p0x30.sequence[1] = _sequence & 0x00ff;
                    _sequence += 1;

                    Serial.printf("Pair: ");
                    for (uint8_t x=0; x<=packets2send[typn]->payload.packet.header.framelength; x++)
                        Serial.printf("%2.2x", packets2send[typn]->payload.buffer[x]);
                    Serial.printf("\n");


                    digitalWrite(RX_LED, digitalRead(RX_LED)^1);
                    packets2send[typn]->buffer_length = packets2send[typn]->payload.packet.header.framelength +1;
                    packets2send[typn]->frequency = 0;
                    packets2send[typn]->millis = 25;
                    packets2send[typn]->repeat = 4;
                    packets2send[typn]->lock = true;
                    packets2send[typn+1] = nullptr;

                    digitalWrite(RX_LED, digitalRead(RX_LED)^1);
                    _radioInstance->send(packets2send);
                }
                break;

            default:   // 0x00: 0x1600 + target broadcast + source + 0x00 + Originator + ACEI + Main Param + FP1 + FP2 + sequence + hmac
                for (uint8_t typn=0; typn<_type.size(); typn++)
                {
                    // Packet length
                    packets2send[typn]->payload.packet.header.framelength = sizeof(_header)+sizeof(_p0x00)-1;
                    // Flags
                    packets2send[typn]->payload.packet.header.mode = 1;
                    packets2send[typn]->payload.packet.header.first = 1;
                    packets2send[typn]->payload.packet.header.last = 1;

                    packets2send[typn]->payload.packet.header.prot_v = 0;
                    packets2send[typn]->payload.packet.header._unq1 = 0;
                    packets2send[typn]->payload.packet.header._unq2 = 0;
                    packets2send[typn]->payload.packet.header.ack = 0;
                    packets2send[typn]->payload.packet.header.low_p = 1;
                    packets2send[typn]->payload.packet.header.routed = 0;
                    packets2send[typn]->payload.packet.header.use_beacon = 0;
                    // Source (me)
                    for (uint8_t i=0; i<sizeof(address); i++)
                        packets2send[typn]->payload.packet.header.source[i] = _node[i];
                    // Broadcast Target
                    uint16_t bcast = (_type.at(typn)<<6) + 0b111111;
                    packets2send[typn]->payload.packet.header.target[0] = 0x00;
                    packets2send[typn]->payload.packet.header.target[1] = bcast >> 8;
                    packets2send[typn]->payload.packet.header.target[2] = bcast & 0x00ff;

                    //Command
                    packets2send[typn]->payload.packet.header.cmd = 0x00;
                    packets2send[typn]->payload.packet.msg.p0x00.origin = 0x01; // Command Source Originator is: User
                    packets2send[typn]->payload.packet.msg.p0x00.acei = 0x61;

                    switch (cmd)    // Switch for Main Parameter of cmd 0x00: Open/Close/Stop/Ventilation
                    {
                        case RemoteButton::Open:
                            packets2send[typn]->payload.packet.msg.p0x00.main[0] = 0x00;
                            packets2send[typn]->payload.packet.msg.p0x00.main[1] = 0x00;
                            break;
                        case RemoteButton::Close:
                            packets2send[typn]->payload.packet.msg.p0x00.main[0] = 0xc8;
                            packets2send[typn]->payload.packet.msg.p0x00.main[1] = 0x00;
                            break;
                        case RemoteButton::Stop:
                            packets2send[typn]->payload.packet.msg.p0x00.main[0] = 0xd2;
                            packets2send[typn]->payload.packet.msg.p0x00.main[1] = 0x00;
                            break;
                        case RemoteButton::Vent:
                            packets2send[typn]->payload.packet.msg.p0x00.main[0] = 0xd8;
                            packets2send[typn]->payload.packet.msg.p0x00.main[1] = 0x03;
                            break;
                        case RemoteButton::ForceOpen:
                            packets2send[typn]->payload.packet.msg.p0x00.main[0] = 0x64;
                            packets2send[typn]->payload.packet.msg.p0x00.main[1] = 0x00;
                            break;
                        default:    // If reaching default here, then cmd is not recognized, then return
                            return;
                    }
                    packets2send[typn]->payload.packet.msg.p0x00.fp1 = 0x00;
                    packets2send[typn]->payload.packet.msg.p0x00.fp2 = 0x00;
                    // Sequence
                    packets2send[typn]->payload.packet.msg.p0x00.sequence[0] = _sequence >> 8;
                    packets2send[typn]->payload.packet.msg.p0x00.sequence[1] = _sequence & 0x00ff;
                    _sequence += 1;

                    // hmac
                    uint8_t hmac[6];
                    std::vector<uint8_t> frame(&packets2send[typn]->payload.packet.header.cmd, &packets2send[typn]->payload.packet.header.cmd+7);
                    iohcUtils::create_1W_hmac(hmac, packets2send[typn]->payload.packet.msg.p0x00.sequence, _key, frame);
                    for (uint8_t i=0; i < 6; i ++)
                        packets2send[typn]->payload.packet.msg.p0x00.hmac[i] = hmac[i];


                    Serial.printf("Command: ");
                    for (uint8_t x=0; x<=packets2send[typn]->payload.packet.header.framelength; x++)
                        Serial.printf("%2.2x", packets2send[typn]->payload.buffer[x]);
                    Serial.printf("\n");


                    digitalWrite(RX_LED, digitalRead(RX_LED)^1);
                    packets2send[typn]->buffer_length = packets2send[typn]->payload.packet.header.framelength +1;
                    packets2send[typn]->frequency = 0;
                    packets2send[typn]->millis = 25;
                    packets2send[typn]->repeat = 4;
                    packets2send[typn]->lock = true;
                    packets2send[typn+1] = nullptr;

                    digitalWrite(RX_LED, digitalRead(RX_LED)^1);
                    _radioInstance->send(packets2send);
                }
                break;
        }

        save(); // Save sequence number
    }

    bool iohcRemote1W::load(void)
    {
        if (LittleFS.exists(IOHC_1W_REMOTE))
            Serial.printf("Loading one way remote settings from %s\n", IOHC_1W_REMOTE);
        else
        {
            Serial.printf("*One way remote not available\n");
            return false;
        }

        fs::File f = LittleFS.open(IOHC_1W_REMOTE, "r");
        DynamicJsonDocument doc(256);
        deserializeJson(doc, f);
        f.close();

        // Iterate through the JSON object
        for (JsonPair kv : doc.as<JsonObject>())
        {
            hexStringToBytes(kv.key().c_str(), _node);
            JsonObject jobj = kv.value().as<JsonObject>();
            hexStringToBytes(jobj["key"].as<const char*>(), _key);
            uint8_t btmp[2];  
            hexStringToBytes(jobj["sequence"].as<const char*>(), btmp);
            _sequence = (btmp[0]<<8)+btmp[1];
            JsonArray jarr = jobj["type"];

            // Iterate through the JSON array
            for (uint8_t i=0; i<jarr.size(); i++)
                _type.insert(_type.begin()+i, jarr[i].as<uint16_t>());
            
            _manufacturer = jobj["manufacturer_id"].as<uint8_t>();
        }

        _radioInstance = IOHC::iohcRadio::getInstance();

        return true;
    }

    bool iohcRemote1W::save(void)
    {
        fs::File f = LittleFS.open(IOHC_1W_REMOTE, "w+");
        DynamicJsonDocument doc(256);

        JsonObject jobj = doc.createNestedObject(bytesToHexString(_node, sizeof(_node)));
        jobj["key"] = bytesToHexString(_key, sizeof(_key));
        uint8_t btmp[2];
        btmp[1] = _sequence & 0x00ff;
        btmp[0] = _sequence >> 8;
        jobj["sequence"] = bytesToHexString(btmp, sizeof(btmp));
        JsonArray jarr = jobj.createNestedArray("type");
        for (uint8_t i=0; i<_type.size(); i++)
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